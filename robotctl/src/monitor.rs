//! `robotctl monitor` — the live view of the control loop.
//!
//! Two renderings of one stream, chosen by where stdout goes:
//!
//!  - **a terminal**: a frame that repaints in place, so a number that matters can be
//!    *watched* rather than reconstructed from a thousand scrolled lines. Joint tracking
//!    lives here, which is the reason this exists: fifteen measured angles beside fifteen
//!    commanded ones is unreadable as text at 10 Hz, and is obvious as fifteen bars.
//!  - **anything else** — a pipe, a file, `--json`: one line per tick, exactly as before.
//!    `robotctl monitor > log` and `| grep` must keep working, and a screen-painting CLI
//!    that writes escape codes into a log file is a CLI nobody can script.
//!
//! The stream is read on its own thread. The socket read blocks, terminal events do not
//! arrive on the socket, and a UI that can only notice a keystroke when the robot happens
//! to send a frame stops responding the moment the robot does.

use std::collections::VecDeque;
use std::io::BufRead;
use std::path::Path;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, TryRecvError};
use std::thread;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use ratatui::DefaultTerminal;
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::layout::{Alignment, Constraint, Layout};
use ratatui::style::{Color, Modifier, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{
    Block, Cell, Paragraph, RenderDirection, Row, Sparkline, Table, TableState,
};

use crate::{Client, Failure, exit};

/// Tracking error at the edge of a deviation bar, radians.
///
/// A display scale, **not** a limit: nothing refuses a joint for exceeding it. Sized so the
/// bar spends its time in the middle at rest and swings visibly while walking — a bar that
/// is always saturated and one that never moves are equally uninformative.
const BAR_FULL_SCALE: f64 = 0.20;

/// Half-width of a deviation bar, in cells. The bar is `2 * HALF + 1` wide: zero has a
/// column of its own, because "no error" must not look like "a little to the left".
const BAR_HALF: usize = 12;

/// How much loop-rate history the trace keeps. More than any terminal is wide, so the
/// window is bounded by the display rather than by this number.
const TRACE_SAMPLES: usize = 600;

/// Redraw at least this often even with no new frame, so a stalled stream is visibly
/// stalled — the age in the header has to keep counting up.
const IDLE_REDRAW: Duration = Duration::from_millis(250);

/// What the reader thread has to say.
enum Update {
    State(Box<proto::RobotState>),
    /// The stream ended. Carries the sentence to exit with.
    Ended(String),
}

/// Subscribe to `robot.state` and render it until interrupted.
///
/// Never returns `Ok` on its own: `q`/`Ctrl-C` is the exit in the live view, Ctrl-C alone in
/// the piped one. A closed socket is an error either way — that is what `robotd` restarting
/// mid-update looks like, and it is worth seeing rather than hanging through.
pub fn run(robot_socket: &Path, hz: u32, json: bool) -> Result<(), Failure> {
    let mut client = Client::connect_to("robotd", robot_socket)?;
    let call = proto::Call::RobotSubscribe(proto::SubscribeParams {
        hz: (hz > 0).then_some(hz),
    });
    client.send(&proto::Request::call(proto::Id::Number(1), &call))?;

    if json || !stdout_is_a_terminal() {
        return stream_lines(client, json);
    }

    let Client { reader, writer, .. } = client;
    // Held, not dropped: it is the write half of the subscription. Closing it tells `robotd`
    // this client has gone away, which would end the stream we are about to render.
    let _writer = writer;

    let (tx, rx) = mpsc::channel();
    thread::spawn(move || read_states(reader, &tx));

    let mut terminal = ratatui::init();
    let outcome = live(&mut terminal, &rx, hz);
    ratatui::restore();
    outcome
}

/// One line per tick, for a pipe, a file, or `--json`.
fn stream_lines(mut client: Client, json: bool) -> Result<(), Failure> {
    let mut line = String::new();
    loop {
        line.clear();
        let read = client
            .reader
            .read_line(&mut line)
            .map_err(|e| Failure::new(exit::UNREACHABLE, format!("stream ended: {e}")))?;
        if read == 0 {
            return Err(Failure::new(
                exit::UNREACHABLE,
                "robotd closed the connection".to_owned(),
            ));
        }

        let Ok(request) = serde_json::from_str::<proto::Request>(&line) else {
            continue;
        };
        let Some(state) = request.as_state() else {
            // The subscribe acknowledgement, or anything else this client does not model.
            continue;
        };

        if json {
            println!("{}", line.trim_end());
            continue;
        }

        let limits = if state.movement.limited_by.is_empty() {
            String::new()
        } else {
            format!("  [{}]", state.movement.limited_by.join(","))
        };
        // Gravity and gain sit next to the fall verdict on purpose: `fallen` is derived from
        // the first and overrides the second, and reading the verdict without its input made
        // "the robot is down" indistinguishable from "the IMU frame is wrong".
        println!(
            "{:8.2}  {:>5}  {:5.1}Hz miss={:<4} {}  g[{:+.2} {:+.2} {:+.2}] kp={:<4} \
             req[{:+.2} {:+.2} {:+.2}] app[{:+.2} {:+.2} {:+.2}]{}",
            state.t,
            state.policy,
            state.control_loop.hz,
            state.control_loop.missed,
            if state.safety.fallen {
                "FALLEN"
            } else {
                "ok    "
            },
            state.safety.gravity[0],
            state.safety.gravity[1],
            state.safety.gravity[2],
            state
                .safety
                .gain
                .map_or_else(|| "-".to_owned(), |g| g.to_string()),
            state.movement.requested[0],
            state.movement.requested[1],
            state.movement.requested[2],
            state.movement.applied[0],
            state.movement.applied[1],
            state.movement.applied[2],
            limits,
        );
    }
}

/// Decode the stream on a thread of its own. Ends by describing how it ended, so the UI can
/// exit with the reason rather than with a channel that merely went quiet.
fn read_states(mut reader: impl BufRead, tx: &mpsc::Sender<Update>) {
    let mut line = String::new();
    loop {
        line.clear();
        let ended = match reader.read_line(&mut line) {
            Err(e) => format!("stream ended: {e}"),
            Ok(0) => "robotd closed the connection".to_owned(),
            Ok(_) => {
                if let Some(state) = serde_json::from_str::<proto::Request>(&line)
                    .ok()
                    .and_then(|request| request.as_state())
                    && tx.send(Update::State(Box::new(state))).is_err()
                {
                    return; // the UI is gone
                }
                continue;
            }
        };
        let _ = tx.send(Update::Ended(ended));
        return;
    }
}

/// The live view's loop: absorb whatever has arrived, honour the keyboard, repaint.
fn live(terminal: &mut DefaultTerminal, rx: &Receiver<Update>, hz: u32) -> Result<(), Failure> {
    // A frame every `1/hz`, so waiting a fifth of a period keeps the keyboard responsive
    // without spinning. Clamped: `--hz 1000` must not turn this into a busy loop.
    let poll = Duration::from_secs_f64(1.0 / f64::from(hz.clamp(1, 200)) / 5.0);
    let mut view = View::new(hz);
    let mut painted = Instant::now();

    loop {
        let mut fresh = false;
        while event::poll(Duration::ZERO).map_err(terminal_failure)? {
            match event::read().map_err(terminal_failure)? {
                Event::Key(key) if key.kind == KeyEventKind::Press => match key.code {
                    KeyCode::Char('q') | KeyCode::Esc => return Ok(()),
                    // Ctrl-C by hand: raw mode delivers it as a keypress rather than a signal,
                    // so the key that stops every other `robotctl` command has to stop this one
                    // too.
                    KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                        return Ok(());
                    }
                    // Scrolling only does anything on a terminal too short for every joint.
                    KeyCode::Up | KeyCode::Char('k') => {
                        view.scroll_by(-1);
                        fresh = true;
                    }
                    KeyCode::Down | KeyCode::Char('j') => {
                        view.scroll_by(1);
                        fresh = true;
                    }
                    KeyCode::PageUp => {
                        view.scroll_pages(-1);
                        fresh = true;
                    }
                    KeyCode::PageDown => {
                        view.scroll_pages(1);
                        fresh = true;
                    }
                    KeyCode::Home => {
                        view.scroll_home();
                        fresh = true;
                    }
                    _ => {}
                },
                // A resize changes what fits, and the next frame may be a whole period away.
                Event::Resize(_, _) => fresh = true,
                _ => {}
            }
        }

        match rx.recv_timeout(poll) {
            Ok(update) => fresh |= view.absorb(update)?,
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => {
                return Err(Failure::new(
                    exit::UNREACHABLE,
                    "the robot.state reader stopped".to_owned(),
                ));
            }
        }
        // Drain the backlog: on a slow terminal the newest frame is the only one worth
        // drawing, and rendering a queue one frame at a time is how a view falls behind and
        // stays behind.
        loop {
            match rx.try_recv() {
                Ok(update) => fresh |= view.absorb(update)?,
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => break,
            }
        }

        if fresh || painted.elapsed() >= IDLE_REDRAW {
            terminal
                .draw(|frame| view.render(frame))
                .map_err(terminal_failure)?;
            painted = Instant::now();
        }
    }
}

fn terminal_failure(e: std::io::Error) -> Failure {
    Failure::new(exit::FAILED, format!("terminal error: {e}"))
}

/// Everything on screen, plus the little history the trace needs.
struct View {
    /// Requested rate, for judging whether the stream has stalled.
    hz: u32,
    latest: Option<proto::RobotState>,
    /// When `latest` arrived, so its age can be shown. A frozen number with nothing saying
    /// it is frozen is the one failure mode a live view must not have.
    arrived: Option<Instant>,
    /// Achieved loop rate, **newest first** — the trace is drawn right to left, so the newest
    /// sample is the one at the right edge and history scrolls away to the left.
    trace: VecDeque<u64>,
    /// Best rate seen since the command started — the trace's full height. Never falls, so
    /// the baseline a dip is read against stays put.
    peak: u64,
    frames: u64,
    /// First joint row on screen. Only ever non-zero on a terminal too short for all of them,
    /// and it exists so that case is *navigable* rather than a table that quietly stops at the
    /// last row that happened to fit — half a leg, presented as the whole robot.
    scroll: usize,
    /// Joint rows the last frame had room for. Known only at render time, and kept because
    /// clamping the scroll and sizing a page both need it.
    visible: usize,
}

impl View {
    fn new(hz: u32) -> Self {
        Self {
            hz,
            latest: None,
            arrived: None,
            trace: VecDeque::with_capacity(TRACE_SAMPLES),
            peak: 0,
            frames: 0,
            scroll: 0,
            visible: 0,
        }
    }

    /// Move the joint window. Clamped on render, where the number of rows that fit is known.
    fn scroll_by(&mut self, rows: isize) {
        self.scroll = self.scroll.saturating_add_signed(rows);
    }

    /// A page is whatever the last frame had room for, so the keys mean the same thing on a
    /// terminal of any height.
    fn scroll_pages(&mut self, pages: isize) {
        self.scroll_by(pages * self.visible.max(1) as isize);
    }

    fn scroll_home(&mut self) {
        self.scroll = 0;
    }

    /// Take one update. `true` when the screen has something new to say.
    fn absorb(&mut self, update: Update) -> Result<bool, Failure> {
        match update {
            Update::Ended(why) => Err(Failure::new(exit::UNREACHABLE, why)),
            Update::State(state) => {
                if self.trace.len() == TRACE_SAMPLES {
                    self.trace.pop_back();
                }
                let rate = state.control_loop.hz.round().max(0.0) as u64;
                self.trace.push_front(rate);
                self.peak = self.peak.max(rate);
                self.frames += 1;
                self.arrived = Some(Instant::now());
                self.latest = Some(*state);
                Ok(true)
            }
        }
    }

    /// Has the stream gone quiet? Five periods, floored at half a second, so a slow
    /// `--hz 1` is not accused of stalling between two perfectly ordinary frames.
    fn stalled_for(&self) -> Option<Duration> {
        let age = self.arrived?.elapsed();
        let quiet = Duration::from_secs_f64(5.0 / f64::from(self.hz.clamp(1, 200)))
            .max(Duration::from_millis(500));
        (age > quiet).then_some(age)
    }

    fn render(&mut self, frame: &mut ratatui::Frame) {
        let area = frame.area();
        let Some(rows) = self.latest.as_ref().map(joint_rows) else {
            frame.render_widget(
                Paragraph::new("waiting for robot.state…")
                    .block(Block::bordered().title(" monitor "))
                    .dim(),
                area,
            );
            return;
        };

        // Split by hand rather than by constraint solving, because the order the space is
        // wanted in is a decision: the joints table gets the height it needs, the trace gets
        // what is left. Floored at three rows so the trace survives an 80×24 terminal — a joint
        // scrolled off the bottom is still reachable, whereas a missing loop rate is the one
        // number that says whether the others can be trusted at all. Capped at six because a
        // mostly-flat rate drawn twenty rows tall is a wall of ink, not more information.
        let trace_height = area.height.saturating_sub(6 + rows as u16 + 3).clamp(3, 6);
        let [header, joints, trace] = Layout::vertical([
            Constraint::Length(6),
            Constraint::Min(4),
            Constraint::Length(trace_height),
        ])
        .areas(area);

        // Two borders and the column header come off the table's height before any joint fits.
        self.visible = usize::from(joints.height.saturating_sub(3));
        self.scroll = self.scroll.min(rows.saturating_sub(self.visible));

        let (scroll, visible) = (self.scroll, self.visible);
        let state = self.latest.as_ref().expect("a state, checked above");

        frame.render_widget(self.header(state), header);
        frame.render_stateful_widget(
            self.joints(state, rows, visible),
            joints,
            &mut TableState::new().with_offset(scroll),
        );
        frame.render_widget(self.trace(), trace);
    }

    /// The whole-robot line block: who is driving, whether it is upright, what was asked
    /// for, and what actually went out.
    fn header(&self, state: &proto::RobotState) -> Paragraph<'_> {
        let missed = state.control_loop.missed;
        let mut top = vec![
            Span::raw("policy "),
            Span::styled(
                state.policy.clone(),
                Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!("   t {:.2} s   ", state.t)),
            Span::styled(
                format!("{:.1} Hz", state.control_loop.hz),
                Style::new().fg(Color::Cyan),
            ),
            Span::raw("   missed "),
            Span::styled(
                missed.to_string(),
                if missed > 0 {
                    Style::new().fg(Color::Yellow)
                } else {
                    Style::new().dim()
                },
            ),
        ];
        if let Some(age) = self.stalled_for() {
            top.push(Span::styled(
                format!("   STALLED {:.1}s", age.as_secs_f64()),
                Style::new().fg(Color::Red).add_modifier(Modifier::BOLD),
            ));
        }

        // `fallen` overrides the requested gain, and `limp` is how that override shows up at
        // the servos. Kept on one line with the gravity vector it was decided from: the
        // verdict alone cannot distinguish a robot on its side from an IMU mounted the way
        // this build does not expect.
        let mut safety = vec![Span::raw("safety "), fall_verdict(&state.safety)];
        if state.safety.limp {
            safety.push(Span::styled(" limp", Style::new().fg(Color::Yellow)));
        }
        safety.push(Span::raw(format!(
            "   g [{:+.2} {:+.2} {:+.2}]   kp {}",
            state.safety.gravity[0],
            state.safety.gravity[1],
            state.safety.gravity[2],
            state
                .safety
                .gain
                .map_or_else(|| "-".to_owned(), |g| g.to_string()),
        )));

        let mut movement = vec![Span::raw(format!(
            "move   req [{}]  →  app [{}]",
            triple(&state.movement.requested),
            triple(&state.movement.applied),
        ))];
        if !state.movement.limited_by.is_empty() {
            movement.push(Span::styled(
                format!("   limited by {}", state.movement.limited_by.join(", ")),
                Style::new().fg(Color::Yellow),
            ));
        }

        Paragraph::new(vec![
            Line::from(top),
            Line::from(movement),
            Line::from(safety),
            Line::from(format!(
                "head   [{:+.2} {:+.2} {:+.2} {:+.2}]",
                state.head[0], state.head[1], state.head[2], state.head[3]
            )),
        ])
        .block(
            Block::bordered()
                .title(" robot ")
                .title_top(Line::from(" q quits ").dim().right_aligned()),
        )
    }

    /// Measured against commanded, per joint, with the difference as a bar.
    ///
    /// The bar is the point. A servo that is not keeping up, a leg holding a load, a policy
    /// asking for something the joint cannot do — all of them are a column of numbers that
    /// look plausible, and a bar that is obviously off centre.
    fn joints(&self, state: &proto::RobotState, count: usize, visible: usize) -> Table<'_> {
        let rows = (0..count).map(|i| {
            let measured = state.joints.get(i).copied();
            let target = state.targets.get(i).copied();
            let error = measured.zip(target).map(|(m, t)| m - t);
            Row::new(vec![
                // A joint the wire has but this build cannot name: a `robotd` running a
                // model this `robotctl` predates. Show the index rather than dropping the
                // row — an unnamed joint is still a joint someone is debugging.
                Cell::from(
                    proto::JOINT_NAMES
                        .get(i)
                        .map_or_else(|| format!("joint {i}"), |name| (*name).to_owned()),
                ),
                Cell::from(Line::from(radians(measured)).right_aligned()),
                Cell::from(Line::from(radians(target)).right_aligned()),
                Cell::from(
                    Line::from(match error {
                        Some(e) => Span::styled(format!("{e:+.3}"), error_style(e)),
                        None => Span::raw("-").dim(),
                    })
                    .right_aligned(),
                ),
                Cell::from(deviation_bar(error)),
            ])
        });

        Table::new(
            rows,
            [
                Constraint::Length(15),
                Constraint::Length(8),
                Constraint::Length(8),
                Constraint::Length(8),
                Constraint::Min(BAR_HALF as u16 * 2 + 1),
            ],
        )
        .header(
            Row::new(vec!["joint", "measured", "target", "error", "deviation"]).style(
                Style::new()
                    .add_modifier(Modifier::BOLD)
                    .add_modifier(Modifier::DIM),
            ),
        )
        .column_spacing(1)
        .block(
            Block::bordered().title(" joints ").title_top(
                Line::from(self.window_note(count, visible))
                    .dim()
                    .right_aligned(),
            ),
        )
    }

    /// What the joints block says about itself on the right of its border: the bar's scale
    /// when every joint is on screen, and *which* joints are on screen when they are not.
    /// Never silently truncated — a table that stops mid-leg with nothing saying so is a
    /// display that lies.
    fn window_note(&self, count: usize, visible: usize) -> String {
        if visible >= count {
            return format!(" ±{BAR_FULL_SCALE:.2} rad full scale ");
        }
        let last = (self.scroll + visible).min(count);
        format!(
            " ±{BAR_FULL_SCALE:.2} rad · {}–{last} of {count} · ↑↓ scrolls ",
            self.scroll + 1
        )
    }

    /// Achieved loop rate over time.
    ///
    /// The instantaneous number in the header cannot show a *dropout*: a loop that fell to
    /// 20 Hz for half a second and recovered reads as 50 Hz by the time anybody looks. This
    /// is where a robot that stutters every few seconds becomes visible.
    fn trace(&self) -> Sparkline<'_> {
        let (low, high) = self
            .trace
            .iter()
            .fold((u64::MAX, 0), |(lo, hi), &v| (lo.min(v), hi.max(v)));
        let title = if self.trace.is_empty() {
            " loop rate ".to_owned()
        } else {
            format!(
                " loop rate · {low}–{high} Hz over {} frames · full height {} Hz · newest right ",
                self.frames, self.peak
            )
        };
        // Scaled to the best rate seen this session rather than to the tallest bar on screen.
        // An auto-scaled trace moves its own baseline as the window slides, and a loop running
        // uniformly at half speed then draws exactly like a healthy one.
        Sparkline::default()
            .data(self.trace.iter().copied())
            .direction(RenderDirection::RightToLeft)
            .max(self.peak.max(1))
            .style(Style::new().fg(Color::Cyan))
            .block(Block::bordered().title(Line::from(title).alignment(Alignment::Left)))
    }
}

/// How many joint rows this frame has. The names this build knows, extended by anything
/// extra the wire carried — a `robotd` speaking a longer joint vector than this `robotctl`
/// was built against must not have the tail silently dropped.
fn joint_rows(state: &proto::RobotState) -> usize {
    state
        .joints
        .len()
        .max(state.targets.len())
        .max(proto::JOINT_NAMES.len())
}

fn fall_verdict(safety: &proto::SafetyState) -> Span<'static> {
    if safety.fallen {
        Span::styled(
            "FALLEN",
            Style::new().fg(Color::Red).add_modifier(Modifier::BOLD),
        )
    } else {
        Span::styled("upright", Style::new().fg(Color::Green))
    }
}

fn triple(v: &[f64; 3]) -> String {
    format!("{:+.2} {:+.2} {:+.2}", v[0], v[1], v[2])
}

fn radians(v: Option<f64>) -> Span<'static> {
    match v {
        Some(v) => Span::raw(format!("{v:+.3}")),
        None => Span::raw("-").dim(),
    }
}

/// Green while the joint is where it was told to be, red once it plainly is not. Thresholds
/// are fractions of the bar's own scale, so the colour and the bar always agree.
fn error_style(error: f64) -> Style {
    let magnitude = error.abs() / BAR_FULL_SCALE;
    if !error.is_finite() {
        Style::new().fg(Color::Magenta).add_modifier(Modifier::BOLD)
    } else if magnitude < 0.25 {
        Style::new().fg(Color::Green)
    } else if magnitude < 0.6 {
        Style::new().fg(Color::Yellow)
    } else {
        Style::new().fg(Color::Red)
    }
}

/// Tracking error as a bar growing from a fixed centre, left for negative, right for
/// positive.
///
/// Centred rather than left-anchored because the sign is diagnostic: a knee that lags
/// behind its target and a knee that overshoots it are different faults, and a bar drawn
/// from the left end makes them the same picture. Saturation is marked, so an error off the
/// scale cannot be mistaken for one that merely reaches the edge.
fn deviation_bar(error: Option<f64>) -> Line<'static> {
    let Some(error) = error.filter(|e| e.is_finite()) else {
        return Line::from(Span::raw("no reading").dim());
    };

    let cells = (error.abs() / BAR_FULL_SCALE * BAR_HALF as f64).round();
    let saturated = cells > BAR_HALF as f64;
    let filled = (cells as usize).min(BAR_HALF);
    let style = error_style(error);
    // A dim rail rather than blank space: the bar's extent is what makes its length mean
    // something, and a bar with no visible track is a number drawn in a different font.
    let pad = |n: usize| Span::raw("·".repeat(n)).dim();
    let bar = |n: usize| Span::styled("█".repeat(n), style);
    let edge = |mark: &'static str, on: bool| {
        if on {
            Span::styled(mark, style.add_modifier(Modifier::BOLD))
        } else {
            Span::raw(" ")
        }
    };

    if error < 0.0 {
        Line::from(vec![
            edge("«", saturated),
            pad(BAR_HALF - filled),
            bar(filled),
            Span::raw("│").dim(),
            pad(BAR_HALF),
            edge(" ", false),
        ])
    } else {
        Line::from(vec![
            edge(" ", false),
            pad(BAR_HALF),
            Span::raw("│").dim(),
            bar(filled),
            pad(BAR_HALF - filled),
            edge("»", saturated),
        ])
    }
}

/// Is stdout a terminal? Decides which of the two renderings runs.
fn stdout_is_a_terminal() -> bool {
    // SAFETY: `isatty` only inspects a file descriptor; it touches no memory of ours.
    unsafe { libc::isatty(libc::STDOUT_FILENO) == 1 }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every bar is the same width whatever the error, or the table's columns dance from
    /// frame to frame and the whole display becomes unreadable while walking.
    #[test]
    fn deviation_bars_are_all_one_width() {
        let width = |e: Option<f64>| {
            deviation_bar(e)
                .spans
                .iter()
                .map(|s| s.content.chars().count())
                .sum::<usize>()
        };
        let expected = 2 * BAR_HALF + 3;
        for error in [0.0, 0.01, -0.01, BAR_FULL_SCALE, -BAR_FULL_SCALE, 5.0, -5.0] {
            assert_eq!(width(Some(error)), expected, "error {error}");
        }
    }

    /// An error past the scale is marked as such. Without this a saturated bar and a bar at
    /// exactly full scale are the same picture, and "how far past" stops being askable.
    #[test]
    fn an_error_off_the_scale_is_marked() {
        let text = |e: f64| {
            deviation_bar(Some(e))
                .spans
                .iter()
                .map(|s| s.content.to_string())
                .collect::<String>()
        };
        assert!(text(5.0).contains('»'), "{}", text(5.0));
        assert!(text(-5.0).contains('«'), "{}", text(-5.0));
        assert!(!text(BAR_FULL_SCALE).contains('»'));
        assert!(!text(0.0).contains('«'));
    }

    /// A missing reading says so rather than drawing a centred bar, which would claim the
    /// joint is tracking perfectly.
    #[test]
    fn a_missing_reading_draws_no_bar() {
        let line = deviation_bar(None);
        let text: String = line.spans.iter().map(|s| s.content.to_string()).collect();
        assert_eq!(text, "no reading");
        assert!(!deviation_bar(Some(f64::NAN)).spans.is_empty());
        let nan: String = deviation_bar(Some(f64::NAN))
            .spans
            .iter()
            .map(|s| s.content.to_string())
            .collect();
        assert_eq!(nan, "no reading");
    }

    /// The bar's centre is the zero column: no error must not look like a small one.
    #[test]
    fn zero_error_fills_nothing() {
        let text: String = deviation_bar(Some(0.0))
            .spans
            .iter()
            .map(|s| s.content.to_string())
            .collect();
        assert!(!text.contains('█'), "{text}");
    }

    /// A stream that has gone quiet is reported as such, and one arriving on time is not.
    #[test]
    fn a_quiet_stream_is_called_stalled() {
        let mut view = View::new(50);
        assert_eq!(view.stalled_for(), None, "nothing has arrived yet");

        view.arrived = Some(Instant::now());
        assert_eq!(view.stalled_for(), None, "just arrived");

        view.arrived = Some(Instant::now() - Duration::from_secs(2));
        assert!(view.stalled_for().is_some());
    }

    /// The trace is bounded. It is fed at the loop rate for as long as the command runs,
    /// which is the shape of an unbounded buffer if nothing trims it.
    #[test]
    fn the_trace_does_not_grow_without_end() {
        let mut view = View::new(50);
        for _ in 0..TRACE_SAMPLES + 50 {
            assert!(view.absorb(Update::State(Box::new(a_state()))).is_ok());
        }
        assert_eq!(view.trace.len(), TRACE_SAMPLES);
        assert_eq!(view.frames as usize, TRACE_SAMPLES + 50);
    }

    /// Every joint is named and numbered on a terminal with room for all of them, and the
    /// block says nothing about a window because there is nothing hidden.
    #[test]
    fn a_tall_terminal_shows_every_joint() {
        let screen = draw(96, 32, &a_state(), 0);

        for name in proto::JOINT_NAMES {
            assert!(screen.contains(name), "{name} is missing:\n{screen}");
        }
        assert!(screen.contains("full scale"), "{screen}");
        assert!(!screen.contains("scrolls"), "{screen}");
    }

    /// A terminal too short for fifteen joints says which ones it is showing. The failure this
    /// guards against is silent: a table that stops at the last row that fits, with the rest of
    /// the robot simply absent from a display someone is trusting.
    #[test]
    fn a_short_terminal_says_what_it_is_hiding() {
        let screen = draw(96, 24, &a_state(), 0);

        assert!(screen.contains("of 15"), "{screen}");
        assert!(screen.contains("↑↓ scrolls"), "{screen}");
        assert!(!screen.contains("right_ankle"), "no room for the last row");

        // Scrolled to the end, the last joint is on screen and the first is not.
        let scrolled = draw(96, 24, &a_state(), 99);
        assert!(scrolled.contains("right_ankle"), "{scrolled}");
        assert!(!scrolled.contains("left_hip_yaw"), "{scrolled}");
    }

    /// Scrolling stops at the last joint rather than running the table off the screen.
    #[test]
    fn scrolling_stops_at_the_end() {
        let mut view = View::new(20);
        assert!(view.absorb(Update::State(Box::new(a_state()))).is_ok());
        view.scroll_by(99);
        render_to(&mut view, 96, 24);
        let rows = proto::JOINT_NAMES.len();
        assert_eq!(view.scroll, rows - view.visible);
    }

    /// A fall is the loudest thing on the screen, and the gravity vector it was decided from
    /// is next to it — the verdict alone cannot tell a robot on its side from a rotated IMU.
    #[test]
    fn a_fallen_robot_says_so_beside_its_gravity() {
        let mut state = a_state();
        state.safety.fallen = true;
        state.safety.limp = true;
        state.movement.limited_by = vec!["fallen".to_owned()];

        let screen = draw(96, 32, &state, 0);
        assert!(screen.contains("FALLEN"), "{screen}");
        assert!(screen.contains("limp"), "{screen}");
        assert!(screen.contains("+0.00 +0.00 -1.00"), "{screen}");
        assert!(screen.contains("limited by fallen"), "{screen}");
    }

    /// Render one frame and return it as text.
    fn draw(width: u16, height: u16, state: &proto::RobotState, scroll: usize) -> String {
        let mut view = View::new(20);
        assert!(
            view.absorb(Update::State(Box::new(state.clone()))).is_ok(),
            "a state is not a failure"
        );
        view.scroll_by(scroll as isize);
        render_to(&mut view, width, height)
    }

    fn render_to(view: &mut View, width: u16, height: u16) -> String {
        let mut terminal =
            ratatui::Terminal::new(ratatui::backend::TestBackend::new(width, height))
                .expect("a test backend never fails to initialise");
        terminal
            .draw(|frame| view.render(frame))
            .expect("nor does drawing to one");
        let buffer = terminal.backend().buffer();
        (0..buffer.area.height)
            .map(|y| {
                (0..buffer.area.width)
                    .map(|x| buffer[(x, y)].symbol())
                    .collect::<String>()
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// The end of the stream becomes the command's exit code, not a blank screen.
    #[test]
    fn the_stream_ending_is_an_unreachable_failure() {
        let mut view = View::new(50);
        let failure = view
            .absorb(Update::Ended("robotd closed the connection".to_owned()))
            .expect_err("ending the stream is a failure");
        assert_eq!(failure.code, exit::UNREACHABLE);
        assert_eq!(failure.message, "robotd closed the connection");
    }

    fn a_state() -> proto::RobotState {
        proto::RobotState {
            t: 1.0,
            movement: proto::MoveState {
                requested: [0.0; 3],
                applied: [0.0; 3],
                limited_by: vec![],
            },
            head: [0.0; 4],
            policy: "stand".to_owned(),
            safety: proto::SafetyState {
                fallen: false,
                limp: false,
                gravity: [0.0, 0.0, -1.0],
                gain: Some(32),
            },
            control_loop: proto::LoopState {
                hz: 50.0,
                missed: 0,
            },
            joints: vec![0.0; proto::JOINT_NAMES.len()],
            targets: vec![0.0; proto::JOINT_NAMES.len()],
        }
    }
}
