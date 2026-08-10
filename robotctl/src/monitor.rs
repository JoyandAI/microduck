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
///
/// Kept in radians whatever the display is set to: it is compared against a wire value, and a
/// constant that changes unit with a keypress is a threshold nobody can reason about.
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

/// Rows the robot block occupies: two borders, a four-row half for the command and the IMU
/// side by side, then the limits and the head. Fixed, because a header that grows when a
/// limit appears would shift every joint row down at the moment the reader is staring at one.
const HEADER_HEIGHT: u16 = 8;

/// Request id for the subscribe call, so its answer can be told apart from the stream that
/// follows it on the same connection.
const SUBSCRIBE_ID: u64 = 1;

/// Which unit the angles on screen are drawn in.
///
/// The wire is radians and stays radians: [`proto::RobotState`] carries nothing else, and the
/// piped rendering keeps them, because a script parsing that output must not have its numbers
/// change under it. This is a reading aid for the live view alone — a hip at `-0.52` and a hip
/// at `-30°` are the same joint, and only one of them can be pictured without arithmetic.
///
/// Radians are still one keypress away rather than gone, because they are what every other
/// surface speaks: the protocol docs, a policy's own inputs, and the numbers a client sends.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Units {
    Degrees,
    Radians,
}

impl Units {
    /// The other one. There are exactly two, so this is the whole of the `u` keybinding.
    fn toggled(self) -> Self {
        match self {
            Self::Degrees => Self::Radians,
            Self::Radians => Self::Degrees,
        }
    }

    /// One wire angle as text.
    ///
    /// Two decimals of a degree is finer than three of a radian, so the display resolves at
    /// least as much either way — flipping the unit must not quietly hide a difference the
    /// other unit was showing.
    fn angle(self, radians: f64) -> String {
        match self {
            Self::Degrees => format!("{:+.2}°", radians.to_degrees()),
            Self::Radians => format!("{radians:+.3}"),
        }
    }

    /// The same, for a rate: the twist's yaw is per second.
    fn rate(self, radians_per_second: f64) -> f64 {
        match self {
            Self::Degrees => radians_per_second.to_degrees(),
            Self::Radians => radians_per_second,
        }
    }

    fn rate_unit(self) -> &'static str {
        match self {
            Self::Degrees => "°/s",
            Self::Radians => "rad/s",
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::Degrees => "degrees",
            Self::Radians => "radians",
        }
    }

    /// The deviation bar's full scale, said in whatever is on screen. The bar is drawn from a
    /// ratio, so only its caption has a unit at all.
    fn bar_scale(self) -> String {
        match self {
            Self::Degrees => format!("±{:.1}°", BAR_FULL_SCALE.to_degrees()),
            Self::Radians => format!("±{BAR_FULL_SCALE:.2} rad"),
        }
    }
}

/// What the reader thread has to say.
enum Update {
    State(Box<proto::RobotState>),
    /// The subscribe acknowledgement, which names the policy this `robotd` is running.
    Policy(Box<proto::SubscribeResult>),
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
    client.send(&proto::Request::call(
        proto::Id::Number(SUBSCRIBE_ID),
        &call,
    ))?;

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
                if let Some(update) = decode(&line)
                    && tx.send(update).is_err()
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

/// Read one line of the connection.
///
/// Two shapes arrive on it: the answer to `robot.subscribe`, once, and `robot.state`
/// notifications forever after. A notification carries `method`, a response does not, so the
/// two cannot be confused — and anything else is ignored rather than treated as an error,
/// because a `robotd` newer than this build may say things this one has no use for.
fn decode(line: &str) -> Option<Update> {
    if let Ok(request) = serde_json::from_str::<proto::Request>(line) {
        return request.as_state().map(|s| Update::State(Box::new(s)));
    }
    let response = serde_json::from_str::<proto::Response>(line).ok()?;
    if response.id != Some(proto::Id::Number(SUBSCRIBE_ID)) {
        return None;
    }
    // A `robotd` that predates `SubscribeResult` answered with `IntentResult`, whose
    // `accepted` field this parses and whose missing policy fields stay `None` — so an old
    // robot reports an unknown policy rather than failing to render.
    response
        .result_as::<proto::SubscribeResult>()
        .ok()
        .map(|r| Update::Policy(Box::new(r)))
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
                    // Radians back, for reading a number that is about to be compared with the
                    // wire, a policy's input, or the protocol docs.
                    KeyCode::Char('u') => {
                        view.toggle_units();
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
    /// Which policy this `robotd` is running, from the subscribe acknowledgement. `None`
    /// until it arrives, and on a `robotd` too old to say.
    policy: Option<proto::SubscribeResult>,
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
    /// What every angle on screen is drawn in. Degrees to start with: this view is read while
    /// looking at the robot, and a leg is a picture in degrees and arithmetic in radians.
    units: Units,
}

impl View {
    fn new(hz: u32) -> Self {
        Self {
            hz,
            policy: None,
            latest: None,
            arrived: None,
            trace: VecDeque::with_capacity(TRACE_SAMPLES),
            peak: 0,
            frames: 0,
            scroll: 0,
            visible: 0,
            units: Units::Degrees,
        }
    }

    fn toggle_units(&mut self) {
        self.units = self.units.toggled();
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
            Update::Policy(policy) => {
                self.policy = Some(*policy);
                Ok(true)
            }
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
        let trace_height = area
            .height
            .saturating_sub(HEADER_HEIGHT + rows as u16 + 3)
            .clamp(3, 6);
        let [header, joints, trace] = Layout::vertical([
            Constraint::Length(HEADER_HEIGHT),
            Constraint::Min(4),
            Constraint::Length(trace_height),
        ])
        .areas(area);

        // Two borders and the column header come off the table's height before any joint fits.
        self.visible = usize::from(joints.height.saturating_sub(3));
        self.scroll = self.scroll.min(rows.saturating_sub(self.visible));

        let (scroll, visible) = (self.scroll, self.visible);
        let state = self.latest.as_ref().expect("a state, checked above");

        self.render_header(frame, header, state);
        frame.render_stateful_widget(
            self.joints(state, rows, visible),
            joints,
            &mut TableState::new().with_offset(scroll),
        );
        frame.render_widget(self.trace(), trace);
    }

    /// The whole-robot block: what was asked of it, what it did, and what it can feel.
    ///
    /// Everything is spelled out — axis names, units, the sense of each direction — because
    /// the previous version wrote `req [+0.30 +0.00 +0.10]` and a reader had to already know
    /// that a velocity twist is `vx, vy, vyaw`, that the numbers are m/s and rad/s, and which
    /// way positive turns. That convention is written down in `duck-ipc-proto`, and a display
    /// that omits it makes every reader re-derive it — which is the exact failure the protocol
    /// docs name as the reason the prototype grew five sign-flip flags.
    fn render_header(
        &self,
        frame: &mut ratatui::Frame,
        area: ratatui::layout::Rect,
        state: &proto::RobotState,
    ) {
        // The policy's identity goes on the bottom border: it never changes while the command
        // runs, so it belongs where a caption belongs rather than taking a row from the joints.
        let block = Block::bordered()
            .title(Line::from(self.title(state)))
            .title_top(
                // The units key is named next to the others because a reader who does not know
                // it exists has no way to discover that the numbers could be radians instead.
                Line::from(format!(
                    " q quits · ↑↓ scrolls · u {} ",
                    self.units.toggled().name()
                ))
                .dim()
                .right_aligned(),
            )
            .title_bottom(Line::from(self.policy_caption()));
        let inner = block.inner(area);
        frame.render_widget(block, area);

        // Command on the left, sensing on the right: two four-row columns rather than eight
        // stacked rows, because every row here is a row the joints table does not get.
        let [top, bottom] =
            Layout::vertical([Constraint::Length(4), Constraint::Length(2)]).areas::<2>(inner);
        let [asked, felt] =
            Layout::horizontal([Constraint::Percentage(52), Constraint::Percentage(48)])
                .areas::<2>(top);

        frame.render_widget(self.movement(state), asked);
        frame.render_widget(self.imu(state), felt);
        frame.render_widget(self.limits_and_head(state), bottom);
    }

    /// Which network is driving this robot, from the subscribe acknowledgement.
    ///
    /// [`proto::RobotState::policy`] says `walk`, `stand` or `held` — the mode of *this tick*.
    /// That is not the same question as which policy is loaded, and it was the only one this
    /// view could answer: two releases with different gaits both say `walk`, and "which
    /// network is this?" is the first thing anyone comparing them asks.
    fn policy_caption(&self) -> Vec<Span<'static>> {
        let Some(policy) = self.policy.as_ref() else {
            // No acknowledgement yet, or a `robotd` that predates it. Said out loud, because
            // the alternative is a caption that looks like a robot with no policy.
            return vec![Span::raw(" policy · not reported by robotd ").dim()];
        };

        let mut caption = vec![Span::raw(" policy · ").dim()];
        match policy.walk.as_deref() {
            Some(walk) => {
                caption.push(Span::styled(
                    walk.to_owned(),
                    Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
                ));
                if let Some(stand) = policy.stand.as_deref() {
                    caption.push(Span::raw(" · standing ").dim());
                    caption.push(Span::styled(stand.to_owned(), Style::new().fg(Color::Cyan)));
                } else {
                    // Not an omission: without a standing network the walking one runs at
                    // every velocity, which changes how the robot behaves at rest.
                    caption.push(Span::raw(" · no standing policy").dim());
                }
            }
            None => caption.push(Span::raw("none loaded").dim()),
        }
        if let Some(why) = policy.unavailable.as_deref() {
            caption.push(Span::styled(
                format!(" — {why}"),
                Style::new().fg(Color::Yellow),
            ));
        }
        caption.push(Span::raw(" ").dim());
        caption
    }

    /// The block's own title: who is driving, how fast the loop is going, and whether the
    /// frame on screen is still arriving.
    fn title(&self, state: &proto::RobotState) -> Vec<Span<'static>> {
        let missed = state.control_loop.missed;
        let mut title = vec![
            Span::raw(" policy "),
            Span::styled(
                state.policy.clone(),
                Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!(" · t {:.2} s · ", state.t)),
            Span::styled(
                format!("{:.1} Hz", state.control_loop.hz),
                Style::new().fg(Color::Cyan),
            ),
            Span::raw(" · missed "),
            Span::styled(
                missed.to_string(),
                if missed > 0 {
                    Style::new().fg(Color::Yellow)
                } else {
                    Style::new().dim()
                },
            ),
            Span::raw(" "),
        ];
        if let Some(age) = self.stalled_for() {
            title.push(Span::styled(
                format!("· STALLED {:.1}s ", age.as_secs_f64()),
                Style::new().fg(Color::Red).add_modifier(Modifier::BOLD),
            ));
        }
        title
    }

    /// The velocity twist, one labelled row per axis: what a client asked for, and what
    /// actually reached the policy after safety had its say.
    fn movement(&self, state: &proto::RobotState) -> Paragraph<'static> {
        // `angular` says whether this axis is a turn rate: the two linear ones are m/s in any
        // unit setting, and converting them would be nonsense dressed as consistency.
        let axis = |name: &str, sense: &str, unit: &str, i: usize, angular: bool| {
            let (asked, applied) = (state.movement.requested[i], state.movement.applied[i]);
            // Highlight the difference, not the pair: "asked for 0.3, got 0.15" is the whole
            // reason this command exists, and it is invisible when both numbers look alike.
            // Judged on the wire values, so the same clamp reads the same in either unit.
            let style = if (asked - applied).abs() > 1e-6 {
                Style::new().fg(Color::Yellow)
            } else {
                Style::new()
            };
            let show = |v: f64| if angular { self.units.rate(v) } else { v };
            let (asked, applied) = (show(asked), show(applied));
            Line::from(vec![
                Span::raw(format!(" {name:<5}{sense:<10}{asked:>+7.2}")),
                Span::styled(format!("{applied:>+8.2}"), style),
                Span::raw(format!("  {unit}")).dim(),
            ])
        };

        Paragraph::new(vec![
            Line::from(" move                asked applied").dim(),
            axis("vx", "forward", "m/s", 0, false),
            axis("vy", "left", "m/s", 1, false),
            axis("vyaw", "turn left", self.units.rate_unit(), 2, true),
        ])
    }

    /// What the IMU is telling the robot, and the verdict drawn from it.
    ///
    /// Projected gravity is the only IMU quantity on this stream — `robot.health` has the
    /// stale-read counters — and it is here rather than reduced to `fallen` because the
    /// verdict alone cannot tell a robot lying on its side from an IMU mounted the way this
    /// build does not expect. Upright is about `[0, 0, -1]`.
    fn imu(&self, state: &proto::RobotState) -> Paragraph<'static> {
        let axis = |name: &str, i: usize| {
            Line::from(format!(" {name:<11}{:>+6.2}", state.safety.gravity[i]))
        };
        let mut down = axis("z up", 2).spans;
        down.push(Span::raw("   "));
        down.push(fall_verdict(&state.safety));

        Paragraph::new(vec![
            Line::from(" imu · gravity in the trunk frame").dim(),
            axis("x forward", 0),
            axis("y left", 1),
            Line::from(down),
        ])
    }

    /// Two rows that are always present whether or not they have anything to report, because
    /// a header that changes height moves the joints table under the reader's eyes.
    fn limits_and_head(&self, state: &proto::RobotState) -> Paragraph<'static> {
        let limits = if state.movement.limited_by.is_empty() {
            Line::from(" limits  none — the command went through untouched").dim()
        } else {
            // Explained, not just named: `deadman` is a token the reader has to look up, and
            // the sentence is the thing they were looking it up for.
            Line::from(vec![
                Span::raw(" limits  "),
                Span::styled(
                    state
                        .movement
                        .limited_by
                        .iter()
                        .map(|l| explain_limit(l))
                        .collect::<Vec<_>>()
                        .join("; "),
                    Style::new().fg(Color::Yellow),
                ),
            ])
        };

        let angle = |i: usize| self.units.angle(state.head[i]);
        let mut head = vec![Span::raw(format!(
            " head    neck_pitch {}  head_pitch {}  head_yaw {}  head_roll {}",
            angle(0),
            angle(1),
            angle(2),
            angle(3)
        ))];
        // Degrees carry their own `°`; radians are bare, and so have to be named here or the
        // row is four numbers in no unit at all.
        head.push(
            Span::raw(match self.units {
                Units::Degrees => "   ",
                Units::Radians => " rad   ",
            })
            .dim(),
        );
        // The gain the servos are actually running at, next to `limp`, which is what a gain
        // that safety has overridden looks like from the outside.
        head.push(Span::raw(format!(
            "kp {}",
            state
                .safety
                .gain
                .map_or_else(|| "-".to_owned(), |g| g.to_string())
        )));
        if state.safety.limp {
            head.push(Span::styled(
                " limp — gains dropped so the robot yields",
                Style::new().fg(Color::Yellow),
            ));
        }

        Paragraph::new(vec![limits, Line::from(head)])
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
                Cell::from(Line::from(self.angle(measured)).right_aligned()),
                Cell::from(Line::from(self.angle(target)).right_aligned()),
                Cell::from(
                    Line::from(match error {
                        Some(e) => Span::styled(self.units.angle(e), error_style(e)),
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
                // Wide enough for a degree: `-123.45°` is two characters longer than the
                // `-2.155` a radian needs, and a column that fits one unit but truncates the
                // other turns the toggle into a way to lose digits.
                Constraint::Length(15),
                Constraint::Length(9),
                Constraint::Length(9),
                Constraint::Length(9),
                Constraint::Min(BAR_HALF as u16 * 2 + 1),
            ],
        )
        .header(
            // "commanded", not "target": the wire calls it `targets`, but next to a *measured*
            // angle the word target reads as a goal the robot is working towards rather than
            // the number that was written to the servo on this very tick.
            Row::new(vec!["joint", "measured", "commanded", "error", "deviation"]).style(
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

    /// One joint angle, or a dash where the wire carried none.
    fn angle(&self, v: Option<f64>) -> Span<'static> {
        match v {
            Some(v) => Span::raw(self.units.angle(v)),
            None => Span::raw("-").dim(),
        }
    }

    /// What the joints block says about itself on the right of its border: the bar's scale
    /// when every joint is on screen, and *which* joints are on screen when they are not.
    /// Never silently truncated — a table that stops mid-leg with nothing saying so is a
    /// display that lies.
    fn window_note(&self, count: usize, visible: usize) -> String {
        let (unit, scale) = (self.units.name(), self.units.bar_scale());
        if visible >= count {
            return format!(" {unit} · bar reaches {scale} ");
        }
        let last = (self.scroll + visible).min(count);
        format!(
            " {unit} · bar {scale} · {}–{last} of {count} · ↑↓ scrolls ",
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

/// Say what a limit *means*, not just what it is called.
///
/// The wire carries `duck_control::safety::Limit`'s names — `deadman`, `joint_range`,
/// `not_finite`, `fallen` — and each one is a token whose meaning lives in a doc comment in
/// another crate. Anything unrecognised is passed through verbatim: a `robotd` newer than this
/// `robotctl` may have limits this build has never heard of, and printing the raw name is
/// strictly better than hiding it.
fn explain_limit(limit: &str) -> String {
    match limit {
        "deadman" => "deadman — no intent arrived recently, velocity zeroed".to_owned(),
        "joint_range" => "joint_range — a target was outside the actuator's travel".to_owned(),
        "not_finite" => "not_finite — a target was NaN or infinite".to_owned(),
        "fallen" => "fallen — the robot is down, the policy is not driving".to_owned(),
        other => other.to_owned(),
    }
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
        // The columns carry a unit, and the bar carries the scale it is drawn to.
        assert!(screen.contains("degrees"), "{screen}");
        assert!(screen.contains("bar reaches ±11.5°"), "{screen}");
        assert!(
            !screen.contains("of 15"),
            "nothing is hidden, so nothing is counted:\n{screen}"
        );
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
        // The verdict sits beside the axis it was decided from, each named.
        assert!(screen.contains("z up"), "{screen}");
        assert!(screen.contains("-1.00"), "{screen}");
        // And the limit is explained, not just named.
        assert!(
            screen.contains("fallen — the robot is down"),
            "the limit is spelled out:\n{screen}"
        );
    }

    /// Every number in the header names itself: the axis, which way is positive, and the unit.
    /// A bare `req [+0.30 +0.00 +0.10]` needs the reader to already know it is a velocity twist
    /// in m/s and rad/s, which is exactly the convention `duck-ipc-proto` documents *because*
    /// leaving it implicit is how the prototype grew five sign-flip flags.
    #[test]
    fn the_header_labels_its_axes_and_units() {
        let mut state = a_state();
        state.movement.requested = [0.30, 0.0, 0.10];
        state.movement.applied = [0.15, 0.0, 0.10];

        let screen = draw(96, 32, &state, 0);
        for label in [
            "vx   forward",
            "vy   left",
            "vyaw turn left",
            "m/s",
            "°/s",
            "asked",
            "applied",
            // The IMU is named as such, not left as a bare `g[...]`.
            "imu · gravity in the trunk frame",
            "x forward",
            "neck_pitch",
            "kp 32",
        ] {
            assert!(screen.contains(label), "{label} is missing:\n{screen}");
        }
        assert!(screen.contains("+0.30"), "what was asked for:\n{screen}");
        assert!(screen.contains("+0.15"), "what was applied:\n{screen}");
    }

    /// The policy driving the robot is named, not just its mode. `walk` is a mode two
    /// different releases share; the file name is what tells them apart.
    #[test]
    fn the_frame_names_the_policy_it_was_told_about() {
        let mut view = View::new(20);
        assert!(
            view.absorb(Update::Policy(Box::new(proto::SubscribeResult {
                accepted: true,
                walk: Some("alpha_walking.onnx".to_owned()),
                stand: Some("alpha_stand.onnx".to_owned()),
                unavailable: None,
            })))
            .is_ok()
        );
        assert!(view.absorb(Update::State(Box::new(a_state()))).is_ok());

        let screen = render_to(&mut view, 110, 32);
        assert!(screen.contains("alpha_walking.onnx"), "{screen}");
        assert!(screen.contains("standing alpha_stand.onnx"), "{screen}");
    }

    /// Nothing said about the policy is reported as such. The alternative — an empty caption —
    /// looks exactly like a robot running no policy at all, which is a different robot.
    #[test]
    fn an_unnamed_policy_is_called_unreported() {
        let screen = draw(100, 32, &a_state(), 0);
        assert!(screen.contains("not reported by robotd"), "{screen}");
    }

    /// A walking policy with no standing one runs at every velocity, which changes how the
    /// robot behaves at rest. Said out loud rather than left as an absence.
    #[test]
    fn a_missing_standing_policy_is_stated() {
        let mut view = View::new(20);
        assert!(
            view.absorb(Update::Policy(Box::new(proto::SubscribeResult {
                accepted: true,
                walk: Some("alpha_walking.onnx".to_owned()),
                stand: None,
                unavailable: None,
            })))
            .is_ok()
        );
        assert!(view.absorb(Update::State(Box::new(a_state()))).is_ok());

        let screen = render_to(&mut view, 110, 32);
        assert!(screen.contains("no standing policy"), "{screen}");
    }

    /// The two shapes that arrive on this connection are told apart by `method`, which only a
    /// notification has — and the subscribe answer is matched by its id, so a response to
    /// something else could not be mistaken for it.
    #[test]
    fn a_state_notification_and_a_subscribe_answer_are_told_apart() {
        let state = serde_json::to_string(&proto::Request::notify_state(&a_state())).unwrap();
        assert!(matches!(decode(&state), Some(Update::State(_))));

        let ack = serde_json::to_string(&proto::Response::ok(
            Some(proto::Id::Number(SUBSCRIBE_ID)),
            &proto::SubscribeResult {
                accepted: true,
                walk: Some("alpha_walking.onnx".to_owned()),
                ..Default::default()
            },
        ))
        .unwrap();
        let Some(Update::Policy(policy)) = decode(&ack) else {
            panic!("the subscribe answer must decode as a policy: {ack}");
        };
        assert_eq!(policy.walk.as_deref(), Some("alpha_walking.onnx"));

        // A response to some other call is not the subscribe answer.
        let other = serde_json::to_string(&proto::Response::ok(
            Some(proto::Id::Number(SUBSCRIBE_ID + 1)),
            &proto::SubscribeResult::default(),
        ))
        .unwrap();
        assert!(decode(&other).is_none());
    }

    /// A `robotd` that predates `SubscribeResult` answers `robot.subscribe` with an
    /// `IntentResult`. That must render as an unnamed policy, not as a failure to parse: the
    /// two are installed separately, and a monitor that dies against last week's robot is a
    /// monitor nobody can use to diagnose one.
    #[test]
    fn an_older_robotd_answer_still_decodes() {
        let old = serde_json::to_string(&proto::Response::ok(
            Some(proto::Id::Number(SUBSCRIBE_ID)),
            &proto::IntentResult::accepted(),
        ))
        .unwrap();

        let Some(Update::Policy(policy)) = decode(&old) else {
            panic!("an older acknowledgement must still decode: {old}");
        };
        assert!(policy.accepted);
        assert_eq!(policy.walk, None);
        assert_eq!(policy.unavailable, None);
    }

    /// With nothing clamped, the limits row says so rather than going blank — a blank row reads
    /// as "this display has nothing to tell you", which is a different claim.
    #[test]
    fn an_unclamped_command_says_it_was_untouched() {
        let screen = draw(96, 32, &a_state(), 0);
        assert!(
            screen.contains("none — the command went through"),
            "{screen}"
        );
    }

    /// Every angle on screen is a degree by default — joints, head and the yaw rate alike.
    /// Radians are what the wire carries and what nobody can picture: a hip at `-0.52` says
    /// nothing to someone looking at the leg it describes.
    #[test]
    fn angles_are_drawn_in_degrees() {
        let screen = draw(110, 32, &a_bent_state(), 0);

        // A joint, its command, and the error between them.
        assert!(screen.contains("+90.00°"), "a right angle:\n{screen}");
        assert!(screen.contains("+85.00°"), "what it was told:\n{screen}");
        assert!(
            screen.contains("+5.00°"),
            "the error between them:\n{screen}"
        );
        // The head, and the turn rate in the twist.
        assert!(screen.contains("neck_pitch +45.00°"), "{screen}");
        assert!(screen.contains("+57.30"), "1 rad/s as °/s:\n{screen}");
        // And no unit label still says radians while the numbers are degrees.
        assert!(!screen.contains("rad/s"), "{screen}");
        assert!(!screen.contains("±0.20 rad"), "{screen}");
    }

    /// `u` puts the radians back. They are what the protocol, the policy's own inputs and every
    /// number a client sends are in, so a reader comparing the screen against any of those has
    /// to be able to see the wire value rather than convert it back by hand.
    #[test]
    fn pressing_u_puts_the_radians_back() {
        let mut view = View::new(20);
        assert!(view.absorb(Update::State(Box::new(a_bent_state()))).is_ok());
        // The key is on screen before it is pressed, or nobody knows it is there.
        assert!(
            render_to(&mut view, 110, 32).contains("u radians"),
            "hinted"
        );

        view.toggle_units();
        let screen = render_to(&mut view, 110, 32);
        assert!(screen.contains("+1.571"), "the wire angle:\n{screen}");
        assert!(screen.contains("+1.00"), "the wire rate:\n{screen}");
        assert!(screen.contains("rad/s"), "{screen}");
        assert!(screen.contains("bar reaches ±0.20 rad"), "{screen}");
        assert!(!screen.contains('°'), "{screen}");
        // And back again, to the unit the view opened in.
        view.toggle_units();
        assert!(render_to(&mut view, 110, 32).contains("+90.00°"));
    }

    /// A joint the wire did not carry stays a dash in either unit — converting an absent
    /// reading would print `+0.00°`, which is a claim about a joint nothing measured.
    #[test]
    fn a_missing_angle_is_a_dash_in_either_unit() {
        let mut view = View::new(20);
        assert_eq!(view.angle(None).content, "-");
        view.toggle_units();
        assert_eq!(view.angle(None).content, "-");
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

    /// A state with an angle on every surface that draws one: a joint a right angle from
    /// straight, its command five degrees off, a tilted head, and a turn rate of 1 rad/s.
    fn a_bent_state() -> proto::RobotState {
        let mut state = a_state();
        state.joints[0] = std::f64::consts::FRAC_PI_2;
        state.targets[0] = std::f64::consts::FRAC_PI_2 - 5.0_f64.to_radians();
        state.head[0] = std::f64::consts::FRAC_PI_4;
        state.movement.requested[2] = 1.0;
        state.movement.applied[2] = 1.0;
        state
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
