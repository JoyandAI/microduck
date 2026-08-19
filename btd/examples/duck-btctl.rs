//! `duck-btctl` — talk to a robot over BLE from a laptop.
//!
//! The phone app's stand-in, and the only way to test `btd` against a real radio.
//!
//! An **example, not a binary**, so `btleplug` never reaches the robot: examples' dependencies
//! are dev-dependencies, and nothing here is in the shipped artifact. `robotctl` is the tool
//! that ships, and it speaks unix sockets on the robot itself.
//!
//! `btleplug` rather than `bluer`, because this runs on a developer's machine: CoreBluetooth on
//! macOS, BlueZ on Linux, WinRT on Windows. `bluer` would restrict the client to Linux, which
//! defeats the point.
//!
//! It reuses `btd::framing` deliberately. The chunking here is the *client* half of the same
//! module the robot uses, so if the framing were asymmetric this would not work — which makes
//! it a real test of the protocol rather than a reimplementation that could agree with itself.
//!
//! ```text
//! cargo run -p btd --example duck-btctl -- scan          # robots in range, and their addresses
//! cargo run -p btd --example duck-btctl -- status
//! cargo run -p btd --example duck-btctl -- wifi scan
//! cargo run -p btd --example duck-btctl -- wifi connect "Pollen" --psk secret
//! cargo run -p btd --example duck-btctl -- name "Ducky"
//! cargo run -p btd --example duck-btctl -- call robot.health
//! ```
//!
//! `DUCK_ROBOT` and `DUCK_PIN` in the environment are the defaults for `--name` and `--pin`, for
//! the machine that talks to the same robot every day. See [`Target`].

use std::net::Ipv4Addr;
use std::time::{Duration, Instant};

use btd::adv;
use btd::framing::{self, Reassembler};
use btd::gatt::{RPC_UUID, SERVICE_UUID};
use btleplug::api::{
    Central, CharPropFlags, Characteristic, Manager as _, Peripheral as _, PeripheralProperties,
    ScanFilter, WriteType,
};
use btleplug::platform::{Manager, Peripheral};
use clap::{Parser, Subcommand};
use futures::StreamExt;

/// How long to look for a robot before giving up.
///
/// Generous, because BLE discovery is genuinely slow and a robot advertises at whatever interval
/// BlueZ chose. Shorter than this and a laptop that was simply unlucky reports "no robot".
const SCAN_TIME: Duration = Duration::from_secs(8);
/// How often the scan results are re-read while waiting.
///
/// A single snapshot after a fixed sleep is what this used to do, and it failed intermittently:
/// BLE advertising is periodic and CoreBluetooth's view of a bonded peripheral comes and goes, so
/// whether the robot was in that one snapshot was partly luck — `no robot found` for a robot that
/// answered fine on the next attempt. Polling until something appears also makes the common case
/// finish in well under a second instead of always paying `SCAN_TIME`.
const SCAN_POLL: Duration = Duration::from_millis(250);

/// How long to wait for a reply once the request is written.
///
/// Longer than any single call except `net.connect`, which polls NetworkManager for up to 45s
/// and so gets its own budget below.
const REPLY_TIMEOUT: Duration = Duration::from_secs(15);
const SLOW_REPLY_TIMEOUT: Duration = Duration::from_secs(60);

/// Every step before the first reply gets its own budget and its own message.
///
/// btleplug bounds none of these, so without them a stall anywhere between "found the robot" and
/// "sent a request" prints `connecting to …` and then nothing at all — which says only that
/// something is wrong, not what. Each of connect, discovery and the first read fails differently
/// and wants a different next move, so each says which one it was.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(20);
const DISCOVER_TIMEOUT: Duration = Duration::from_secs(20);
const READ_TIMEOUT: Duration = Duration::from_secs(15);

/// How many devices a failure lists before summarising the rest.
///
/// A scan in an office reports dozens, and a wall of earbuds is as unreadable as no list at all.
/// Twelve fits a terminal; the count of what was dropped is printed rather than the list silently
/// ending, because "that was everything" and "that was the first twelve" want different next moves.
const LISTED_DEVICES: usize = 12;

/// One peripheral the Mac reported, kept for `scan` and for the failure message.
///
/// The name is held as it arrived — `None` when the advertisement carried none — rather than as the
/// address fallback the tiers use, because "reported without a name" is the diagnosis and the
/// fallback hides it.
struct Seen {
    peripheral: Peripheral,
    identity: String,
    local_name: Option<String>,
    services: usize,
    /// Whether this advertisement carried the duck service UUID, which is the strongest evidence a
    /// listing has: anything better needs a connection, and `scan` deliberately makes none.
    duck: bool,
    /// What the robot broadcast about its place on the network — see [`Address`], and `btd::adv`
    /// for why four bytes of IPv4 and not the SSID too.
    address: Address,
}

/// What a device said about its IPv4 address, which is three answers rather than two.
///
/// `Option<Ipv4Addr>` would collapse the two blanks into one, and they send the reader somewhere
/// different: a robot that broadcast `0.0.0.0` has no network, and a robot that broadcast nothing is
/// on a release from before this existed. The first is a wifi problem and the second is an update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Address {
    At(Ipv4Addr),
    /// The field was there and said `0.0.0.0`: this robot has no address, because it is on no
    /// network or because DHCP has not given it one yet.
    ///
    /// Not called `None`: it sits next to `Option`'s in [`Address::read`], and one of the two is
    /// about a robot's network while the other is about a missing field.
    Unassigned,
    /// No field at all — an older `btd`, or a device that is not a robot.
    Unsaid,
}

impl Address {
    /// Read from one advertisement, and **only for a robot**.
    ///
    /// `btd` files the address under company id `0xFFFF`, which the Bluetooth SIG leaves open to
    /// anyone, so four bytes from `0xFFFF` on an arbitrary device are four bytes of somebody else's
    /// business. Reading it only where the duck service UUID was also advertised is what keeps a
    /// beacon from being listed with an invented address.
    fn read(properties: &PeripheralProperties, duck: bool) -> Self {
        if !duck {
            return Self::Unsaid;
        }
        match adv::address_in(&properties.manufacturer_data) {
            Some(address) => Self::At(address),
            None if adv::has_address_field(&properties.manufacturer_data) => Self::Unassigned,
            None => Self::Unsaid,
        }
    }

    /// How it reads on the device's line in a listing, or nothing at all.
    ///
    /// `Unsaid` renders as nothing rather than as "unknown": every non-robot line is `Unsaid`, and a
    /// column of "unknown" against a room full of earbuds is noise. The robot on an older release is
    /// covered by the note under the list instead, which has room to say what to do about it.
    fn note(self) -> Option<String> {
        match self {
            Self::At(address) => Some(address.to_string()),
            Self::Unassigned => Some("no address".to_owned()),
            Self::Unsaid => None,
        }
    }
}

/// Whatever names this device on this platform.
///
/// **CoreBluetooth never discloses a peripheral's address**, so on macOS every device reports
/// `00:00:00:00:00:00` and a list keyed on it cannot tell one unnamed device from another — which is
/// the case the list exists for. The per-Mac `id` is stable and does distinguish them, so it stands
/// in. BlueZ reports the real address, and there it is the more useful of the two: it is what
/// `pad pair --mac` and `bluetoothctl` take.
fn identity(peripheral: &Peripheral, address: btleplug::api::BDAddr) -> String {
    if address.into_inner() == [0; 6] {
        peripheral.id().to_string()
    } else {
        address.to_string()
    }
}

/// Does this reported name answer to `wanted`?
///
/// **A peripheral can arrive under two names at once.** CoreBluetooth exposes the *cached GAP
/// name* — `CBPeripheral.name`, learned by reading `0x2A00` on an earlier connection — separately
/// from the local name in the advertisement, and btleplug reports them joined when they differ:
/// `radxa-zero3 [duck-c51b]` (`corebluetooth/internal.rs`, `on_discovered_peripheral`).
///
/// They used to differ on every robot, because the two names came from different places: the GAP
/// name is BlueZ's adapter alias, hostname-derived and therefore `radxa-zero3` on every board
/// flashed from one image, while the advertisement carried the name `configd` owns. `btd` now sets
/// the alias to the advertised name (`btd/src/bluez.rs`, `advertise`), so a robot on a current
/// release reports one name however it is asked.
///
/// **This still has to accept both**, and will for as long as a bench has robots on it. A board on
/// an older release has the old alias; so does a client that cached the old GAP name before the
/// robot was updated, until `bluetoothctl remove <mac>` or forgetting it in macOS Bluetooth
/// settings clears that. Matching the joined string exactly meant **both** spellings a person
/// would type were rejected — and the failure then listed the robot as evidence it was not in
/// range.
///
/// So either half is accepted. The advertised half is the robot's real name, and the one the phone
/// app has to match on; the GAP half is accepted because it is what macOS Bluetooth settings shows.
fn answers_to(reported: &str, wanted: &str) -> bool {
    if reported == wanted {
        return true;
    }
    // `rsplit_once`, so a GAP name that itself contains a bracket keeps the *last* group as the
    // advertised half — which is the one btleplug appended.
    match reported.strip_suffix(']').and_then(|s| s.rsplit_once(" [")) {
        Some((gap, advertised)) => gap == wanted || advertised == wanted,
        None => false,
    }
}

/// The default PIN, which every robot has until somebody sets one.
const DEFAULT_PIN: &str = "000000";

/// Which robot to talk to, and whether anybody typed it.
///
/// A laptop reaches the same robot nearly every time, so the name belongs in the environment rather
/// than in every command line: `export DUCK_ROBOT=duck-c51b`, and `--name` stops being something to
/// remember. `--pin` gets the same treatment through `DUCK_PIN`, which a robot with a real PIN needs
/// more than this does.
///
/// **An empty value means unset**, and that is the reason this is not clap's own `env` support.
/// clap reads the variable with `env::var_os` and treats `DUCK_ROBOT=` as a value, so a variable
/// exported in a shell profile could only be escaped by unsetting it — and the command that needs
/// escaping is the one being typed now, on a bench that has somebody else's robot on it. Empty means
/// unset, so `DUCK_ROBOT= duck-btctl scan` is the escape hatch, in the shape a shell already has.
///
/// **Provenance is carried rather than recomputed.** A default makes the tool *stricter*: it
/// suppresses the already-connected fallback tier, and turns "the first robot found wins" into "no
/// robot named duck-c51b in range" — a confusing failure six weeks after editing a shell profile,
/// especially when the same message lists a robot sitting right there. So every message about a
/// robot nobody named says where the name came from.
struct Target {
    /// The name to look for, if any. Empty is not a name.
    name: Option<String>,
    /// Whether [`Self::name`] came from the environment rather than from `--name`.
    from_env: bool,
}

impl Target {
    /// `--name` if it was given, otherwise `DUCK_ROBOT` if it says anything.
    fn new(flag: Option<String>, var: Option<String>) -> Self {
        match flag {
            // An empty `--name` is still `--name`. The flag beats the environment in every case
            // including this one, so beating it with nothing is the second escape hatch: a command
            // line can drop the default without touching the shell it runs in.
            Some(name) => Self {
                name: Some(name).filter(|name| !name.is_empty()),
                from_env: false,
            },
            None => {
                let name = var.filter(|name| !name.is_empty());
                Self {
                    from_env: name.is_some(),
                    name,
                }
            }
        }
    }

    /// The name to match on, for the tiers and the search.
    fn wanted(&self) -> Option<&str> {
        self.name.as_deref()
    }

    /// Is this the device the name points at? Answers the question a listing is read to ask.
    fn marks(&self, local_name: Option<&str>) -> bool {
        match (self.wanted(), local_name) {
            (Some(wanted), Some(reported)) => answers_to(reported, wanted),
            _ => false,
        }
    }

    /// What to blame for the name, in a line that points at one device.
    fn source(&self) -> &'static str {
        if self.from_env {
            "DUCK_ROBOT"
        } else {
            "--name"
        }
    }

    /// Where the name came from, appended to a failure about a robot nobody asked for by name.
    ///
    /// Empty when `--name` was typed: whoever typed it does not need telling.
    fn provenance(&self) -> String {
        match &self.name {
            Some(name) if self.from_env => format!(
                "\n\nNothing on this command line said {name:?} — `DUCK_ROBOT` in this shell's \
                 environment did. `DUCK_ROBOT= duck-btctl …` ignores it for one command, and \
                 `unset DUCK_ROBOT` for the shell."
            ),
            _ => String::new(),
        }
    }

    /// The note after a rename that leaves `DUCK_ROBOT` naming a robot that no longer answers.
    ///
    /// The rename works and then every later command searches for the old name and fails, which
    /// looks like a robot that went away rather than a variable that went stale. Only for the
    /// environment: a `--name` typed once is not still in effect.
    fn stale_after_rename(&self, command: &Command) -> Option<String> {
        let Command::Name { name: new } = command else {
            return None;
        };
        let old = self
            .name
            .as_deref()
            .filter(|old| self.from_env && *old != new.as_str())?;
        Some(format!(
            "note: this robot now answers to {new:?}, and `DUCK_ROBOT` still says {old:?}. Every \
             later command looks for {old:?} until that changes."
        ))
    }
}

/// `--pin`, then `DUCK_PIN`, then the factory default.
///
/// Empty means unset here too, for the reason in [`Target`]: a `DUCK_PIN=` left over from a script
/// would otherwise authenticate with an empty string and be reported as a wrong PIN.
///
/// Unlike `--name`, an empty value is *skipped* rather than final. There is no "no PIN" state to
/// express — every request carries one — so `--pin ''` can only mean "not this one".
fn resolve_pin(flag: Option<String>, var: Option<String>) -> String {
    flag.filter(|pin| !pin.is_empty())
        .or(var.filter(|pin| !pin.is_empty()))
        .unwrap_or_else(|| DEFAULT_PIN.to_owned())
}

/// Which of the candidates to talk to, given what was asked for.
///
/// Generic over the payload so the rule can be tested: a `Peripheral` cannot be constructed off a
/// radio, and this is the one place where getting it wrong means acting on the wrong robot.
///
/// **A name that matches more than one candidate is refused, not resolved.** The two are
/// indistinguishable from here, so there is nothing to prefer and picking either means a write
/// landing on whichever the scan happened to report first — `net.connect` puts a wifi password on
/// that robot. `identity.rs` names the way this happens with nobody doing anything wrong: a board
/// whose bootloader leaves `serial-number` empty falls back to the hostname, so every board flashed
/// from one image answers to `radxa-zero3`.
///
/// Without a name the first candidate still wins. That path is unchanged on purpose — choosing
/// between robots nobody named is exactly what omitting `--name` asks for, and making it an error
/// would break the shorthand on any bench with two boards on it.
///
/// Both failures carry [`Target::provenance`]: a name from `DUCK_ROBOT` is a name nobody on this
/// command line typed, and that is worth saying most where the message is about which robot the
/// command would have landed on.
fn choose<T>(found: Vec<(T, String)>, target: &Target) -> Result<(T, String), String> {
    let Some(wanted) = target.wanted() else {
        // `run` returns early on an empty `found`, so there is at least one.
        return found
            .into_iter()
            .next()
            .ok_or_else(|| "no candidates, which `run` should have reported already".to_owned());
    };

    // Collected before the filter consumes `found`: robots *were* there, they just call themselves
    // something else, and naming them beats "not in range" for a robot that has been renamed since
    // whoever is typing last looked.
    let others: Vec<String> = found.iter().map(|(_, name)| name.clone()).collect();
    let mut matching: Vec<(T, String)> = found
        .into_iter()
        .filter(|(_, name)| answers_to(name, wanted))
        .collect();

    if matching.len() > 1 {
        let names: Vec<String> = matching.iter().map(|(_, name)| name.clone()).collect();
        return Err(format!(
            "{} robots answer to {wanted:?}: {}\nRefusing to guess between them — whichever the \
             scan reported first is not a choice. Rename one from the robot itself (`robotctl \
             system set-name`) and use the new name here.{}",
            names.len(),
            names.join(", "),
            target.provenance(),
        ));
    }

    matching.pop().ok_or_else(|| {
        format!(
            "no robot named {wanted:?} in range. These answered to the duck service: {}\nA name of \
             the form `alias [advertised]` is one robot reported under two names, and either half \
             works.{}",
            others.join(", "),
            target.provenance(),
        )
    })
}

/// Devices as indented lines: what names each one, what it calls itself, where it is, what it is
/// doing.
///
/// Shared by `scan` and by the failure message, because identifying a robot in a list of earbuds is
/// the same problem whether the list is the answer or the diagnosis — and two renderings of it would
/// drift apart exactly where the reader is comparing one run against another.
async fn device_list(mut devices: Vec<&Seen>, target: &Target) -> String {
    // The named robot first, then named devices, then by identity: a device that reported a name is
    // the line worth reading, and sorting keeps a re-run's output comparable with the last one.
    //
    // The default's own line leads, because this list is truncated at `LISTED_DEVICES` and an office
    // holds more devices than that: sorted by identity alone, the one line the reader is looking for
    // lands in "… and 6 more" as often as not, and a marker nobody can see is not an answer.
    devices.sort_by(|a, b| {
        (!target.marks(a.local_name.as_deref()))
            .cmp(&!target.marks(b.local_name.as_deref()))
            .then(a.local_name.is_none().cmp(&b.local_name.is_none()))
            .then(a.identity.cmp(&b.identity))
    });

    let mut lines: Vec<String> = Vec::new();
    for device in devices.iter().take(LISTED_DEVICES) {
        let mut notes: Vec<String> = Vec::new();
        // The leading note, because it is what the line is read for: `scan` is how someone finds the
        // robot to ssh into or point a browser at, and the service count is diagnosis by comparison.
        notes.extend(device.address.note());
        if device.services > 0 {
            notes.push(format!("{} service(s)", device.services));
        }
        // Checked here rather than during the scan: it is one call per device once, instead of one
        // per device per 250ms poll, and nothing before the list is printed needs the answer.
        if device.peripheral.is_connected().await.unwrap_or(false) {
            notes.push("connected".to_owned());
        }
        let notes = if notes.is_empty() {
            String::new()
        } else {
            format!(" — {}", notes.join(", "))
        };
        // The line the reader is looking for, marked. `scan` with a default set is read to answer
        // "is my robot here", and that is otherwise a string comparison done by eye against a list
        // of hex — worse on macOS, where the robot is reported under two names joined together.
        let mark = if target.marks(device.local_name.as_deref()) {
            format!("  ← {}", target.source())
        } else {
            String::new()
        };
        lines.push(format!(
            "  {} {}{notes}{mark}",
            device.identity,
            device.local_name.as_deref().unwrap_or("(no name)"),
        ));
    }

    if devices.len() > LISTED_DEVICES {
        lines.push(format!("  … and {} more", devices.len() - LISTED_DEVICES));
    }
    lines.join("\n")
}

/// Whether the devices that are not robots are listed, or only counted.
///
/// The duck service UUID in the advertisement is the strongest evidence available to a listing —
/// anything better needs a connection, and connecting to 43 devices to ask each whether it is a
/// robot would be minutes of pairing prompts. So that block is not padding: a robot already bonded
/// with this Mac frequently advertises no services at all, and it is the reason `--name` exists.
///
/// But `scan` is read to answer "which robots can I talk to", and a dozen lines of earbuds above
/// the answer buries it. So the block is the diagnosis rather than the output, and it appears when
/// it is one:
///
/// - `--verbose`, which is the flag for asking what the radio actually saw.
/// - **No robot advertised the service**, verbose or not. That is precisely the case where the robot
///   is plausibly in the other list and hiding the evidence would leave nothing to act on.
///
/// Otherwise it is a count and how to expand it, because "that was every device" and "that was the
/// robots" want different next moves.
fn lists_others(verbose: bool, robots: usize) -> bool {
    verbose || robots == 0
}

/// What `scan` prints: the robots, and — per [`lists_others`] — everything else.
async fn listing(seen: &[Seen], verbose: bool, target: &Target) -> String {
    let (robots, others): (Vec<&Seen>, Vec<&Seen>) = seen.iter().partition(|d| d.duck);
    // Kept before `device_list` consumes the vector, since they decide the blocks below.
    let found = robots.len();
    let silent = robots
        .iter()
        .filter(|d| d.address == Address::Unsaid)
        .count();

    let mut out = if robots.is_empty() {
        "no robot advertised the duck service.".to_owned()
    } else {
        format!(
            "{} robot(s) advertising the duck service:\n{}",
            robots.len(),
            device_list(robots, target).await,
        )
    };

    // A robot whose line carries no address at all is on a release from before `btd` broadcast one,
    // and its line cannot say so: an absent field looks the same as a device that never had one. Said
    // once, below the list, where there is room for what to do about it — and only when it happened,
    // because on a bench of current robots this sentence is noise.
    if silent > 0 {
        out.push_str(&format!(
            "\n\n{silent} of them broadcast no address, which is a release from before `btd` \
             advertised one. `duck-btctl wifi status` still reports it; updating the robot puts it \
             in this list."
        ));
    }

    if !others.is_empty() {
        if lists_others(verbose, found) {
            let anonymous = others.iter().filter(|d| d.local_name.is_none()).count();
            out.push_str(&format!(
                "\n\n{} other device(s) in {SCAN_TIME:?}, {anonymous} with no name. A robot bonded \
                 with this Mac often stops advertising the service to it, so it can be one of \
                 these — `--name <its name>` connects to it anyway:\n{}",
                others.len(),
                device_list(others, target).await,
            ));
        } else {
            out.push_str(&format!(
                "\n\n{} other device(s) in range, not listed. A robot bonded with this Mac can be \
                 among them, advertising no service — `--verbose` lists them.",
                others.len(),
            ));
        }
    }
    out
}

/// Why the scan came back empty, in terms of what the radio actually reported.
///
/// Without this, two failures print the same sentence and want opposite next moves: an empty list is
/// a problem on *this* machine — Bluetooth off, the permission never granted, another scan holding
/// the radio — while a list the robot is missing from points at the robot.
///
/// And the robot can be *in* that list, unrecognisable. `btd` advertises flags (3 bytes), a 128-bit
/// service UUID (18) and the address field (8, see `btd::adv`), which is 29 of the 31 bytes a legacy
/// advertisement holds — so the name never travels in it. It goes in the scan response, a second
/// exchange that can be missed on its own. A device reported with no name and no services is
/// therefore a plausible robot, which is why the unnamed ones are listed rather than filtered out.
async fn nothing_found(seen: &[Seen], target: &Target) -> String {
    if seen.is_empty() {
        return format!(
            "no robot found — and the Mac reported no BLE devices at all in {SCAN_TIME:?}, not one \
             pair of earbuds. That points at this machine rather than the robot: is Bluetooth on, \
             and has this terminal been granted the Bluetooth permission?"
        );
    }

    let missed = match target.wanted() {
        Some(name) => format!(" and nothing was named {name:?}"),
        None => String::new(),
    };
    // The count of unnamed devices is in the summary rather than left to be inferred from the list:
    // named ones sort first, so truncation hides exactly the lines the robot could be hiding in, and
    // "is it plausibly one of those" is the question this list is read to answer.
    let anonymous = seen.iter().filter(|d| d.local_name.is_none()).count();
    let mut message = format!(
        "no robot found. Nothing advertised the duck service{missed}. The Mac saw {} device(s) in \
         {SCAN_TIME:?}, {anonymous} of them with no name:\n{}",
        seen.len(),
        device_list(seen.iter().collect(), target).await,
    );
    // Before the generic advice, because "why is it looking for that name" comes first for a reader
    // who did not type one.
    message.push_str(&target.provenance());
    message.push_str(
        "\nIf the robot is one of the unnamed lines, it was reported without the name and the \
         service UUID this matches on, and retrying usually finds it. If it is absent entirely, \
         `journalctl -u btd -b` on the robot says whether the GATT application is registered.",
    );
    message
}

/// Run one step with a budget, naming it if the budget runs out.
async fn step<T>(
    what: &str,
    hint: &str,
    budget: Duration,
    f: impl std::future::Future<Output = Result<T, btleplug::Error>>,
) -> Result<T, Box<dyn std::error::Error>> {
    match tokio::time::timeout(budget, f).await {
        Ok(Ok(value)) => Ok(value),
        Ok(Err(e)) => Err(format!("{what} failed: {e}\n{hint}").into()),
        Err(_) => Err(format!("{what} timed out after {budget:?}\n{hint}").into()),
    }
}

#[derive(Parser)]
#[command(
    // Spelled out because clap would otherwise take it from the crate, and `--version` on the
    // installed binary answered `btd 0.5.1` — the daemon's name, for the laptop-side client.
    name = "duck-btctl",
    version,
    about = "Talk to a robot over BLE — the phone app's stand-in",
    long_about = "Finds a robot advertising the duck GATT service and speaks the same JSON-RPC \
                  lines every other transport uses. This is a development tool: it is an example \
                  rather than a binary, so it never ships to a robot."
)]
struct Cli {
    /// Connect to this robot by advertised name. Without it, `DUCK_ROBOT`; without that, the first
    /// robot found wins.
    ///
    /// The advertised name, which is what `system.info` reports and what `name` below sets: a
    /// board that has never been renamed answers to its derived default, `duck-7f3a`.
    ///
    /// `export DUCK_ROBOT=duck-c51b` in a shell profile makes that the robot every command talks
    /// to. `DUCK_ROBOT= duck-btctl …` ignores it for one command.
    //
    // The id is spelled out rather than derived from the field, because clap keys arguments by id
    // and the `name` subcommand has a positional argument that derives the same one. With both
    // called `name` the positional won, so `--name duck-c51b name leduckpierre` searched for
    // `leduckpierre` — the name it was about to set — and then reported the robot standing in
    // front of it as out of range. `value_name` keeps the help line reading `--name <ROBOT_NAME>`
    // rather than leaking the id into it.
    #[arg(long = "name", id = "robot", value_name = "ROBOT_NAME", global = true)]
    name: Option<String>,

    /// Print every line sent and received, and have `scan` list every device rather than the robots.
    #[arg(long, global = true)]
    verbose: bool,

    /// The robot's pairing PIN. Defaults to `DUCK_PIN`, then to `000000`.
    ///
    /// Six digits, shown by `robotctl system pin` on the robot. The factory default is `000000`
    /// and authenticates anyone who has read this repository, which is why a shipped robot needs a
    /// per-robot one — and why `export DUCK_PIN=…` is worth more than the name is.
    //
    // No `default_value`, because the default has to be applied *after* the environment or it would
    // shadow it: clap fills a `default_value` in and nothing downstream can then tell `000000` typed
    // from `000000` assumed. It is spelled out in the help text above instead.
    #[arg(long, global = true)]
    pin: Option<String>,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List robots in range with the address each one broadcast, and stop.
    Scan,
    /// Version handshake plus update status.
    Status,
    /// Name, serial and uptime.
    Info,
    /// Is the control loop healthy?
    Health,
    /// Wifi.
    #[command(subcommand)]
    Wifi(Wifi),
    /// Rename the robot.
    Name {
        /// What to call it from now on. `--name` above still names the robot to rename.
        #[arg(value_name = "NEW_NAME")]
        name: String,
    },
    /// Reboot it.
    Reboot,
    /// Send any method, for whatever is not wrapped above.
    Call {
        method: String,
        /// Parameters as JSON. Defaults to `{}`.
        params: Option<String>,
    },
}

#[derive(Subcommand)]
enum Wifi {
    /// What the wifi is doing — SSID, signal, addresses.
    Status,
    /// Networks the robot can see.
    Scan,
    /// Join a network.
    Connect {
        ssid: String,
        /// Omit for an open network.
        #[arg(long)]
        psk: Option<String>,
    },
    /// Forget a stored network.
    Forget { ssid: String },
}

/// Print the error rather than returning it, and that is not a style preference.
///
/// A `main` returning `Err` is reported by Rust's `Termination` impl, which **`Debug`-formats** the
/// error: every hint in this file is multi-line, and `Debug` on a string renders the newlines as
/// literal `\n` and wraps the lot in quotes. So the guidance written to be read as lines arrived as
/// one escaped blob — worst for the failure that lists what the radio saw, which is a dozen lines.
#[tokio::main]
async fn main() -> std::process::ExitCode {
    match run().await {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            std::process::ExitCode::FAILURE
        }
    }
}

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();
    // Read once, here, so everything below asks `target` rather than the environment: which robot
    // was chosen and who chose it is one decision, and a second reader of `DUCK_ROBOT` could
    // disagree with the first.
    let target = Target::new(cli.name.clone(), std::env::var("DUCK_ROBOT").ok());
    let pin = resolve_pin(cli.pin.clone(), std::env::var("DUCK_PIN").ok());

    // `scan` shares the discovery below and then stops, because a listing and a search look for the
    // same thing and differ only in what they do with it. It connects to nothing at all: that is
    // what makes it the safe command to reach for when a robot cannot be reached, and it is also why
    // it can only report what an advertisement carries.
    let list_only = matches!(cli.command, Command::Scan);

    let manager = Manager::new().await?;
    let adapter = manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or("no Bluetooth adapter on this machine")?;

    // Disclosed before the eight seconds rather than in the failure afterwards: a search nobody
    // typed is worth saying out loud, and this is also how somebody who set `DUCK_ROBOT` months ago
    // finds out it is still in force.
    if let Some(name) = target.wanted().filter(|_| target.from_env) {
        eprintln!("looking for {name:?} — `DUCK_ROBOT` in this shell's environment");
    }
    eprintln!("scanning for up to {SCAN_TIME:?}…");
    // **Unfiltered on purpose.** This used to pass `ScanFilter { services: [SERVICE_UUID] }`, on the
    // theory that a busy office would otherwise drown the robot in headphones. But CoreBluetooth
    // honours that filter *strictly*: a peripheral whose current advertisement does not carry the
    // UUID is never reported at all. A bonded robot frequently reports with an empty service list —
    // so the `--name` fallback below could only ever match something the filtered scan had already
    // returned, which made it dead weight in exactly the case it exists for. That is the whole
    // explanation for `no robot found` on one run and success on the next.
    //
    // So: report everything, and discriminate here, where the rules are ours.
    adapter.start_scan(ScanFilter::default()).await?;

    // Candidates, strongest evidence first.
    //
    // The advertised service UUID is an *optimisation*, not the identity check — and treating it as
    // the latter broke as soon as the Mac bonded with the robot. The authoritative test is whether
    // it serves our characteristic, which is only knowable after connecting.
    let mut advertised: Vec<(Peripheral, String)> = Vec::new();
    let mut named: Vec<(Peripheral, String)> = Vec::new();
    let mut connected: Vec<(Peripheral, String)> = Vec::new();
    // Everything the Mac reported, kept only so a failure can say what was in range. `configd`
    // learned this on the other side — a failed `pad pair` lists what the radio saw, because the
    // escape hatch needs an address nobody has otherwise.
    let mut seen: Vec<Seen> = Vec::new();
    let deadline = Instant::now() + SCAN_TIME;

    loop {
        advertised.clear();
        named.clear();
        connected.clear();
        // Cleared with the tiers, and rebuilt from the same sweep: `peripherals()` reports
        // everything known to this scan session rather than only what arrived since the last poll,
        // so the final sweep is the fullest one.
        seen.clear();

        for peripheral in adapter.peripherals().await? {
            let Some(properties) = peripheral.properties().await? else {
                continue;
            };
            let name = properties
                .local_name
                .clone()
                .unwrap_or_else(|| properties.address.to_string());

            let duck = properties.services.contains(&SERVICE_UUID);
            seen.push(Seen {
                peripheral: peripheral.clone(),
                identity: identity(&peripheral, properties.address),
                local_name: properties.local_name.clone(),
                services: properties.services.len(),
                duck,
                address: Address::read(&properties, duck),
            });

            if list_only {
                // A listing connects to nothing, so the tiers — which exist to choose what to
                // connect to — have no work to do, and the `is_connected` call below would cost one
                // round trip per device per poll for an answer nothing reads.
                continue;
            }

            if duck {
                advertised.push((peripheral, name));
            } else if target.wanted().is_some_and(|w| answers_to(&name, w)) {
                named.push((peripheral, name));
            } else if target.wanted().is_none() && peripheral.is_connected().await? {
                // Last resort, and only without a name: an unfiltered scan sees every connected
                // peripheral on the Mac, so this tier is full of keyboards and earbuds. Each one
                // costs a connect and a service discovery before it can be ruled out, which is why
                // an explicit name suppresses the tier entirely rather than being merged into it.
                connected.push((peripheral, name));
            }
        }

        // Stop as soon as there is anything worth connecting to: a bonded robot may never
        // re-advertise the service to this Mac, so waiting out the deadline for a better candidate
        // would just be eight seconds of nothing.
        //
        // A listing is the exception, and runs the deadline out: stopping at the first robot would
        // report one and hide the second, which is the only question worth asking in a room with
        // three of them.
        if (!list_only && (!advertised.is_empty() || !named.is_empty() || !connected.is_empty()))
            || Instant::now() >= deadline
        {
            break;
        }
        tokio::time::sleep(SCAN_POLL).await;
    }
    let _ = adapter.stop_scan().await;

    if list_only {
        // Nothing at all is a fault on this machine rather than a report about robots, and
        // `nothing_found` is where that diagnosis lives. An error, so the exit status says so too.
        if seen.is_empty() {
            return Err(nothing_found(&seen, &target).await.into());
        }
        println!("{}", listing(&seen, cli.verbose, &target).await);
        return Ok(());
    }

    let mut found = advertised;
    if found.is_empty() && !named.is_empty() {
        if cli.verbose {
            eprintln!(
                "nothing advertised the service; trying {} peripheral(s) matching the name — a \
                 bonded robot often stops advertising it to a Mac that has already paired",
                named.len()
            );
        }
        found = named;
    } else if found.is_empty() && !connected.is_empty() {
        if cli.verbose {
            eprintln!(
                "nothing advertised the service; trying {} already-connected peripheral(s), which \
                 may well be earbuds. `--name <robot name>`, or `DUCK_ROBOT`, skips this guesswork",
                connected.len()
            );
        }
        found = connected;
    }

    if found.is_empty() {
        return Err(nothing_found(&seen, &target).await.into());
    }

    let (peripheral, name) = choose(found, &target)?;
    eprintln!("connecting to {name}…");

    step(
        "connecting",
        "The robot advertised but would not accept a connection. If macOS shows it as paired, \
         forget it there and retry; `sudo pkill bluetoothd` also clears a half-finished bond.",
        CONNECT_TIMEOUT,
        peripheral.connect(),
    )
    .await?;
    if cli.verbose {
        eprintln!("connected; discovering services…");
    }

    step(
        "service discovery",
        "Connected, but the robot never described its services. Check `journalctl -u btd -b` on \
         the robot for whether the GATT application is registered.",
        DISCOVER_TIMEOUT,
        peripheral.discover_services(),
    )
    .await?;

    let (request, response) = characteristics(&peripheral)?;

    // Read first, and this is load-bearing rather than a courtesy.
    //
    // The robot requires an authenticated encrypted link to *write*, but a subscribe needs no
    // encryption — so without this a central subscribes happily, has its first write refused, and
    // on macOS sees neither a prompt nor an error. A read is acknowledged, so an unpaired link
    // fails here instead, which is what makes CoreBluetooth start pairing.
    //
    // The value is the robot's API version, and it is reported rather than enforced. See the
    // mismatch warning below for why this tool refuses nothing on it.
    let read = step(
        "reading the API version",
        "This read requires an encrypted link, so it is what triggers pairing. A hang here usually \
         means the bond did not complete: forget the robot in macOS Bluetooth settings, or run \
         `sudo pkill bluetoothd`, and retry.",
        READ_TIMEOUT,
        peripheral.read(&response),
    )
    .await;

    match read {
        Ok(value) => {
            let theirs = value.first().copied().unwrap_or(0);
            if cli.verbose {
                eprintln!("robot speaks API v{theirs}");
            }
            if u32::from(theirs) != duck_ipc_proto::API_VERSION {
                warn_about_skew(theirs);
            }
        }
        Err(e) => return Err(e),
    }

    // Subscribe *before* writing, or a reply can arrive before there is anywhere to put it.
    // btd's session begins on the first write, so the order here is not merely defensive: the
    // notify half has to exist for the session to have somewhere to answer.
    peripheral.subscribe(&response).await?;
    let mut notifications = peripheral.notifications().await?;

    // Prove the PIN before anything else. The bond is just-works, so it encrypts the link and
    // authenticates nobody; the robot serves nothing until this succeeds. See
    // `btd/src/pairing.rs` for why the check is here rather than in the pairing.
    let auth = serde_json::json!({
        "jsonrpc": "2.0",
        "id": 0,
        "method": "system.authenticate",
        "params": { "pin": pin },
    });
    let auth = serde_json::to_string(&auth)?;
    if cli.verbose {
        // The PIN is deliberately not printed, even here: a terminal is a log too.
        eprintln!("→ system.authenticate (pin redacted)");
    }
    write_line(&peripheral, &request, &auth).await?;

    let reply = read_line(&mut notifications, REPLY_TIMEOUT).await?;
    if cli.verbose {
        eprintln!("← {reply}");
    }
    let parsed: serde_json::Value = serde_json::from_str(&reply)?;
    if parsed["result"]["authenticated"] != serde_json::json!(true) {
        let left = parsed["result"]["attempts_remaining"].as_u64();
        return Err(match left {
            Some(0) => "wrong PIN, and no attempts left — the robot closed the session. \
                        Check it with `robotctl system pin` on the robot."
                .to_owned()
                .into(),
            Some(n) => format!(
                "wrong PIN ({n} attempt(s) left). Check it with `robotctl system pin` on the robot."
            )
            .into(),
            None => format!("authentication failed: {reply}").into(),
        });
    }

    let (line, timeout) = request_line(&cli.command)?;
    if cli.verbose {
        eprintln!("→ {line}");
    }

    // Chunked by the same code the robot uses. btleplug does not expose the negotiated MTU, so
    // 20 bytes — the floor every BLE link guarantees — is the safe assumption. Slower than
    // necessary on a good link, and correct on every link.
    for chunk in framing::chunks(&line, 20) {
        peripheral
            .write(&request, &chunk, WriteType::WithoutResponse)
            .await?;
    }

    let mut reassembler = Reassembler::new();
    let deadline = tokio::time::Instant::now() + timeout;

    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            return Err(format!("no reply within {timeout:?}").into());
        }

        let Ok(Some(notification)) = tokio::time::timeout(remaining, notifications.next()).await
        else {
            return Err(format!("no reply within {timeout:?}").into());
        };

        for line in reassembler.push(&notification.value)? {
            if cli.verbose {
                eprintln!("← {line}");
            }
            // Notifications with no `id` are a progress stream, not an answer; print them and
            // keep waiting for the response that closes the call.
            let value: serde_json::Value = serde_json::from_str(&line)?;
            let is_answer = value.get("id").is_some_and(|id| !id.is_null());

            println!("{}", serde_json::to_string_pretty(&value)?);
            if is_answer {
                let _ = peripheral.disconnect().await;
                // A JSON-RPC error is the robot answering, not this tool failing — so it is
                // printed above and reported through the exit status rather than as a panic.
                return if value.get("error").is_some() {
                    Err("the robot returned an error".into())
                } else {
                    // After the reply, and only for one that succeeded: a rename that the robot
                    // refused leaves nothing stale.
                    if let Some(note) = target.stale_after_rename(&cli.command) {
                        eprintln!("{note}");
                    }
                    Ok(())
                };
            }
        }
    }
}

/// Write one NDJSON line, chunked.
///
/// Chunked by the same code the robot uses. btleplug does not expose the negotiated MTU, so 20
/// bytes — the floor every BLE link guarantees — is the safe assumption: slower than necessary on a
/// good link, correct on every link.
///
/// **Acknowledged writes**, and that is not a detail. An ATT Write *Command* (`WithoutResponse`)
/// carries no reply, so a refusal — for insufficient encryption, say — is invisible: the request
/// silently never arrives and the client waits out its timeout with no idea why. That is exactly
/// how this first behaved, against a robot that was working perfectly.
async fn write_line(
    peripheral: &Peripheral,
    characteristic: &Characteristic,
    line: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    for chunk in framing::chunks(line, 20) {
        peripheral
            .write(characteristic, &chunk, WriteType::WithResponse)
            .await?;
    }
    Ok(())
}

/// Read one complete NDJSON line from the notification stream.
async fn read_line(
    notifications: &mut (impl futures::Stream<Item = btleplug::api::ValueNotification> + Unpin),
    timeout: Duration,
) -> Result<String, Box<dyn std::error::Error>> {
    let mut reassembler = Reassembler::new();
    let deadline = tokio::time::Instant::now() + timeout;

    loop {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            return Err(format!("no reply within {timeout:?}").into());
        }
        let Ok(Some(notification)) = tokio::time::timeout(remaining, notifications.next()).await
        else {
            return Err(format!("no reply within {timeout:?}").into());
        };
        if let Some(line) = reassembler.push(&notification.value)?.into_iter().next() {
            return Ok(line);
        }
    }
}

/// Find the two characteristics, and check they can do what we need.
///
/// Checking the properties rather than assuming them turns a confusing silence — a write that
/// lands nowhere — into a clear message naming which half is wrong.
fn characteristics(
    peripheral: &Peripheral,
) -> Result<(Characteristic, Characteristic), Box<dyn std::error::Error>> {
    let all = peripheral.characteristics();
    let find = |uuid| all.iter().find(|c| c.uuid == uuid).cloned();

    let rpc =
        find(RPC_UUID).ok_or("the robot has no RPC characteristic; is this the right service?")?;

    // One characteristic carries both directions, so it must be able to do both. Checking rather
    // than assuming turns a confusing silence — a write that lands nowhere — into a message
    // naming which half is missing.
    if !rpc
        .properties
        .intersects(CharPropFlags::WRITE | CharPropFlags::WRITE_WITHOUT_RESPONSE)
    {
        return Err("the RPC characteristic is not writable".into());
    }
    if !rpc.properties.contains(CharPropFlags::NOTIFY) {
        return Err("the RPC characteristic cannot notify".into());
    }
    // Cloned rather than borrowed twice: btleplug takes a &Characteristic for each operation.
    Ok((rpc.clone(), rpc))
}

/// One command becomes one JSON-RPC line, plus how long to wait for it.
/// Say that the two ends were not built together, and carry on.
///
/// **This used to be a refusal, and the refusal was wrong twice over.**
///
/// It was wrong about what it was reading. `API_VERSION` is an agreement between the binaries on
/// one board — `robotctl` and `updaterd` come from one release, and `updaterd`'s exact `!=` on
/// `Hello` is what enforces it. A laptop is not a binary on the board, and will routinely be a
/// release ahead of a robot it is talking to precisely because it is the machine that builds
/// releases. Nothing on the far side of this link agrees with the refusal either: this tool never
/// sends `Hello`, `configd` checks no version on `net.*` or `system.*`, and `updaterd` requires no
/// handshake before `update.status`. So every call the refusal blocked would have been answered.
///
/// And it was wrong about when to be strict. BLE is the transport for a robot that has no network,
/// and `wifi connect` is how that robot gets one — so refusing on version skew took away the
/// command that fixes the skew, at the one moment it was needed. A robot with a stale release and
/// no wifi could not be given wifi by the tool whose reason for existing is that case.
///
/// What a genuine mismatch costs without the gate is a method whose params changed shape, which
/// comes back as a JSON-RPC error naming the method — printed, and reported through the exit
/// status. That is a worse message than this one and a much better outcome than a locked door.
fn warn_about_skew(theirs: u8) {
    eprintln!(
        "warning: the robot speaks API v{theirs} and this client speaks v{}, so they were not \
         built together. Carrying on: most calls do not care, and a call that does will say so. \
         Install matching versions before believing anything surprising.",
        duck_ipc_proto::API_VERSION
    );
}

fn request_line(command: &Command) -> Result<(String, Duration), Box<dyn std::error::Error>> {
    let (method, params, timeout) = match command {
        // `scan` returns from `run` as soon as the discovery loop ends, so it never reaches a
        // request: there is no method to send, and connecting is the thing it exists not to do.
        Command::Scan => unreachable!("scan returns before anything connects"),
        Command::Status => ("update.status", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Info => ("system.info", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Health => ("robot.health", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Name { name } => (
            "system.setName",
            serde_json::json!({ "name": name }),
            REPLY_TIMEOUT,
        ),
        Command::Reboot => ("system.reboot", serde_json::json!({}), REPLY_TIMEOUT),
        Command::Wifi(Wifi::Status) => ("net.status", serde_json::json!({}), REPLY_TIMEOUT),
        // A scan asks NetworkManager to re-scan, which takes seconds on a quiet radio.
        Command::Wifi(Wifi::Scan) => ("net.scan", serde_json::json!({}), SLOW_REPLY_TIMEOUT),
        Command::Wifi(Wifi::Connect { ssid, psk }) => {
            let mut params = serde_json::json!({ "ssid": ssid });
            if let Some(psk) = psk {
                params["psk"] = serde_json::Value::String(psk.clone());
            }
            // configd polls NM for up to 45s before calling a join timed out, so this must wait
            // longer than that or the tool gives up before the robot has decided.
            ("net.connect", params, SLOW_REPLY_TIMEOUT)
        }
        Command::Wifi(Wifi::Forget { ssid }) => (
            "net.forget",
            serde_json::json!({ "ssid": ssid }),
            REPLY_TIMEOUT,
        ),
        Command::Call { method, params } => {
            let params = match params {
                Some(text) => {
                    serde_json::from_str(text).map_err(|e| format!("params must be JSON: {e}"))?
                }
                None => serde_json::json!({}),
            };
            (method.as_str(), params, SLOW_REPLY_TIMEOUT)
        }
    };

    let request = serde_json::json!({
        "jsonrpc": "2.0",
        "id": 1,
        "method": method,
        "params": params,
    });
    Ok((serde_json::to_string(&request)?, timeout))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The ordinary case, and the only one on Linux: one name, reported as it was advertised.
    #[test]
    fn a_single_name_answers_to_itself() {
        assert!(answers_to("duck-c51b", "duck-c51b"));
        assert!(!answers_to("duck-c51b", "duck-ffff"));
    }

    /// The case that made `--name` unusable: the exact string a person types is *neither* of the
    /// names macOS reported, so `wifi status` failed against a robot the same message listed.
    #[test]
    fn either_half_of_a_macos_composite_answers() {
        let reported = "radxa-zero3 [duck-c51b]";
        assert!(answers_to(reported, "duck-c51b"), "the advertised name");
        assert!(answers_to(reported, "radxa-zero3"), "the cached GAP name");
        assert!(answers_to(reported, reported), "copied from the failure");
        assert!(!answers_to(reported, "duck-ffff"));
    }

    /// `scan` is read to learn which robots are reachable, and a dozen lines of earbuds above the
    /// answer bury it. The clause worth pinning is the second one: with nothing advertising the
    /// service, the other devices *are* the answer — the robot is plausibly among them — so they are
    /// listed whether or not `--verbose` was given, and gating them purely on the flag would leave
    /// that failure with nothing to act on.
    #[test]
    fn other_devices_are_listed_when_they_are_the_diagnosis() {
        assert!(!lists_others(false, 1), "the robot is the answer");
        assert!(lists_others(true, 1), "--verbose asks what the radio saw");
        assert!(lists_others(false, 0), "no robot: the list is all there is");
        assert!(lists_others(true, 0));
    }

    /// Named candidates, as `choose` takes them.
    fn candidates(names: &[&str]) -> Vec<(usize, String)> {
        names
            .iter()
            .enumerate()
            .map(|(i, name)| (i, (*name).to_owned()))
            .collect()
    }

    /// A name typed on the command line, which is the ordinary way to reach `choose`.
    fn asked_for(name: &str) -> Target {
        Target::new(Some(name.to_owned()), None)
    }

    /// The ordinary case: one name, one robot, and the composite spelling still resolves.
    #[test]
    fn a_name_selects_the_one_robot_that_answers_to_it() {
        let (which, _) = choose(
            candidates(&["duck-aaaa", "duck-c51b"]),
            &asked_for("duck-c51b"),
        )
        .expect("the named robot");
        assert_eq!(which, 1);

        let (which, _) = choose(
            candidates(&["duck-aaaa", "radxa-zero3 [duck-c51b]"]),
            &asked_for("duck-c51b"),
        )
        .expect("either half of a composite");
        assert_eq!(which, 1);
    }

    /// **The safety rule.** Two robots answering to one name is not rare or hypothetical: a board
    /// whose bootloader leaves `serial-number` empty is named from its hostname, so a bench flashed
    /// from one image is full of `radxa-zero3`. Whichever the scan reported first is not a choice,
    /// and the command that lands on it may be `net.connect` with someone's wifi password.
    #[test]
    fn a_name_matching_two_robots_is_refused_rather_than_guessed() {
        let error = choose(
            candidates(&["radxa-zero3", "radxa-zero3", "duck-c51b"]),
            &asked_for("radxa-zero3"),
        )
        .expect_err("a collision is an error");

        assert!(error.contains("2 robots"), "{error}");
        // Both are named, so the reader can tell which two collided.
        assert!(error.contains("radxa-zero3, radxa-zero3"), "{error}");
        assert!(error.contains("set-name"), "the way out: {error}");
    }

    /// A collision on a name from the environment says so. This is the failure where provenance
    /// matters most: nothing on the command line named the robot, and the message is about which of
    /// two the command would otherwise have written to.
    #[test]
    fn a_collision_on_a_default_says_where_the_name_came_from() {
        let from_env = Target::new(None, Some("radxa-zero3".to_owned()));
        let error = choose(candidates(&["radxa-zero3", "radxa-zero3"]), &from_env)
            .expect_err("a collision is an error whoever named it");

        assert!(error.contains("2 robots"), "{error}");
        assert!(error.contains("DUCK_ROBOT"), "{error}");
    }

    /// Omitting `--name` is a request to pick one, and it stays one. Making the ambiguity an error
    /// on this path would break the shorthand on every bench with two boards on it.
    #[test]
    fn without_a_name_the_first_candidate_still_wins() {
        let (which, _) = choose(
            candidates(&["duck-aaaa", "duck-c51b"]),
            &Target::new(None, None),
        )
        .expect("the first one");
        assert_eq!(which, 0);
    }

    /// A name nobody answers to lists what was there, because the usual cause is a robot that has
    /// been renamed since whoever is typing last looked.
    #[test]
    fn a_name_nobody_answers_to_lists_the_robots_that_were_there() {
        let error = choose(
            candidates(&["duck-aaaa", "duck-bbbb"]),
            &asked_for("duck-c51b"),
        )
        .expect_err("not in range");

        assert!(error.contains("no robot named"), "{error}");
        assert!(error.contains("duck-aaaa, duck-bbbb"), "{error}");
    }

    /// `--name` says which robot to talk to and the `name` subcommand's positional says what to
    /// call it, and only an explicit id keeps the two apart. Parsing is pinned rather than left to
    /// review because the failure did not look like a CLI bug: the tool scanned for the new name,
    /// found nothing, and listed the robot it was talking to seconds earlier as merely in range.
    #[test]
    fn a_rename_still_selects_the_robot_by_the_name_it_has_now() {
        let cli =
            Cli::try_parse_from(["duck-btctl", "--name", "duck-c51b", "name", "leduckpierre"])
                .expect("the rename form parses");

        assert_eq!(cli.name.as_deref(), Some("duck-c51b"), "which robot");
        let Command::Name { name } = &cli.command else {
            panic!("the name subcommand");
        };
        assert_eq!(name, "leduckpierre", "what to call it");

        // And the new name is what reaches the robot, not the one it was found by.
        let (line, _) = request_line(&cli.command).expect("a request");
        assert!(line.contains(r#""method":"system.setName""#), "{line}");
        assert!(line.contains(r#""name":"leduckpierre""#), "{line}");
    }

    /// The point of the whole thing: a robot named by nobody who is typing. The environment has to
    /// reach the same search `--name` does, and say so, since a default that silently redirects
    /// every command is worse than no default.
    #[test]
    fn the_environment_names_the_robot_when_the_flag_does_not() {
        let target = Target::new(None, Some("duck-c51b".to_owned()));

        assert_eq!(target.wanted(), Some("duck-c51b"));
        assert!(target.from_env);
        assert_eq!(target.source(), "DUCK_ROBOT");
        let provenance = target.provenance();
        assert!(
            provenance.contains("DUCK_ROBOT"),
            "names the variable: {provenance}"
        );
        assert!(
            provenance.contains("DUCK_ROBOT= "),
            "and the way out: {provenance}"
        );
    }

    /// A command line beats a shell profile, and whoever typed `--name` does not need telling where
    /// the name came from.
    #[test]
    fn the_flag_beats_the_environment() {
        let target = Target::new(Some("duck-ffff".to_owned()), Some("duck-c51b".to_owned()));

        assert_eq!(target.wanted(), Some("duck-ffff"));
        assert!(!target.from_env);
        assert_eq!(target.source(), "--name");
        assert!(target.provenance().is_empty(), "nothing to disclose");
    }

    /// Empty is unset, which is why clap's own `env` support is not used: it treats `DUCK_ROBOT=` as
    /// a value, and a variable exported in a shell profile could then only be escaped by unsetting
    /// it — for a command being typed on a bench that has somebody else's robot on it.
    #[test]
    fn an_empty_value_is_no_default_at_all() {
        let escaped = Target::new(None, Some(String::new()));
        assert_eq!(escaped.wanted(), None, "`DUCK_ROBOT= duck-btctl …`");
        assert!(
            escaped.provenance().is_empty(),
            "no name, nothing to explain"
        );

        let overridden = Target::new(Some(String::new()), Some("duck-c51b".to_owned()));
        assert_eq!(
            overridden.wanted(),
            None,
            "`--name ''` drops the default too"
        );
    }

    /// The rename works, and then every later command searches for a name nothing answers to — which
    /// reads as a robot that went away rather than a variable that went stale.
    #[test]
    fn a_rename_says_when_it_leaves_the_default_stale() {
        let rename = Command::Name {
            name: "leduckpierre".to_owned(),
        };
        let from_env = Target::new(None, Some("duck-c51b".to_owned()));

        let note = from_env
            .stale_after_rename(&rename)
            .expect("the environment still says the old name");
        assert!(note.contains("duck-c51b"), "what to change: {note}");
        assert!(note.contains("leduckpierre"), "what it is now: {note}");

        let typed = Target::new(Some("duck-c51b".to_owned()), None);
        assert!(
            typed.stale_after_rename(&rename).is_none(),
            "a `--name` typed once is not still in force"
        );
        assert!(
            from_env
                .stale_after_rename(&Command::Name {
                    name: "duck-c51b".to_owned()
                })
                .is_none(),
            "renamed to the name it already answers to"
        );
        assert!(
            from_env.stale_after_rename(&Command::Info).is_none(),
            "nothing was renamed"
        );
    }

    /// `scan` with a default set is read to answer one question — is my robot here — and otherwise
    /// leaves it as a string comparison done by eye against a column of hex.
    #[test]
    fn a_listing_marks_the_robot_the_default_names() {
        let target = Target::new(None, Some("duck-c51b".to_owned()));

        assert!(target.marks(Some("duck-c51b")));
        assert!(
            target.marks(Some("radxa-zero3 [duck-c51b]")),
            "the macOS pair"
        );
        assert!(!target.marks(Some("duck-ffff")));
        assert!(
            !target.marks(None),
            "an unnamed device is not the named one"
        );
        assert!(
            !Target::new(None, None).marks(Some("duck-c51b")),
            "with no default there is nothing to mark"
        );
    }

    /// One advertisement, as `btleplug` would report it.
    fn advertised(
        duck: bool,
        manufacturer_data: &[(u16, Vec<u8>)],
    ) -> (PeripheralProperties, bool) {
        (
            PeripheralProperties {
                manufacturer_data: manufacturer_data.iter().cloned().collect(),
                ..Default::default()
            },
            duck,
        )
    }

    /// The whole point of the change: a listing says where to reach the robot, with no connection.
    #[test]
    fn a_robot_broadcasts_where_it_is() {
        let (properties, duck) = advertised(
            true,
            &[(
                adv::COMPANY_ID,
                adv::address_data(Some(Ipv4Addr::new(192, 168, 1, 42))),
            )],
        );
        let address = Address::read(&properties, duck);
        assert_eq!(address, Address::At(Ipv4Addr::new(192, 168, 1, 42)));
        assert_eq!(address.note().as_deref(), Some("192.168.1.42"));
    }

    /// The two blanks are not one blank. A robot with no wifi is a wifi problem; a robot that said
    /// nothing is an update — and the listing sends the reader somewhere different for each.
    #[test]
    fn no_wifi_and_no_field_read_differently() {
        let (properties, duck) = advertised(true, &[(adv::COMPANY_ID, adv::address_data(None))]);
        assert_eq!(Address::read(&properties, duck), Address::Unassigned);
        assert_eq!(
            Address::read(&properties, duck).note().as_deref(),
            Some("no address")
        );

        let (properties, duck) = advertised(true, &[]);
        assert_eq!(Address::read(&properties, duck), Address::Unsaid);
        assert_eq!(
            Address::read(&properties, duck).note(),
            None,
            "nothing on the line; the note under the list covers it"
        );
    }

    /// `0xFFFF` is the company id the SIG leaves open to anyone, so four bytes of it on a device that
    /// never advertised the duck service are somebody else's four bytes. Listing an earbud with an
    /// invented address would be worse than listing it with none.
    #[test]
    fn only_a_robot_is_read_for_an_address() {
        let (properties, duck) = advertised(
            false,
            &[(
                adv::COMPANY_ID,
                adv::address_data(Some(Ipv4Addr::new(10, 0, 0, 1))),
            )],
        );
        assert_eq!(Address::read(&properties, duck), Address::Unsaid);
    }

    /// The PIN matters more than the name does — a robot with a real one needs it on every
    /// command — and an empty `DUCK_PIN` left over from a script must not become the PIN, or the
    /// robot answers "wrong PIN" for a PIN nobody chose.
    #[test]
    fn the_pin_falls_back_through_the_environment_to_the_factory_default() {
        let six = |pin: &str| Some(pin.to_owned());

        assert_eq!(resolve_pin(six("111111"), six("222222")), "111111");
        assert_eq!(resolve_pin(None, six("222222")), "222222");
        assert_eq!(resolve_pin(None, None), DEFAULT_PIN);
        assert_eq!(resolve_pin(None, Some(String::new())), DEFAULT_PIN);
        assert_eq!(resolve_pin(Some(String::new()), six("222222")), "222222");
    }

    /// The split is a guess and it can be wrong: a robot whose own name ends in a bracket group is
    /// indistinguishable from the composite, so its halves are accepted as well. Tolerated rather
    /// than fixed — btleplug joins the two names before we see them, and the pair is gone — because
    /// the cost is only that an explicit `--name` matches more, on names nobody gives a robot. What
    /// has to hold is that the shape must actually be there.
    #[test]
    fn the_split_needs_the_shape_it_looks_for() {
        assert!(answers_to("duck [1]", "duck [1]"));
        assert!(!answers_to("[duck-c51b]", "duck-c51b"));
        assert!(!answers_to("duck-c51b [", "duck-c51b"));
    }
}
