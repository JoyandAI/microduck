//! Which calls BLE may make, which socket answers them, and which connection carries them.
//!
//! BLE exposes a **subset** of the robot API (`architecture.md` §4.1): provisioning, status,
//! and the update commands with their progress. It is too slow and too constrained for the full
//! surface, and — more to the point — a radio anybody within a few metres can talk to is not
//! the transport over which to offer "reset this robot to factory state".
//!
//! One table decides all three questions, because they are the same question asked about the
//! same call: a call is permitted exactly when this file names the service that answers it, and
//! the [`Lane`] it names is which of that service's connections it travels on.
//!
//! **The match is deliberately exhaustive.** Adding a variant to [`proto::Call`] makes this
//! file fail to compile, so a new method cannot reach the radio because someone forgot this
//! file existed. A `_ => None` wildcard would be the safe default in the moment and the wrong
//! one over time: it would silently deny new methods, and the first symptom would be a phone
//! app that cannot see a feature nobody remembered to route. The lane lives in the same table
//! for the same reason — a new long-running method given the wrong lane is a session that
//! stops answering, which is [`Lane`]'s whole subject.

use duck_ipc_proto as proto;

/// The service that owns the answer to a call.
///
/// One socket per service, connected directly — there is no broker (`architecture.md` §2.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Upstream {
    /// `updaterd`, at `proto::DEFAULT_SOCKET`.
    Updater,
    /// `robotd`.
    Robot,
    /// `configd` — wifi and the robot's identity.
    Config,
}

/// Which connection to a service carries a call.
///
/// **Every daemon here serves one connection one request at a time**: `updaterd` and `configd`
/// both read a line, await the whole call, and only then read the next
/// (`updater/src/ipc.rs::handle_connection`, `configd/src/main.rs::handle`). A single connection
/// per service would therefore put every call on one queue behind the slowest thing on it, and
/// two orderings a phone app reaches for first are broken by that:
///
/// - `update.apply` then `update.status` — the status line waits in a socket `updaterd` will not
///   read for minutes, so the client times out having heard nothing while the robot is fine.
/// - `update.subscribe` then `update.apply` — worse. `stream_progress` owns its connection until
///   the peer goes away and never reads another request, so the apply is written into a socket
///   nobody reads: it never runs, never replies and never errors. An update the owner asked for
///   that the robot silently did not perform.
///
/// So calls are grouped by *how long they hold a connection*, and each group gets its own. Four
/// lanes is at most four sockets per service per session, which costs nothing and is bounded
/// without any bookkeeping — the alternative, a connection per call, needs `btd` to know when a
/// call ended, which needs it to parse replies. It deliberately never does (`session`).
///
/// Calls that share a lane are queued behind each other, and each grouping says why that is the
/// right answer rather than a compromise.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Lane {
    /// Answers as fast as the daemon can look something up. Reads from memory, and the writes
    /// that only store a value.
    ///
    /// Queueing these behind each other is invisible, and keeping them off the other lanes is
    /// the point: a status poll during an update must answer, and `updaterd` is built so it can
    /// — `update.status` falls back to a cached snapshot rather than waiting for the engine
    /// (`updater/src/ipc.rs`). That fallback is wasted if the request never gets read.
    Prompt,
    /// Seconds: a read that goes to the network or sweeps a radio.
    ///
    /// Two of them, and queueing them behind each other is fine. What matters is that neither
    /// sits in front of a `Prompt` call nor behind an `Operation` one — `update.check` during an
    /// update should come straight back `BUSY`, not resolve whenever the update finishes.
    Slow,
    /// As long as it takes, and it changes the robot: an update, joining a network, bonding a pad.
    ///
    /// Minutes, for a daemon update. Sharing one lane is correct rather than tolerated: `updaterd`
    /// single-flights mutations behind a file lock and answers `BUSY` for a second one, and two
    /// radio operations at once on `configd` is not a thing to want either. So a second
    /// `Operation` waiting for the first is the behaviour to have.
    Operation,
    /// Never answers. The service writes notifications until the peer goes away.
    ///
    /// One call, `update.subscribe`, and it must be alone: a connection handed to a progress
    /// stream reads no further requests at all.
    Stream,
}

/// What happens to a call that arrives over BLE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    /// Forwarded verbatim to a service, on that service's connection for this lane.
    To(Upstream, Lane),
    /// Answered by `btd` itself. Only `system.authenticate`: the PIN check belongs to the
    /// transport, because BLE cannot express a fixed printed passkey and the check therefore had
    /// to move up a layer (`docs/design/app-path-design.md` §5).
    Local,
    /// Not available over this transport.
    Refused,
}

/// Where this call goes and on which connection, or `None` if BLE may not make it.
///
/// Read the `None` arms as the security boundary: each one is a deliberate decision that a
/// phone in the room does not get to do this.
pub fn destination_for(call: &proto::Call) -> Option<(Upstream, Lane)> {
    use Lane::*;
    use Upstream::*;
    use proto::Call::*;
    match call {
        // The version handshake. Must be reachable or no client can establish anything.
        Hello(_) => Some((Updater, Prompt)),

        // ── the update subset §4.1 names ────────────────────────────────────
        //
        // `Apply` is intended: BLE implies physical presence plus pairing (§4.2), and "update
        // the robot from the phone" is M6's headline. It also has to pass `updaterd`'s own peer
        // policy, and does — `deploy/updater.toml` names `btd` in `allow_users`, which is a
        // narrower claim than granting the robot group. Routing it here without that grant would
        // have produced a phone button that always returned PERMISSION_DENIED.
        Apply(_) => Some((Updater, Operation)),
        // Reaches the network, so `Slow` — and off the `Operation` lane deliberately, because
        // "is there an update?" asked during one has an immediate answer (`BUSY`) and queueing
        // it would turn that into a spinner that resolves minutes later.
        Check(_) => Some((Updater, Slow)),
        Status => Some((Updater, Prompt)),
        Subscribe => Some((Updater, Stream)),
        // Read-only, and what support asks for first. `update.log` is the record that
        // survives a wiped journal (§8.2), so a phone able to read it is worth having.
        Log(_) => Some((Updater, Prompt)),
        ListInstalled(_) => Some((Updater, Prompt)),

        // Going back. Both are permitted, and both are less consequential than the `Apply`
        // above them: they move the robot to a release that has already run on this board,
        // download nothing, and are gated and auto-reverted like any other transition
        // (`Engine::rollback` and `Engine::select` both go through `transition_to`).
        //
        // They were refused until the update path was driven from a phone, on the reasoning that
        // the engine reverts a bad release on its own. It does — the one that fails its health
        // gate. That is not the case an owner reaches for a phone about, which is a release that
        // installs, passes its gate, and then behaves *worse*: a policy that walks unsteadily
        // rather than not at all, a pad that stops reconnecting. Nothing reverts that but a
        // person, and the person is holding a phone and has no ssh.
        //
        // `Rollback` is the undo — the previous release, no arguments, one tap. `Select` is the
        // same authority plus a version number, and it is what a list of installed releases is
        // *for*: `ListInstalled` is already routed above, so an app can show them, and being able
        // to show them without being able to choose one would be the odd half.
        Rollback(_) => Some((Updater, Operation)),
        Select(_) => Some((Updater, Operation)),

        // Is the robot alright? The one `robot.*` call an app has any use for.
        RobotHealth => Some((Robot, Prompt)),

        // ── provisioning, which is what §4.1 puts BLE here for ──────────────
        //
        // This is the case the whole transport exists to serve: a robot that has never seen a
        // network cannot be configured over that network, so BLE is the only way in. All four
        // are permitted, including the two that change things.
        NetStatus => Some((Config, Prompt)),
        // `configd` re-sweeps the radio rather than returning the last scan, which takes seconds.
        NetScan => Some((Config, Slow)),
        // Carries a wifi passphrase, which §7 requires to travel over a paired, encrypted link.
        // It does: the characteristic sets `encrypt_authenticated_write` and the PIN agent makes
        // the bond an authenticated one (`crate::pairing`). Routing this before that existed
        // would have been the ordering mistake.
        //
        // `Operation` because `configd` polls NetworkManager for up to 45 seconds before calling
        // a join failed, and a phone showing "connecting…" wants `net.status` to keep answering
        // throughout — which is the same defect as the update one, on the path BLE exists for.
        NetConnect(_) => Some((Config, Operation)),
        NetForget(_) => Some((Config, Prompt)),

        // Name and identity. Renaming from the app is the reason `system.setName` exists.
        SystemInfo => Some((Config, Prompt)),
        SystemSetName(_) => Some((Config, Prompt)),

        // Which daemons are up and which release each is running. Routed because an app that can
        // trigger an update should be able to show whether it took — and because the one daemon it
        // cannot report on this way is `btd` itself, which answering at all proves is running.
        SystemServices => Some((Config, Prompt)),

        // Rebooting is drastic but recoverable, and it is what an app offers when a robot is
        // confused — the alternative being "unplug it", which for a walking robot is worse.
        // Unlike `resetToGolden` it discards nothing.
        SystemReboot => Some((Config, Prompt)),

        // ── the gamepad ─────────────────────────────────────────────────────
        //
        // Pairing a controller from the phone, which is where it belongs: whoever is holding the
        // robot is holding the pad, and the alternative is an ssh session. The same physical-presence
        // argument §4.2 makes for `net.connect` covers it — a pad has to be in the room, in pairing
        // mode, in a fifteen-second window — and it is `configd` that does the work either way.
        //
        // `pad.pair` is the more consequential of the two, because a bonded pad can enable the
        // policy afterwards. That is deliberate: it is the same authority as standing next to the
        // robot with a controller, and the PIN gate is what stands in front of it. It waits on a
        // pad for its whole timeout, so it shares the `Operation` lane with the wifi join.
        PadStatus => Some((Config, Prompt)),
        PadPair(_) => Some((Config, Operation)),
        PadForget(_) => Some((Config, Prompt)),

        // Answered by `btd`, so it has no upstream. See `route_for`.
        SystemAuthenticate(_) => None,

        // The pairing PIN, and the one refusal in this file that is load-bearing rather than
        // conservative: a PIN readable by an unpaired peer authorises nothing at all. `btd`
        // reads it over the unix socket to answer BlueZ's passkey request, and BLE never can.
        SystemPairingPin | SystemSetPairingPin(_) => None,

        // ── refused ─────────────────────────────────────────────────────────

        // Pinning, and it stays refused while `Select` above it does not. The difference is what
        // the mistake looks like afterwards: a wrong `select` is one release away from being
        // undone and the robot says which release it is on, whereas a robot pinned by a mistap
        // refuses every later update and reports itself as up to date. That is the one failure
        // here that looks exactly like correct behaviour, and it needs `robotctl` and a person
        // who meant it.
        Pin(_) => None,

        // Factory reset in all but name: back to the golden image, discarding every release
        // since. Never over a radio — and note that `Rollback` and `Select` being routed does
        // not weaken this, because neither discards anything.
        ResetToGolden(_) => None,

        // `updaterd`'s private questions to `robotd` — may I restart the control loop, which
        // model API is this, is a telepresence session live. Internal plumbing of the update
        // decision, of no use to a client and misleading if exposed: a phone reading
        // `safeToRestart` would learn nothing it could act on.
        RobotSafeToRestart | RobotModelApi | RobotRemoteSessionActive => None,

        // Motor control. **Never over BLE**, which is what §4.1 means by a subset: BLE is too
        // slow and too constrained for the full surface, and teleop belongs on WebRTC's
        // unreliable `teleop` datachannel where a stale command is dropped rather than
        // retransmitted (§5.2). A 20-byte notification budget and a link that does not exist for
        // the first ~73s of a boot is not a control transport. The skills, the body pose and
        // the mouth are motor control like the rest.
        RobotMove(_) | RobotHead(_) | RobotLook(_) | RobotEnable(_) | RobotDo(_) | RobotPose(_)
        | RobotMouth(_) => None,

        // Harmless and rather charming from a phone — but it rides the same refusal as the
        // rest of robot.* until the app path exists to want it: opening one call to the
        // radio ahead of a client that can use it buys nothing and widens the surface.
        //
        // The theremin sits here rather than with motor control even though it moves the
        // mouth, because what it is is a sound: the mouth is following the note. Same
        // refusal either way, and the same reason to lift it — an app that can play the duck.
        RobotSound(_) | RobotTheremin(_) | RobotChorale(_) => None,

        // The chorale's own namespace is between `btd` and `robotd` — it is how this daemon is told
        // what to advertise and how it reports what it heard. Not a client surface at all, so a
        // phone asking for it is asking for something that does not exist for it.
        ChoraleSubscribe | ChoraleBeaconSet(_) | ChoraleHeard(_) => None,

        // Powering the machine off from a phone in the room is `system.reboot` without the
        // coming back. The sit-then-power-off flow wants whoever asked to be watching the
        // robot, and that is `robotctl` or the pad's long-press, deliberately.
        RobotShutdown => None,

        // Constant for the life of the process, and only a stick-mapping hint for local
        // clients like `padd`. An app gets the same answer through `system.info` territory
        // when it ever needs one; no reason to open another read to the radio today.
        RobotMode => None,

        // Power to the joints. A phone button that drops the robot on the floor is not one to
        // offer, and `robot.init` is its counterpart: standing a robot up moves every joint at once,
        // which wants the person doing it to be looking at the robot rather than at a screen. Both
        // are `robotctl` on the robot, deliberately.
        RobotInit | RobotRelax => None,

        // `robot.stop` deserves its own line, because refusing it looks wrong. An emergency stop
        // in the app is exactly what someone reaches for, and §6 does say local should preempt
        // remote — but a stop button that works over an unbonded, high-latency, sometimes-absent
        // radio is worse than no button, because it *looks* like an e-stop and is not one. The
        // deadman in `robotd` already stops the robot when intents stop arriving, which is the
        // mechanism that does not depend on a phone being in range. A real e-stop is physical.
        // Reconsider deliberately if the app ever needs it, with that caveat stated in the UI.
        RobotStop => None,

        // High-rate telemetry. `robot.subscribe` streams state at up to the control rate; over
        // BLE that is a firehose into a 20-byte pipe, and a client would get a decimated,
        // unpredictably-lagged view it could not reason about. `robot.health` is the question an
        // app actually has.
        RobotSubscribe(_) => None,

        // The same objection as `robot.subscribe`, only more so: this is every evdev event the pad
        // sends, over a hundred reports a second, and it exists to *measure the cadence of its own
        // delivery*. Carried over BLE the measurement would be of the phone's link rather than the
        // pad's, which is worse than refusing — it would be a number that looks like an answer.
        //
        // It is also not `btd`'s to forward. Every route here goes to one of three sockets `btd`
        // holds, and this one is served by `padd`, which is deliberately not among them: `padd` is
        // the unprivileged client whose whole value is having no special access, and giving the BLE
        // transport a connection to it would be the first thing to make that untrue.
        PadInput => None,

        // Depth frames, and the same two objections as the pad tap. A 64-zone frame
        // fifteen times a second is a firehose into a 20-byte pipe; and it is served by
        // `tofd`, which is not one of the three sockets `btd` holds. When a phone has a
        // reason to see what the robot sees, it will be through `mediad`'s video path
        // (`architecture.md` §5.2), where depth belongs next to the frame it annotates.
        TofStream => None,
    }
}

/// The service that answers a call, ignoring which connection carries it.
///
/// The permission question on its own, which is what most callers and every test about the
/// security boundary are asking.
pub fn upstream_for(call: &proto::Call) -> Option<Upstream> {
    destination_for(call).map(|(upstream, _)| upstream)
}

/// The full routing decision, including the one call the transport answers itself.
pub fn route_for(call: &proto::Call) -> Route {
    match call {
        proto::Call::SystemAuthenticate(_) => Route::Local,
        other => match destination_for(other) {
            Some((upstream, lane)) => Route::To(upstream, lane),
            None => Route::Refused,
        },
    }
}

/// The JSON-RPC error to answer a refused call with.
///
/// [`proto::code::PERMISSION_DENIED`] rather than `METHOD_NOT_FOUND`, because the two mean
/// different things to whoever is holding the phone: this method exists and this transport
/// may not use it — "try `robotctl`", not "upgrade your app".
pub fn refusal(call: &proto::Call) -> proto::Error {
    proto::Error::new(
        proto::code::PERMISSION_DENIED,
        format!(
            "{} is not available over Bluetooth; use robotctl on the robot",
            call.method()
        ),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use duck_ipc_proto::{ComponentId, semver};

    fn component() -> ComponentId {
        ComponentId::new("daemon")
    }

    /// Exactly which mutating calls BLE may make, named one by one.
    ///
    /// The list is the security boundary, so it is spelled out rather than counted: adding a
    /// mutating method and routing it should have to change this line and say why in the
    /// commit. `update.apply` is the update trigger §4.1 names; the rest are provisioning,
    /// which is what BLE is *for* — a robot that has never seen a network cannot be configured
    /// over that network.
    #[test]
    fn only_these_mutating_calls_are_reachable_over_ble() {
        let mutating_and_allowed: Vec<&str> = every_call()
            .iter()
            .filter(|c| c.is_mutating() && upstream_for(c).is_some())
            .map(proto::Call::method)
            .collect();

        assert_eq!(
            mutating_and_allowed,
            vec![
                proto::method::APPLY,
                // Going back, both of them. Routed when the update path was driven from a phone:
                // an owner whose robot got worse after an update has no other way to undo it, and
                // neither call discards anything or downloads anything.
                proto::method::ROLLBACK,
                proto::method::SELECT,
                proto::method::NET_CONNECT,
                proto::method::NET_FORGET,
                proto::method::SYSTEM_SET_NAME,
                proto::method::SYSTEM_REBOOT,
                // Bonding a gamepad, which afterwards can enable the walking policy. Allowed for
                // the same reason as provisioning: it takes a pad held in pairing mode next to the
                // robot, so BLE's physical-presence claim (§4.2) is not being stretched — and the
                // alternative is an ssh session, which is not a thing an owner has.
                proto::method::PAD_PAIR,
                proto::method::PAD_FORGET,
            ]
        );
    }

    /// Pairing a controller from the phone reaches `configd`, which is the service that owns the
    /// radio's configuration. `btd` must not answer this itself: it owns nothing (§4.1).
    #[test]
    fn a_pad_can_be_paired_from_the_phone() {
        for call in [
            proto::Call::PadStatus,
            proto::Call::PadPair(proto::PadPairParams::default()),
            proto::Call::PadForget(proto::PadForgetParams {
                mac: "78:86:2E:BB:13:28".into(),
            }),
        ] {
            assert_eq!(
                upstream_for(&call),
                Some(Upstream::Config),
                "{}",
                call.method()
            );
        }
    }

    /// The PIN must never be readable or writable over the radio.
    ///
    /// This is the one refusal here that is not merely cautious: pairing is what authorises a
    /// BLE client at all (§4.2), and a passkey an unpaired peer could ask for — or worse,
    /// overwrite — would make the whole mechanism theatre. `btd` gets it over the unix socket.
    #[test]
    fn the_pairing_pin_is_not_reachable_over_ble() {
        assert_eq!(upstream_for(&proto::Call::SystemPairingPin), None);
        assert_eq!(
            upstream_for(&proto::Call::SystemSetPairingPin(
                proto::SetPairingPinParams {
                    pin: "000000".into()
                }
            )),
            None
        );
    }

    /// Provisioning must be reachable, and reach `configd` — the case BLE exists for.
    #[test]
    fn provisioning_reaches_configd() {
        for call in [
            proto::Call::NetStatus,
            proto::Call::NetScan,
            proto::Call::NetConnect(proto::NetConnectParams {
                ssid: "Home".into(),
                psk: None,
            }),
            proto::Call::NetForget(proto::NetForgetParams {
                ssid: "Home".into(),
            }),
            proto::Call::SystemInfo,
            proto::Call::SystemSetName(proto::SetNameParams {
                name: "duck".into(),
            }),
            proto::Call::SystemReboot,
        ] {
            assert_eq!(
                upstream_for(&call),
                Some(Upstream::Config),
                "{}",
                call.method()
            );
        }
    }

    /// The refusals, named individually. If a future change makes one of these reachable it
    /// should have to delete a line here and say why in the commit.
    ///
    /// Two lines were deleted from it when the update path was driven from a phone —
    /// `update.rollback` and `update.select` — and the reasoning is on their arms in
    /// `destination_for`. What is left is a factory reset, a pin whose mistake looks like correct
    /// behaviour, and `updaterd`'s private questions to `robotd`.
    #[test]
    fn the_refused_calls_stay_refused() {
        for call in [
            proto::Call::ResetToGolden(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Pin(proto::PinParams {
                component: component(),
                version: None,
            }),
            proto::Call::RobotSafeToRestart,
            proto::Call::RobotModelApi,
            proto::Call::RobotRemoteSessionActive,
        ] {
            assert_eq!(upstream_for(&call), None, "{}", call.method());
        }
    }

    /// A phone must be able to establish a session, see the robot's state, start an update
    /// and watch it. Without all four the transport is not useful for what it exists to do.
    #[test]
    fn the_app_path_is_reachable() {
        let expected = [
            (
                proto::Call::Hello(proto::HelloParams {
                    api_version: proto::API_VERSION,
                }),
                Upstream::Updater,
            ),
            (proto::Call::Status, Upstream::Updater),
            (proto::Call::Subscribe, Upstream::Updater),
            (proto::Call::RobotHealth, Upstream::Robot),
        ];
        for (call, want) in expected {
            assert_eq!(upstream_for(&call), Some(want), "{}", call.method());
        }
    }

    /// A refusal must be distinguishable from "no such method", because the two ask the user
    /// for different things.
    #[test]
    fn a_refusal_says_permission_denied_and_names_the_method() {
        let call = proto::Call::ResetToGolden(proto::ComponentParams {
            component: component(),
        });
        let err = refusal(&call);

        assert_eq!(err.code, proto::code::PERMISSION_DENIED);
        assert!(
            err.message.contains(proto::method::RESET_TO_GOLDEN),
            "{}",
            err.message
        );
    }

    /// Nothing a phone does during an update may share a connection with the update.
    ///
    /// This is the defect the lanes exist for, and it is asserted as a property rather than as a
    /// table: whatever else changes, `update.apply` must not be able to block a status poll, a
    /// check, or the progress stream, because every daemon here serves one connection one request
    /// at a time. The three calls below are the three an app makes *while* an update runs.
    #[test]
    fn an_apply_shares_its_connection_with_nothing_a_client_does_during_one() {
        let apply = destination_for(&proto::Call::Apply(proto::ApplyParams {
            component: component(),
            target: proto::Target::Latest,
            options: proto::ApplyOptions::default(),
        }))
        .expect("apply is routed");

        for call in [
            proto::Call::Status,
            proto::Call::Subscribe,
            proto::Call::Check(proto::ComponentParams {
                component: component(),
            }),
        ] {
            let during = destination_for(&call).expect("routed");
            assert_eq!(during.0, apply.0, "{} is served by updaterd", call.method());
            assert_ne!(
                during.1,
                apply.1,
                "{} would queue behind an apply",
                call.method()
            );
        }
    }

    /// The progress stream must be alone on its lane, which is a stronger claim than the test
    /// above: a connection handed to `stream_progress` reads no further requests *ever*, so a
    /// second call sharing it is not delayed but lost.
    #[test]
    fn nothing_else_travels_on_the_stream_lane() {
        let others: Vec<&str> = every_call()
            .iter()
            .filter(|c| !matches!(c, proto::Call::Subscribe))
            .filter(|c| destination_for(c).is_some_and(|(_, lane)| lane == Lane::Stream))
            .map(proto::Call::method)
            .collect();

        assert_eq!(others, Vec::<&str>::new());
        assert_eq!(
            destination_for(&proto::Call::Subscribe).map(|(_, lane)| lane),
            Some(Lane::Stream)
        );
    }

    /// A call that holds its connection for as long as the robot needs is never on the lane the
    /// quick answers use. Named one by one, because the cost of getting one wrong is a session
    /// that stops answering and the fix is one word.
    #[test]
    fn the_calls_that_take_their_time_are_off_the_prompt_lane() {
        for call in [
            proto::Call::Apply(proto::ApplyParams {
                component: component(),
                target: proto::Target::Latest,
                options: proto::ApplyOptions::default(),
            }),
            proto::Call::Rollback(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Select(proto::SelectParams {
                component: component(),
                version: semver::Version::new(1, 0, 0),
            }),
            proto::Call::Check(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::NetScan,
            proto::Call::NetConnect(proto::NetConnectParams {
                ssid: "Home".into(),
                psk: None,
            }),
            proto::Call::PadPair(proto::PadPairParams::default()),
        ] {
            let (_, lane) = destination_for(&call).expect("routed");
            assert_ne!(lane, Lane::Prompt, "{}", call.method());
        }
    }

    /// Going back is reachable, and reaches `updaterd`. The pair of them is what §2.4 of
    /// `docs/project/update-over-ble.md` decided.
    #[test]
    fn going_back_is_reachable_from_the_phone() {
        for call in [
            proto::Call::Rollback(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Select(proto::SelectParams {
                component: component(),
                version: semver::Version::new(0, 5, 1),
            }),
        ] {
            assert_eq!(
                destination_for(&call),
                Some((Upstream::Updater, Lane::Operation)),
                "{}",
                call.method()
            );
        }
    }

    /// Every variant, so the tests above cannot silently skip one. The exhaustive match in
    /// `upstream_for` is what forces this list to be maintained: a new variant breaks the
    /// build there, and whoever fixes it arrives here next.
    fn every_call() -> Vec<proto::Call> {
        let version = semver::Version::new(1, 4, 2);
        vec![
            proto::Call::Hello(proto::HelloParams {
                api_version: proto::API_VERSION,
            }),
            proto::Call::Check(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Apply(proto::ApplyParams {
                component: component(),
                target: proto::Target::Latest,
                options: proto::ApplyOptions::default(),
            }),
            proto::Call::Rollback(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::ResetToGolden(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Select(proto::SelectParams {
                component: component(),
                version: version.clone(),
            }),
            proto::Call::Pin(proto::PinParams {
                component: component(),
                version: Some(version),
            }),
            proto::Call::Status,
            proto::Call::ListInstalled(proto::ComponentParams {
                component: component(),
            }),
            proto::Call::Log(proto::LogParams { limit: 20 }),
            proto::Call::Subscribe,
            proto::Call::RobotSafeToRestart,
            proto::Call::RobotHealth,
            proto::Call::RobotModelApi,
            proto::Call::RobotRemoteSessionActive,
            proto::Call::NetStatus,
            proto::Call::NetScan,
            proto::Call::NetConnect(proto::NetConnectParams {
                ssid: "Home".into(),
                psk: Some("secret".into()),
            }),
            proto::Call::NetForget(proto::NetForgetParams {
                ssid: "Home".into(),
            }),
            proto::Call::SystemInfo,
            proto::Call::SystemSetName(proto::SetNameParams {
                name: "duck".into(),
            }),
            proto::Call::SystemReboot,
            proto::Call::RobotMove(proto::MoveParams {
                vx: 0.1,
                vy: 0.0,
                vyaw: 0.0,
            }),
            proto::Call::RobotHead(proto::HeadParams {
                neck_pitch: 0.0,
                head_pitch: 0.0,
                head_yaw: 0.0,
                head_roll: 0.0,
            }),
            proto::Call::RobotStop,
            proto::Call::RobotEnable(proto::EnableParams {
                on: true,
                toggle: false,
            }),
            proto::Call::RobotInit,
            proto::Call::RobotRelax,
            proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(10) }),
            proto::Call::SystemPairingPin,
            proto::Call::SystemSetPairingPin(proto::SetPairingPinParams {
                pin: "000000".into(),
            }),
            proto::Call::PadStatus,
            proto::Call::PadPair(proto::PadPairParams::default()),
            proto::Call::PadForget(proto::PadForgetParams {
                mac: "78:86:2E:BB:13:28".into(),
            }),
        ]
    }
}
