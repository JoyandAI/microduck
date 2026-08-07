# Telling robots apart

Status: **proposed, not built** · Date: 2026-08-07 · Owner: pierre

Three friends, three robots, one room. Each must reach *theirs*. Today none of them can, and the
failure is not the one it looks like.

## Not a UX problem

Three facts, each verified rather than assumed:

- **The advertised name is the hostname.** `configd`'s `Store` falls back to `hostname()` when no name
  has been set, and `btd` advertises that as the BLE `local_name`. Every board flashed from one image
  advertises `radxa-zero3`.
- **There is no serial.** `system.info` returns `serial: null`. The code comment points at
  `updater-design.md` §5.7, but that section is about robot-specific state *surviving updates* — the
  identity itself is specified nowhere.
- **The PIN is `000000` on all of them**, the documented default (`app-path-design.md` §5.3).

Individually each is a rough edge. Together they are a security failure: a phone cannot merely pick
the wrong robot from a list, it can **authenticate to it and write its owner's wifi credentials into
it**. Picking wrong is a mistake; being able to configure it is a breach. Identity is therefore a
shipping prerequisite alongside encryption (§8.1), not a nicety that follows it.

## What has to exist

### 1. A per-device identity

Everything below hangs off this, and it is the one open decision.

| option | for | against |
|---|---|---|
| **Derive from a hardware address** (wifi or Bluetooth MAC) | zero infrastructure, no provisioning step, stable across reflashes, available before any state exists | leaks a hardware address into an advertisement a stranger can collect; two radios means choosing which, and a replaced module changes identity |
| **Assign at provisioning** | clean, controllable, can be printed on the robot before it ships, decoupled from hardware | there is no provisioning step today, so this invents one, and a board flashed by hand has no identity at all |
| **Generate on first boot**, store beside the config | self-contained, no infrastructure, survives updates via the state dir | two boards imaged from the *same* filesystem snapshot share it, which is exactly how these boards are made; needs a "not yet generated" state that is genuinely per-board |

The Bluetooth adapter address is the most apt of the derived options — it is the identity the peer
already sees at the link layer, so advertising a name derived from it leaks nothing new. That
observation is worth weighing against the general objection to hardware-derived identity.

Whatever it is, it belongs in `Store`'s config next to `name` and `pairing_pin`, which already lives in
the state directory and therefore survives updates.

### 2. A default name derived from it

`duck-7f3a`, not `radxa-zero3`. Robots must be distinguishable **out of the box**, before anyone has
renamed anything — the case where three people unbox together is precisely the one that fails today.
`system.setName` stays as it is; this replaces the `hostname()` fallback.

### 3. A per-robot PIN

`app-path-design.md` §5.3 already requires this before shipping; identity makes it actionable, because
a per-robot PIN has to be *derived from or recorded against* something per-robot. `PairingPinResult`
already carries `is_default`, so "this robot still has the factory PIN" is observable today and a
client can warn.

### 4. Something printed on the robot

Name and PIN on a sticker turns "connect to the one in front of me" from a guess into a **check**. It
is also what makes a per-robot PIN usable rather than a support burden. A QR code carrying the name
lets an app skip the list entirely.

### 5. An `identify` action

"Make *this* robot nod, blink or chirp", so a human confirms before configuring. Two constraints, both
awkward, neither about plumbing:

- **It cannot be a motor move.** Motor control is refused over BLE by design (§3.1), and carving an
  exception into that boundary to wave a head is a poor trade. It needs its own narrowly-scoped
  action, which is a policy decision about what the boundary means.
- **It has to work before authentication.** Requiring the PIN first is circular: aiming the PIN at the
  right robot is the problem being solved. So `identify` joins `hello` and `system.authenticate` in
  the pre-auth set, and the cost is real — anyone in radio range can make robots chirp. Probably
  acceptable, and it should be a deliberate decision rather than a consequence.

### 6. Client side

- **RSSI as a sort key, never as identity.** Fine for putting the nearest robot first; not evidence,
  because signal through a body or a table reorders robots freely.
- **Store the peripheral identifier after a first successful connection.** On iOS,
  `retrievePeripherals(withIdentifiers:)` then removes the guesswork entirely — see §3.3, which
  records why a service-filtered scan misses a bonded robot.

## Open decisions

1. **Where identity comes from** — the table above. Everything else waits on it.
2. **Whether `identify` is pre-auth**, and what it is allowed to actuate.
3. **Whether the per-robot PIN is derived from the identity or independent.** Derived is one secret to
   print; independent means a leaked identity does not imply a PIN.
4. **What a factory reset does to it.** A name should reset; an identity probably should not.

## Testing

The identity and naming are ordinary unit tests against `Store`. What is not testable off-board is the
scenario that motivates it: three robots advertising at once, and a client choosing correctly. That
wants either three boards or a fake peripheral, and it is worth saying plainly that the suite will not
cover the case this document exists for.
