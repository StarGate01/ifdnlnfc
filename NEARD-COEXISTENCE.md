# Coexisting with neard on a shared NFC adapter

## How we got here

This started from adding KDE Plasma desktop NFC integration (the
`invent.kde.org/vkrause/nfc-integration` project: a `neardevil` kded module
plus a Plasma applet) on top of `neard`, packaged in a separate NixOS
config. For that integration to behave like a normal "tap a tag and
something happens" desktop feature, `neard` needed
`ConstantPoll = true` (`src/main.c`/`src/adapter.c` in neard's own tree):
without it, `neard` polls once, stops the moment anything is found, and
never resumes on its own — `neardevil` itself has no code path that calls
`Adapter.StartPollLoop()` again after the initial tag interaction (checked
all of `src/neardevil/neardevil.cpp` for any `TagLost`/`InterfacesRemoved`
handling that might do this; there isn't any). So `ConstantPoll` is the only
thing that makes repeat taps work at all.

That raised the obvious question: this machine already uses the *same*
physical NXP1001/npc300 adapter for `ctap-bridge`'s FIDO2 security-key
support, via `ifdnlnfc` registered as a PC/SC reader with `pcscd`. Would a
`neard` that's now continuously polling conflict with that?

Traced it into the kernel (`net/nfc/core.c`, `net/nfc/netlink.c`,
`net/nfc/nci/core.c` in a local kernel checkout) and confirmed it's a real,
hard problem, not a theoretical one:

- `nfc_start_poll()` (`net/nfc/core.c`) checks a single `dev->polling` flag
  on the shared `struct nfc_dev` and returns `-EBUSY` to anyone else who
  tries to poll while it's set. This is **cross-process**, not per-socket:
  whoever gets there first locks everyone else out, regardless of which
  daemon's netlink socket started it.
- `dev->polling` only clears when a target is actually found
  (`nfc_targets_found()`) or the original caller explicitly stops it. With
  `ConstantPoll=true` and nothing in the field, the adapter's autonomous NCI
  discovery loop just keeps running — `neard` holds the adapter
  **indefinitely** whenever idle, not just for the length of a brief poll
  window.
- There is no cross-process preemption: `nfc_genl_stop_poll()`
  (`net/nfc/netlink.c`) checks that the stopping request's netlink portid
  matches whoever's `poll_req_portid` is on file, and rejects it with
  `-EBUSY` otherwise. `ifdnlnfc` cannot kick `neard` off via `STOP_POLL`, and
  `neard` can't kick `ifdnlnfc` off either. `nfc_dev_down()` fails the same
  way while polling/a target is active, so forcing the adapter off isn't an
  escape hatch.
- The one thing that *is* shared: `NFC_GENL_MCAST_EVENT_NAME` multicast
  events (target found/lost, etc.) go to every subscriber, not just the
  current poll owner. Passive observation is free; only *driving* a poll
  session is exclusive.

So in practice: once `hardware.nfc.enable`'s `ConstantPoll=true` ships,
`ifdnlnfc`'s own `poll_for_targets()` (`src/ifdnlnfc.c:662`) will get
`-EBUSY` from the kernel essentially every time it's called while no NFC tag
is nearby — which, with a continuously-polling `neard`, is most of the time.
Today that failure is silent and unrecovered: `poll_for_targets()` just logs
`"Error %x starting NFC target poll"` and returns; nothing retries except
the next `IFDHICCPresence()` call from pcscd's own polling thread
(`src/ifdnlnfc.c:1731-1732`), which will hit the exact same `-EBUSY` again.
FIDO2 taps would be unreliable or dead on arrival for as long as `neard`
holds the lock.

### Why not just move `ifdnlnfc` onto `neard`'s D-Bus API entirely?

Checked whether `ifdnlnfc` could stop touching the kernel netlink interface
at all and become a pure `neard` D-Bus client instead, which would sidestep
the lock question entirely. It can't:

- `org.neard.Tag` (neard's `src/tag.c:574-577`, the interface for an
  externally-tapped target) only has `Write` (NDEF) and `Deactivate`. No
  raw APDU/transceive method exists for external targets anywhere in
  `neard`'s public API — it's an NDEF-tag/handover daemon by design, not a
  smartcard API.
- The only APDU-capable interface in neard's tree at all is
  `org.neard.se.Channel.SendAPDU` (`doc/secureelement-api.txt`), but that's
  for a **Secure Element** (an embedded eSE/SIM accessed over logical
  channels) — a different NFC concept from a card tapped against the
  reader. It also lives in a separate binary, `se/seeld`
  (`Makefile.am:71`), which isn't even built in the neard package this
  config uses (confirmed: no `seeld` binary, no `SendAPDU`/`SecureElement`
  symbols in the built `neard` executable).

`ifdnlnfc`'s actual job — raw ISO7816 APDU exchange with whatever's on the
reader — has no D-Bus equivalent to move to. It still needs direct kernel
netlink/NCI access for real card I/O regardless of what we do here. D-Bus
cooperation can only ever settle who currently owns the poll lock, not
replace the transceive path.

## Why

Priority should go to the interactive, time-sensitive operation: a FIDO2
tap the user is actively waiting on should win over `neard`'s background
"ready for the next NDEF tag" polling. Today there's no coordination at
all, so the outcome is whichever daemon's netlink call lands first — with
`ConstantPoll` on, that's `neard`, almost always, which is backwards.

This is only about *idle* contention, though — periods where neither side
has an actual target. While a card is genuinely connected (mid-transaction,
or just sitting "present" — `IFDHICCPresence()` short-circuits to
`IFD_SUCCESS` without re-polling once `ifdnlnfc_state.card_present` is set,
so "present" can span the whole time a key sits on the reader, not just one
APDU exchange), the kernel's NCI layer enforces a *second*, independent
exclusivity check beyond the polling lock:

```c
// net/nfc/nci/core.c, nci_start_poll()
if (ndev->target_active_prot) {
    pr_err("there is an active target\n");
    return -EBUSY;
}
```

That's a physical constraint, not a software policy — one antenna, one RF
conversation at a time — and there's no D-Bus trick that fixes it, nor
should there be: interrupting an in-progress smartcard transaction so
`neard` can poll would be wrong. `neard` stays locked out for as long as a
card is actually connected, full stop. The fix below only ever helps during
genuinely idle stretches (no card connected to `ifdnlnfc` at all), by making
sure `ifdnlnfc` doesn't sit on an *unused* poll session between pcscd's
polling-thread cycles.

## What

Don't try to fight for the kernel lock — there's no preemption primitive to
use there (confirmed above). Cooperate one layer up instead, using the
D-Bus API `neard` already exposes for exactly this purpose:
`org.neard.Adapter.StopPollLoop()` / `StartPollLoop()`. These are honored
regardless of which D-Bus peer calls them, because it's always `neard`'s
*own* process making the actual netlink call with its own portid — the
same mechanism `neardevil` itself already relies on
(`src/neardevil/neardevil.cpp`'s `activate()`/`deactivate()`). A third
party asking `neard` to yield just works.

Concretely:

1. When `poll_for_targets()` (`src/ifdnlnfc.c:662`) gets `-EBUSY` back from
   `nl_send_msg()`, make a one-shot D-Bus call to `neard`'s adapter object,
   method `StopPollLoop()`. If `neard` isn't running, isn't owning the
   adapter, or the call fails for any other reason, ignore it — this must
   be a best-effort nudge, never a hard dependency. `ifdnlnfc`/pcscd must
   keep working exactly as today when `neard` isn't installed or enabled.
2. Retry `poll_for_targets()` once after that.
3. Hand the adapter back at the end of every idle `IFDHICCPresence()` cycle
   that didn't find anything — **not** only at channel close. See below for
   why the obvious-looking "yield at teardown" point is actually wrong.

`neard`'s own `ConstantPoll` retry logic (`src/adapter.c`, the 1-second
`dep_timer` retry on `-EBUSY`) means it recovers on its own without needing
anything beyond that per-cycle `StartPollLoop` nudge.

### Why "yield at `IFDHCloseChannel`" doesn't work

The first pass at this plan called `StartPollLoop()` wherever
`stop_poll_for_targets_ex()` (`src/ifdnlnfc.c:705`) does a *real* stop —
which turns out to be only two places: `initialize_adapter()`'s
best-effort cleanup at line 977 (immediately followed by a fresh
`poll_for_targets()` at line 988 — rightly excluded, no yield needed there)
and `IFDHCloseChannel()` at line 1086. The second one is the bug: for a
long-running system service, pcscd opens the IFD channel once when the
reader appears and keeps it open for the reader's entire lifetime —
`IFDHCloseChannel` only fires on driver shutdown or reader removal, not
between taps or transactions.

Meanwhile `poll_active` also gets cleared directly — without going through
`stop_poll_for_targets_ex` at all, and without any yield — at
`src/ifdnlnfc.c:501`, inside the `NFC_EVENT_TARGETS_FOUND` handler. And
`IFDHICCPresence()` (`src/ifdnlnfc.c:1731`) re-arms its own poll on every
subsequent call whenever `!poll_active`. Net effect: once `ifdnlnfc` wins
the lock the first time, it just keeps re-polling itself forever, every
cycle, for as long as pcscd runs — `neard` would be starved *permanently*
after the first FIDO2 tap, not just momentarily. Worse, since `ifdnlnfc`
polls the same `NFC_PROTO_ISO14443_MASK`/`_B_MASK` that many NDEF Type-4
tags also use, it would end up silently swallowing taps meant for the KDE
integration too, with no way back short of restarting pcscd.

### The actual fix: yield at the end of every idle cycle

`IFDHICCPresence()` needs to stop treating "I already have a poll session
open" as a steady state to leave alone, and instead release it every time
a cycle comes up empty.

Note the actual blocking wait-with-timeout lives one level up, in
`IFDHPolling()`'s `poll()` call (`src/ifdnlnfc.c:1169`-ish, on the duplicated
`event_sock`/wake fds) — that's pcscd's dedicated polling thread blocking
between cycles, called *before* `IFDHICCPresence()` on each iteration.
`IFDHICCPresence()`'s own `nl_recvmsgs_default(event_sock)` call
(`src/ifdnlnfc.c:1734`) doesn't block at all: `event_sock` is set
non-blocking at setup (`nl_socket_set_nonblocking()`, `src/ifdnlnfc.c:861`),
so this just drains whatever events are already queued and returns
immediately. The yield point below is about what `IFDHICCPresence()` does
*after* that drain, not about the wait itself.

- When a poll cycle comes up empty — `nl_recvmsgs_default(event_sock)` at
  line 1734 has returned, `card_present` is still false, and
  `IFDHICCPresence()` is about to fall through to
  `result = IFD_ICC_NOT_PRESENT` (`src/ifdnlnfc.c:1741`-`1746`) — explicitly
  `stop_poll_for_targets(&ifdnlnfc_state.adapter)` and then
  `neard_reclaim_adapter(adapter->idx)` **before returning** from
  `IFDHICCPresence()`, rather than leaving `poll_active` set for the next
  call to silently skip re-polling.
- The next `IFDHICCPresence()` call then starts from a clean slate: no
  poll session held, so it goes through the normal
  `poll_for_targets()` → (possibly) `-EBUSY` → `neard_yield_adapter()` →
  retry path again.
- This turns the idle state into a real time-share: between any two
  `IFDHICCPresence()` calls (pcscd's own polling-thread cadence — observed
  on the order of a few hundred ms to ~1s), `neard` gets a window where the
  adapter is actually free, instead of a session `ifdnlnfc` holds open
  indefinitely just in case.
- Once a target *is* found and connected, none of this applies — that's
  the active-target exclusivity from the "Why" section above, which is
  correct to leave alone.

### Caught live: never yield a session in the same call that started it

Implementing and deploying this surfaced a real failure mode the design
above didn't anticipate. The first version stopped *any* poll session
found active at the point `IFDHICCPresence()` falls through to
`IFD_ICC_NOT_PRESENT` — including one that `poll_for_targets()` had just
started a few lines earlier in that *same* call, when `poll_active` was
false on entry. That session never survives to see an `IFDHPolling()`
wait at all: `poll_for_targets()` starts it, the immediately-following
`nl_recvmsgs_default(event_sock)` drains nothing (no time has passed,
`event_sock` is non-blocking), and the fix's own yield logic stops it
again on the spot — a START_POLL followed by a STOP_POLL within the same
function call, microseconds apart.

Reproduced live against this hardware (NXP1001/npc300): starting and then
immediately stopping a poll this way reliably left the adapter wedged —
every subsequent `NFC_CMD_START_POLL`, from *either* `ifdnlnfc` or `neard`,
came back `-EBUSY`, and neither side's own `Polling` state agreed with
being the owner (`neard`'s `StopPollLoop` replied "Not polling" while its
`StartPollLoop` simultaneously got "-EBUSY" from the kernel). This matches
the independent `target_active_prot` exclusivity from the "Why" section,
not the polling-lock one this fix targets: something apparently begins
activating against a poll within single-digit milliseconds of it
starting, and yanking the poll via `STOP_POLL` before that handshake
finishes leaves the chip/driver's target state torn and stuck — recoverable
only by a full reboot, *not* by `nlnfc-init --reset`'s raw NCI power-cycle,
which doesn't touch the generic `nfc` core's software-side
`target_active_prot`/`dev->polling` bookkeeping at all. Confirmed the
inverse too: starting a poll via `neard`'s `StartPollLoop` and leaving it
running for a couple of real seconds before `StopPollLoop` works cleanly,
every time, no wedge.

First attempt: snapshot `poll_active` *before* conditionally calling
`poll_for_targets()`, and only run the stop-and-yield block when that
snapshot was already true. Deployed, and still wedged — because a second
instance of the exact same hazard hides behind an entry that reads
`poll_active == true` for a reason that has nothing to do with having
survived a wait: `initialize_adapter()` (`src/ifdnlnfc.c:988`, at
channel-open) starts a session itself, `open_channel()` succeeds, and the
very *first* `IFDHICCPresence()` call pcscd's polling thread makes
afterward — typically within single-digit milliseconds, before
`IFDHPolling()` has ever run a wait against this session — sees
`poll_active == true` on entry (true, but only because channel-open just
set it moments ago) and stops it anyway. Confirmed live, with debug
logging: `poll_for_targets() NFC target poll started` immediately followed
(~10ms later) by `stop_poll_for_targets_ex() NFC target poll stopped`, and
the adapter wedged exactly as before. A boolean snapshot of "was it
already active" can't distinguish "active because it survived a wait"
from "active because someone just started it a moment ago" — both read
`true`.

Actual fix: track real elapsed time, not a boolean. `struct nfc_adapter`
gained `poll_started_at_ms`, a `CLOCK_MONOTONIC` timestamp
(`src/ifdnlnfc.h`) written by `poll_for_targets()` itself
(`src/ifdnlnfc.c:811`, right alongside the existing `poll_active = 1`) —
the one place that transitions a session to active, regardless of which
caller triggered it. `IFDHICCPresence()`'s stop-and-yield block now checks
`poll_active && monotonic_ms() - poll_started_at_ms >= MIN_POLL_DWELL_MS`
(`MIN_POLL_DWELL_MS` = 500ms) instead of a snapshot. This covers both
instances of the hazard uniformly — started-and-reconsidered within the
same call, and started-by-channel-open-stopped-by-the-first-cycle-after —
with the same mechanism, while still guaranteeing every session gets a
real dwell window before `ifdnlnfc` ever asks to stop it. 500ms was picked
empirically: live testing via `neard`'s own D-Bus `StartPollLoop` showed a
session surviving on the order of a couple of real seconds never
reproduced the wedge, so 500ms leaves comfortable margin while still
handing the adapter back to `neard` promptly during genuine idle
contention.

## How

### New dependency

`ifdnlnfc` currently has zero D-Bus dependency (confirmed:
`configure.ac`/`Makefile.am` only pull in `libnl-3`/`libnl-genl-3` plus
pcsclite). Needs a D-Bus client library — `libdbus-1` is the lighter-weight
option and matches the C/libnl style already used here better than
pulling in `sd-bus`/GDBus; `src/ifdnlnfc.c` doesn't use GLib at all
currently and shouldn't gain that dependency just for this.

Add to `configure.ac`: a `PKG_CHECK_MODULES([DBUS], [dbus-1])` (mirroring
the existing `PKG_CHECK_MODULES` calls for `libnl-3.0`/`libnl-genl-3.0`),
and wire `DBUS_CFLAGS`/`DBUS_LIBS` into `Makefile.am` the same way
`NL_CFLAGS`/`NL_LIBS` are already wired.

### Resolving the adapter object path

Don't hardcode `/org/neard/nfc0`. `ifdnlnfc` already tracks
`adapter->idx` (the same netlink `NFC_ATTR_DEVICE_INDEX` it polls), and
`neard` names adapter objects `/org/neard/nfc<idx>` consistently with that
same kernel device index (confirmed against the live system: `nfc0` in both
`/sys/class/nfc/` and neard's `GetManagedObjects()` output). So the object
path can be built directly from `adapter->idx` without needing to query
`org.freedesktop.DBus.ObjectManager` first — simpler, and avoids a second
round-trip on every contended poll. Build the path lazily (only when
actually about to make a D-Bus call), not eagerly at adapter-discovery
time, since `neard` may not even be running yet when `ifdnlnfc` first
starts.

### Call sites

- New static helper, e.g. `neard_yield_adapter(uint32_t idx)`: opens a
  private system-bus connection (`dbus_bus_get_private` +
  `dbus_connection_set_exit_on_disconnect(conn, FALSE)`, so a `neard`
  hiccup can't take `ifdnlnfc`/pcscd down with it), builds
  `/org/neard/nfc<idx>`, calls `org.neard.Adapter.StopPollLoop` with a
  short timeout (don't block pcscd's polling thread waiting on a D-Bus
  round-trip for long — a few hundred ms timeout, not the libdbus default),
  logs at `PCSC_LOG_DEBUG` on failure (this is expected/normal whenever
  `neard` isn't in the picture), and closes the connection. Mirror with
  `neard_reclaim_adapter(uint32_t idx)` calling `StartPollLoop("")` (empty
  string argument, matching what `neardevil` itself passes).
- In `poll_for_targets()` (`src/ifdnlnfc.c:662`): on `err == -EBUSY` from
  `nl_send_msg()`, call `neard_yield_adapter(adapter->idx)` and retry the
  `nl_send_msg()` once before giving up and returning the error as today.
- In `IFDHICCPresence()` (`src/ifdnlnfc.c:1669`), at the point where the
  current cycle's (non-blocking) `nl_recvmsgs_default(event_sock)` drain at
  line 1734 has completed without a target found — i.e. right before the
  fall-through to `result = IFD_ICC_NOT_PRESENT` around lines 1741-1746 —
  call `stop_poll_for_targets(&ifdnlnfc_state.adapter)` followed by
  `neard_reclaim_adapter(adapter->idx)` before returning — this is the new
  per-cycle yield point, replacing the channel-close-only one from the
  first pass of this plan (see "Why yield at `IFDHCloseChannel` doesn't
  work" above).
- `stop_poll_for_targets_ex()`'s existing call sites
  (`src/ifdnlnfc.c:977` and `:1086`) are unaffected — `977`'s force-reset
  path still shouldn't yield (immediately re-polls itself at line 988), and
  `1086`'s `IFDHCloseChannel` path can keep calling the plain
  `stop_poll_for_targets()` wrapper as today; a `neard_reclaim_adapter()`
  call there too is harmless (best-effort, idempotent) but no longer the
  *only* place it happens.

### Lock scope around the D-Bus calls

Both `IFDHICCPresence()` call sites for this (the `poll_for_targets()`
EBUSY retry, and the new per-cycle `neard_reclaim_adapter()` yield) run
with `state_lock` held for the function's entire body — it's locked once
at entry (`src/ifdnlnfc.c:1678`) and only unlocked at `out`
(`src/ifdnlnfc.c:1749`). That's worth calling out explicitly because this
file already has a documented instance of the opposite choice:
`IFDHPolling()` deliberately drops `state_lock` *before* its own blocking
`poll()` wait, specifically "so other IFDH* entry points are not stalled by
it" (comment at `src/ifdnlnfc.c:1152`-`1154`). A synchronous D-Bus round
trip is the same kind of blocking operation, so the same question applies
here: hold the lock through it, or drop/reacquire around it the way
`IFDHPolling()` does?

Decision: **hold it.** Dropping `state_lock` around the D-Bus call would
mean reacquiring afterward and re-validating everything the lock protects
(`channel_open`, `adapter_removed`, the netlink sockets) before touching
them again, since a concurrent `IFDHCloseChannel()` could have torn all of
that down in the gap — `netlink_cleanup()` frees `event_sock`/`cmd_sock`
and nothing currently in this file re-checks for that mid-function. Adding
that re-validation is real new complexity and a new place to get a race
wrong, in exchange for shrinking a stall that is already bounded and rare:

- `poll_for_targets()` is only on the EBUSY path when `card_present` is
  false, and `IFDHICCPresence()` short-circuits past all of this
  (line 1690) whenever `card_present` is true — so these D-Bus calls can
  never run concurrently with an in-flight `IFDHTransmitToICC()`, which
  requires a present card and holds `state_lock` for its own duration
  (`src/ifdnlnfc.c:1602`-`1653`) precisely while one is connected. No
  FIDO2 transaction can be stalled by this.
- The only caller that could be kept waiting on `state_lock` by a stuck
  D-Bus call is something tearing the channel down concurrently —
  `IFDHCloseChannel()` on reader removal or driver shutdown — and only in
  the unlikely case that `neard` is hung rather than merely absent (the
  short per-call timeout from the "Open questions" section below bounds
  this to roughly that timeout, a few hundred ms, one time, on an
  already-rare path).

`IFDHPolling()`'s case is different enough that its precedent doesn't
transfer directly: its wait is multi-second and *not* needed for
correctness there (nothing in its own body touches protected state across
the wait), so dropping the lock is free. Here the D-Bus wait is short,
genuinely frequent (every idle cycle while `neard` holds the adapter), and
sits in the middle of a call sequence (`poll_for_targets()` → retry →
`nl_recvmsgs_default()` → yield) that does want a consistent view of
adapter/channel state throughout. Keep it simple and correct; the
best-effort framing already in this plan (ignore any D-Bus failure, never
a hard dependency) is what makes the bounded stall acceptable rather than
something that needs engineering around.

### NixOS-side prerequisite: neard's bus policy denies pcscd by default

Even with all of the above implemented correctly, the D-Bus calls would be
rejected before ever reaching `neard`. `neard`'s own shipped
`org.neard.conf` bus policy is:

```xml
<policy user="root">
    <allow send_destination="org.neard"/>
    ...
</policy>
<policy at_console="true">
    <allow send_destination="org.neard"/>
</policy>
<policy context="default">
    <deny send_destination="org.neard"/>
</policy>
```

`pcscd` runs as its own dedicated unprivileged user, not root — confirmed
live (`ps`/`systemctl show` on the actual running service: `User=pcscd`,
non-root UID) — and a system service isn't "at console" either. So without
a policy change, every single call from `ifdnlnfc` would be denied by the
bus itself, independent of anything `ifdnlnfc` does right. This needs its
own `<policy user="pcscd"><allow send_destination="org.neard"/></policy>`
snippet shipped via `services.dbus.packages` on the NixOS side (already
added in the nix-config repo's `overlay/modules/nfc-nlnci.nix`, alongside
the module that wires `ifdnlnfc` into `pcscd` in the first place) — nothing
further needed here in `ifdnlnfc` itself, just noting it so the two sides
of this fix aren't separated without a trace of why both are needed.

Checked `systemd.services.pcscd.serviceConfig`'s other sandboxing
(`ProtectSystem=strict`, `RestrictNamespaces=yes`, `SystemCallFilter=
@system-service` minus `@resources @privileged`, no `RestrictAddressFamilies`
set at all) and none of it blocks an `AF_UNIX` D-Bus client connection —
`ProtectSystem=strict` only blocks *writes* outside allowlisted paths, not
`connect()` to an existing socket. The bus policy above is the only actual
blocker.

### What this deliberately does not do

- No persistent D-Bus connection/service watcher — `ifdnlnfc` stays a
  passive, on-demand D-Bus *client* only, opened and closed around single
  method calls. Keeping a long-lived connection would mean tracking
  `neard` coming and going, reconnect logic, etc., for no benefit here:
  these calls are rare (only on actual contention) and best-effort.
- No change to polling scope/protocol masks — `ifdnlnfc` keeps polling
  `NFC_PROTO_ISO14443_MASK | NFC_PROTO_ISO14443_B_MASK`
  (`src/ifdnlnfc.c:667-668`) exactly as today. The overlap with NDEF
  Type-4/ISO-DEP tags is real but irrelevant to this fix: the coordination
  is about *who currently holds the poll session*, not about carving up
  which protocols belong to whom.
- No changes to `neard`, `neardevil`, or `nlnfc-init` — confirmed
  `nlnfc-init` has no polling/netlink involvement at all (it's a one-shot
  chip-priming tool, unrelated to this), and the fix is fully localized to
  `ifdnlnfc`.

## Open questions to settle while implementing

- Exact D-Bus timeout value for the `StopPollLoop`/`StartPollLoop` calls —
  needs to be short enough that a hung/unresponsive `neard` can't stall
  pcscd's polling thread noticeably, but long enough to not spuriously
  time out under normal load. Start conservative (e.g. 200-500ms) and
  adjust based on observed behavior.
- Whether `neard_yield_adapter`/`neard_reclaim_adapter` should cache the
  private D-Bus connection across calls (reconnecting only on failure)
  rather than opening/closing one per call, if this turns out to add
  noticeable latency to `IFDHICCPresence()`'s existing polling cadence.
- Logging verbosity: these D-Bus calls will be genuinely frequent on a
  machine where `neard` is holding the adapter most of the time (expected,
  given `ConstantPoll`), so failure logging needs to stay at a level that
  doesn't flood `pcscd`'s log under normal operation.
