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
3. When `stop_poll_for_targets_ex()` (`src/ifdnlnfc.c:705`) actually stops
   our own poll/transaction (i.e. the existing teardown path, called from
   line 1086 and the reset path at line 977), make a second D-Bus call to
   `StartPollLoop()` on the same adapter object, handing the adapter back
   to `neard` for NDEF-tag duty. Again best-effort, ignore failures.

This gives the right priority for free: `ifdnlnfc` actively preempts
`neard` exactly when it has real work to do, and hands the adapter back the
moment it's done. `neard`'s own `ConstantPoll` retry logic
(`src/adapter.c`, the 1-second `dep_timer` retry on `-EBUSY`) means it
recovers on its own without needing to be told anything beyond the
`StartPollLoop` nudge.

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
- In `stop_poll_for_targets_ex()` (`src/ifdnlnfc.c:705`), after a
  successful stop (the `else` branch at the end that sets
  `adapter->poll_active = 0`), call `neard_reclaim_adapter(adapter->idx)`.
  Only do this for the *real* stop (teardown/transaction-complete path),
  not for the `force` path used when resetting after a bad state
  (`src/ifdnlnfc.c:977`) where the adapter is about to be re-polled by
  `ifdnlnfc` itself immediately after (`src/ifdnlnfc.c:988`) — handing back
  control there would just cause an immediate, pointless second contention
  cycle.

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
