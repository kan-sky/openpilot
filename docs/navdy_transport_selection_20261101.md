# Navdy wired / wireless selection

`NavdyConnectionMode` is a persistent integer, shown beside Navigation Input in
Carrot Web's Speed & Decel / Speed Camera settings.

| Value | Policy |
| --- | --- |
| 0 | Preserve the existing Bluetooth configuration and USB-first fallback policy. |
| 1 | USB only, directly or through an already paired Jetson relay. Never fall back to Comma Bluetooth. |
| 2 | Comma Bluetooth only. USB remains available for diagnostics. |

This chooses the Comma-to-Navdy link, not the phone-to-Navdy connection. Phone
Bluetooth, phone automatic reconnection, ambient BLE, pairing identities, and
`CarrotNaviSource` are unchanged. A missing cable in mode 1 is a disconnected
state, not permission to switch back to Bluetooth. The Jetson relay can use its
existing SSH management network; its final connection to Navdy is USB.

The bridge checks the setting every five seconds, plus any already-running
bounded connection attempt. A policy change closes the old transport, clears
navigation/model cache data and resets retry backoff. Change modes while parked:
the handover briefly interrupts navigation input. Invalid stored values fail
closed instead of silently permitting a different transport. The API rejects
invalid values before numeric clamping; ordinary trailing whitespace in the
backing integer file is accepted.

## Deployment requirement

Rebuild native `openpilot/common/params_pyx.so` and restart the manager before
enabling the setting on an old installation. `Params::clearAll` removes keys
unknown to its loaded native table, including at ignition transitions. Merely
writing a raw parameter file or updating Python is insufficient. Verify the
native key type and retention across manager/onroad/offroad clears in an isolated
Params directory, then confirm the real setting after a fresh manager/reboot.

## Verification scope

- Policy and bridge simulations cover explicit USB/BT, legacy behavior, missing
  cable/peer, Jetson fallback, bidirectional changes, retry during backoff, stale
  feedback clearing, corrupt settings and unchanged Bluetooth configuration.
- Actual catalog/API tests cover registration, selection labels, save/reload,
  preservation of Navigation Input and rejected malformed writes. Shared web
  control/dialog tests exercise selection and cancellation.
- The selector works with v175's existing USB service on port 8765 and common
  navigation/SECTION/model feedback paths. Detailed validation additionally
  found that v175 did not expire the displayed USB state after loss of input.
  The paired v176 `WirelessOwner` patch rejects callbacks aged 500 ms or more
  and expires the display at receipt + 500 ms, using existing main-thread timers.
  It preserves phone reconnect, Bluetooth deadlines, navigation TTL and renderers.
- A USB reply is not proof of completed display rendering. The existing USB path
  posts updates to the HUD main thread before returning its reply. Its display
  queue-age behavior is not identical to Bluetooth's lease validation.
- No steering/braking calculation, navigation TTL, phone reconnection or Bluetooth
  deadline is changed. Offroad transport tests do not prove live route reception,
  panel appearance, physical unplug behavior or long-duration driving stability.

## Installed selector verification

Comma commit `5b907e50902bd959f7f8444ab3e16e55c6d3ab31` was installed by an
exact fast-forward after fresh offroad/ignition-OFF/inactive checks, native
Params rebuild and a normal reboot. The installed checkout passed 1,388 isolated
native tests. The persistent key survived all four transition-clear flags in a
temporary Params store. Real API malformed writes were rejected; browser selection
saved mode 1. Before the paired APK update, 296/296 samples over 60 seconds showed
the same connected USB session with feedback no older than 202 ms.

Explicit Bluetooth mode was also exercised with USB still available for logs.
Its existing intermittent connection losses recurred; this option does not claim
to repair the radio problem.

## Paired receiver and live USB validation

HUD v176 was installed with the original certificate and without resetting app
data or pairings. Its signed SHA256 is
`efacae7d013811c1904400126206c820c71f97866b547ad36d76ccda0799da94`.
Only `WirelessOwner` changes; the phone, USB service, renderer and navigation
code are preserved. Actual Navdy API22/ART isolated tests reproduced four
baseline failures and passed four candidate contracts plus 41,738 pure checks.
Host tests passed 41,764 checks, 26 deployment checks and four runner checks.
Claude's read-only review was separate from these native execution results.

In actual USB-only mode, 300 seconds yielded 1,469/1,469 fresh connected states
and fresh bidirectional feedback, one continuous connection and maximum feedback
age 218 ms. Both Navdy process IDs remained unchanged. Receiver logs had 62 fresh
USB rows, successful applications increasing from 114 to 1,649, no rejected USB
callbacks, no clear failures and no additional display clears during the run.

A guarded restart of only `navdy_bridge` expired stale display ownership once
and recovered the USB connection, with other Comma processes unchanged. A normal
whole-Comma reboot then retained mode 1, commit/native registration and pairing
fingerprints, and reconnected over USB without user action. No force-onroad,
synthetic live vehicle input or control command was used. A further 60-second
post-reboot run passed 298/298 connected USB and fresh-feedback samples.

The APK install succeeded, but its separate Bluetooth observation failed with
`stale_status`; that failed result is retained. This task does not fix or claim
to diagnose the remaining radio instability. Live phone route guidance, physical
unplug recovery, optical panel output and real driving remain unverified. The
Jetson relay is policy/unit tested, not physically connected in this run.
