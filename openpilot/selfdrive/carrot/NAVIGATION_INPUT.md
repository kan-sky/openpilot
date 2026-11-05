# Navigation input selection

Carrot Web: Settings -> Speed & Decel -> Navigation Input.
`CarrotNaviSource` is a persistent integer: `0` (default) retains network
navigation, `1` selects Navdy USB, directly or through a paired Jetson. The navigation process reads this once
at startup. Restart comma after changing it; saving does not switch a running
drive or carry the old source's guidance into the new session.

## Transport and mapping

`navdy_bridge` reads published vehicle messages and sends display-only
`navdy.openpilot.v1` JSON at 5 Hz. It never publishes CAN or control commands.
It uses a separate ADB server on 5038, identifies the installed HUD package,
allocates its own forward to Navdy port 8765, and releases only that forward.
It does not reset USB roles, Bluetooth, other ADB servers or pairing.

Direct USB is attempted first. If unavailable, an already paired Jetson may run
`/opt/carrot-jetlink/venv/bin/python` with
`/opt/carrot-jetlink/current/openpilot/selfdrive/navdy/transport.py --stdio` over
the existing pinned-key SSH management connection. The Jetson bundle includes
this relay and requires ADB. No authentication is disabled.

HUD v157 adds `navigationFeedback` to the existing camera/road-preview ACK.
The comma reader maps camera limits, road limits, TBT, remaining distance/time,
speed bumps and traffic signals into the latest `CarrotNaviControl` adapter.
Existing Carrot maneuver mapping and speed-control settings remain authoritative.
No route polyline, intersection image or vehicle CAN navigation data is invented.
An old APK can still provide its existing camera and road-limit fields, but not
TBT/signals. Source selection does not silently fall back to network navigation.

Network mode retains UDP 7706, KISA, TCP 7712, HTTP 7713, the legacy route socket
7709 and navd fallback. Navdy mode rejects their navigation inputs, clears old
traffic-light/intersection-image state at startup, and publishes an empty route.
Phone GPS, clock synchronization, non-navigation commands, discovery and the
separate phone-media service remain available. The phone's `DETECT` command is
navigation input and is excluded. OEM carState speed limits and model-based
curve detection are not changed by this setting.

The navigation HTTP `/health` response distinguishes `navigationSource` (active)
from `configuredNavigationSource` (saved), with `navigationSourceRestartRequired`.
It reports `navdyRelayImplemented: true` and a `navdy` object with transport
freshness and extended-feedback availability. An ignored network POST
returns `accepted: false`, even though HTTP receipt succeeded. A payload cannot
override the selected source by including a `source` field.

## Freshness and diagnostics

The bridge atomically writes `/dev/shm/navdy-feedback.json` and
`/dev/shm/navdy-status.json`. ACKs are limited to 32 KiB. Link silence expires
after 2 seconds; road/guidance envelopes expire after 4 seconds of monotonic
source age. Repeated or out-of-order source sequences cannot renew guidance.
Null, inactive, expired or disconnected guidance clears the corresponding
Carrot state, including traffic signals. Repeated unchanged speed data does not
reset the local ego-motion distance estimate.

The revised v157 also exports integer `validForMs`, the remaining lifetime of
the shortest guidance/signal deadline. The receiver never extends this deadline
for repeated sequences. Signal display requires this field; a first-build v157
without it can provide TBT but not timed signals. Producer restarts reset the
distance history, while ordinary component expiry does not rewind a hazard.
Identical phone payloads do not renew freshness because a cached retransmission
has no distinct generation identifier. Unchanged guidance can therefore expire;
the bridge does not treat its polling heartbeat as new phone information.

Status reasons distinguish missing USB/paired relay, old HUD APK and stale or
absent phone navigation. Read these before changing pairing or reinstalling.

## Validation boundary

2026-10-16: standalone comma C4 plus direct USB Navdy successfully received real
phone road/camera data, left/right TBT and remaining route data with HUD v157.
The new Python source was tested from an isolated directory against installed
native IPC; the live openpilot checkout and OS were not replaced by that test.
Jetson relay hardware, TURZX output and real-vehicle control remain separate
validation requirements. A passing parser test is not a road-safety claim.

Local regressions: `pytest --noconftest -o addopts='' openpilot/selfdrive/navdy/tests openpilot/selfdrive/carrot/tests/test_navigation_source.py`.
Tests isolate native Params/IPC and perform no CAN transmission, clock changes
or device deployment. Full installation/OS compatibility and actual driving
must be validated separately.
