# Automatic Navdy Diagnostics

This is a recording change, not a Bluetooth dropout fix. The user requested
automatic logs on both devices while keeping Bluetooth and phone reconnection.

## Comma

`navdy_bridge` starts one daemon journal writer automatically. Files are outside
the route-upload tree at `/data/navdy/diagnostics/events.jsonl`, with three older
generations `events.1.jsonl` through `events.3.jsonl`. Each is bounded to 4 MiB
(16 MiB total); directory mode is 0700 and file mode is 0600.

Only allowlisted metadata is persisted: boot/run identity, wall and monotonic
time, transport lifecycle, source availability, receiver session/sequence,
display-power/ambient state, bounded phase timing summaries, and logging loss
counters. Existing cloudlog event cadence is preserved; new five-second
summaries stay local. Raw payloads, routes, song data, peer addresses and keys
are not copied into the journal.

The bounded 256-record queue never waits for the filesystem. Records are limited
to 4 KiB with an 8/s rate and 16-record burst. Storage retries are 30 seconds
apart and stop below 64 MiB or 5% available space. Shutdown waits at most 250 ms;
a kernel-blocked writer can outlive that wait. Logging is best-effort: queue
overflow, disk failure and abrupt power removal can lose recent records.
Kernel-level filesystem stalls may also delay a subsequent process restart.
Storage failure discards queued and incoming records during the 30-second
cooldown; counters are best-effort. `diagnostics.journal` in
`/dev/shm/navdy-status.json` exposes writer errors, drops and disabled state
even when the disk journal itself cannot be written. This status is not durable.

A representative full 1,394-byte heartbeat gives about 16.72 hours of quiet-link
retention at five-second intervals. This is an estimate, not a guaranteed
retention period: connection events reduce it. Collect logs soon after an issue.
Repeated connection failures while parked can overwrite an evening incident
before morning. At the artificial maximum of 8 full 4-KiB records per second,
the ring holds only about 8.5 minutes. No minimum retention is promised.
The production directory must be owned by the bridge user and contain no
symlink path components; confirm actual growth and ownership after deployment.

## Navdy

The companion v169 APK adds a bounded phone-state journal. It retains the v168
socket cancellation fix and the existing main-process wireless/ambient logs.
The new phone journal uses a separate app-private 4 x 1 MiB ring, ten-second
heartbeat, and an invocation-only exception guard around the new hook. It does
not alter phone search, reconnect, pairing, freshness or wire protocol.

The Navdy base-state hook runs after subclass entry work. Treat its state as the
last observed base entry, not proof of a live phone connection. Compare monotonic
times within a Navdy boot; align Comma observations using existing ACK session
and sequence metadata. Wall clocks may jump. USB is only needed to retrieve the
Navdy files later, not for automatic recording.

## Source Verification

Focused diagnostics/bridge/reconnect tests: 145 passed. Navdy suite: 716 passed,
five DBC-dependent tests excluded on this Mac because native `params_pyx` is
unavailable. Lint and whitespace checks pass. No control, model, Panda, transport
or reconnect-policy source changes are included.

Phone-connected navigation/reconnection and road endurance cannot be validated
while the phone is unavailable.

## Device Verification

Code through `47442dbe` was reviewed against `7e2739ea`, published to WIP-FR,
then fast-forwarded on the parked, ignition-OFF, inactive Comma. Existing generated
DBC changes were preserved. Only `navdy_bridge` was restarted; no manager,
model, control, Panda or device reboot was performed. Native full pytest could
not run because the device venv lacks pytest; no global packages were installed.

The actual journal is owned by uid 1000 with mode 0600 under a 0700 directory.
It records Bluetooth-only sessions, ACK sequences and source/ambient metadata.
After a second targeted bridge restart, the first 124,041 bytes retained the
same SHA256 and a new run identity appended. Storage errors and drops were zero.

The Navdy companion is v169 candidate4, APK SHA256
`d1268a7bebbecd34e4ca48f1abac2518138f5048ab14d912c9eda53758c26651`.
Candidate3 was not installed: actual API21 ART exposed same-process overlapping
FileLock behavior absent from the desktop JVM. Candidate4 adds writer-only
in-process ownership before opening the OS lock. Actual candidate APK ART passed
40 checks; host runtime passed 40 checks and tooling passed 49 tests.
Existing classes, phone registration and Bluetooth configuration were preserved.

Navdy's actual entry hook created an app-owned 0600 phone journal and 0700
directory automatically. Force-stop/relaunch preserved the prior 3,829-byte
prefix exactly and appended a new run. IDLE/LISTENING/SEARCHING and ten-second
heartbeats were observed with zero storage errors/drops. Existing wireless and
ambient journals remain app-owned and separate. Navdy had about 200 MiB free.

Known limits: prior Bluetooth resets also occur during this diagnostic task;
recording them is not a dropout fix. Cross-process lock exclusion and abrupt
power-loss durability were not physically tested. Collect both sides promptly
after an incident; Navdy retrieval later requires USB, not phone navigation.

A final 91.46-second observation without a continuous logcat collector added
35 Comma records and 15 Navdy phone records (nine heartbeats). Both writers had
zero drops/errors and Navdy's previous prefix remained unchanged. All 180 status
samples used Bluetooth-only; 155 were connected, sessions were 3/4/5, and USB
updates stayed zero. Three transport failures, including a connection-attempt
timeout, were recorded. The last sample was connected. The current phone-service
PID had no fatal/verifier error in the bounded logcat snapshot. This confirms
automatic evidence capture, not stable radio transport or phone-connected driving.
