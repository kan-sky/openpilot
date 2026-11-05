# Comma resources and Navdy Bluetooth, 2026-10-19

## Scope and evidence

Read-only investigation of the user's October 18 routes on the Traverse's comma
4 (`mici`), WIP-FR source HEAD `5378d124cb836b654611c6c4e21094b61e187774`.
The recorded checkout was dirty; HEAD is not a complete installed-file identity.
No driving software, DM policy, navigation selection or production APK was
changed. Bluetooth tests use an independent synthetic receiver, not the HUD.

All 390 qlogs from seven October 18 routes were parsed on the device at nice 19
while offroad. Compressed extracted metrics and source SHA256 manifests were
retrieved and all 390 hashes matched the independently collected inventory.
There were zero parse failures. The records span about 6 h 27 min, including
parked time, not 6 h 27 min of continuous movement.

The original bulk transfer was stopped because of network retransmissions and
slow throughput. Twelve complete original qlogs were retained locally, including
the long-route warning segment and a later steady segment. The 390 original
qlogs and camera videos were NOT all downloaded. Nine selected full-rate rlogs
were additionally analyzed on-device; their hashes and extracted results were
retrieved. None of the original device logs were modified or deleted.

Private evidence and scripts: `~/Downloads/Comma-Resources-20261019/`.
Do not publish raw routes, device identifiers, phone identities or pairing keys.

## Long routes

Times use recorded UTC wall time converted to Korea time. Route counters below
omit the private route identifier suffixes.

| Metric | Route 168, 15:33:18-18:48:56 | Route 169, 19:00:42-21:07:42 |
| --- | ---: | ---: |
| Duration, including parked intervals | 3 h 15 min 38 s | 2 h 7 min |
| Eight-core mean CPU utilization | 59.1% | 58.8% |
| CPU utilization p95 / maximum sample | 67.1% / 85.6% | 64.8% / 92.3% |
| Memory utilization mean / maximum | 76.0% / 80% | 75.6% / 80% |
| Minimum recorded available memory | 736.8 MiB | 725.2 MiB |
| CPU sensor maximum | 60.7 C | 61.1 C |
| Model inference mean after first minute | 24.65 ms | 24.68 ms |
| Model inference p95 / maximum after first minute | 29.54 / 35.92 ms | 29.75 / 37.60 ms |
| Navdy bridge CPU, one-core scale | 7.09% | 7.91% |
| ADB CPU, one-core scale | 0.68% | 0.70% |
| Navdy bridge RSS, entire route | 62.645 MiB, unchanged | 62.645 MiB, unchanged |
| Navdy bridge PSS range | 40.00-41.81 MiB | 40.01-41.57 MiB |

All seven routes' device thermal status samples were green. No low-memory,
overheat or model-lag event was found in the qlogs. The final short evening route
reached 81% memory use and 686.1 MiB available; absence of a warning is not proof
that all future memory workloads fit. Navdy RSS was stable and PSS did not grow;
this does not establish that every system allocation is leak-free.

CPU percentages for individual processes use one fully occupied core as 100%.
Eight-core system averages use all eight cores as 100%; they must not be compared
without conversion. Core 6 averaged about 86% in both long routes, so the total
average is not evidence of identical spare capacity on every core. Selected
full-rate samples put the signal-color observer around 45-47% of one core,
locationd around 35-37%, and card around 34-35%, larger than the Navdy bridge.

The qlog model stream is decimated to 2 Hz. It reported zero frameDropPerc and
roughly 500-ms sample spacing, but that cannot prove every 20-Hz frame arrived.
Nine selected rlogs contained 10,738 modelV2 frames. The eight non-startup
segments had valid consecutive frame IDs and rates 19.9976-20.0016 Hz; no strict
whole-route minimum-20-Hz claim is made. The startup segment contained one frame
ID discontinuity and a 111.5-ms publication interval. UI draw work averaged
20.6/24.3 ms on the long routes, with isolated maxima 67.4/97.1 ms.

## Findings requiring separation from resource exhaustion

- At 16:47:16, route 168 segment 73 recorded `commIssue/softDisable` for about
  2.20 seconds at approximately 96 km/h. This is a real warning, not an entirely
  clean drive. Concurrent radarState messages became invalid with all decoded
  radar hardware error flags false. The source also generates commIssue for
  aggregate radar-input invalidity. The model's 1,200 frames in that segment
  remained valid/consecutive with a 63.3-ms maximum publication interval.
  Full-rate inspection finds 20 invalid radarState records, but every logged
  carState, modelV2, livePose and liveTracks record in that minute is valid.
  liveTracks publishes 766 records in the minute, with a maximum interval of
  133.8 ms; there is no multi-second publication outage. Receiver frequency/
  liveness checks can fail despite individual valid packets; the exact failing
  SubMaster check is not recorded here and remains unproven. Nearby radard work
  maxima remain below 10 ms. Logger ordering is not the radar subscriber's exact
  reception order and is not used to assert its per-frame input age.
  Do not attribute this warning to memory, Bluetooth or model inference merely
  because it appears during a resource audit.
- At 19:00:49, route 169's startup recorded controlsMismatch while speed was zero
  and gear was Park. No corresponding displayed alert was found in that sampled
  startup segment. This is distinct from the moving radar warning.
- DM became explicitly disabled at 16:33:50 and 19:06:02. Manager then changed
  dmonitoringmodeld's shouldBeRunning to false and recorded a normal exit code 0.
  It was not an unexplained crash. Much of both long routes consequently did not
  run the DM inference model. Those intervals are not an all-features-on stress
  test, and the apparent memory reduction must not be credited to an optimization.
- Short routes had camera synchronization errors during startup, and long routes
  included Athena websocket errors. No direct causal link to the Navdy bridge
  or the moving warning has been established.

## Live Bluetooth resource trial

The original USB bridge remains connected throughout the measurements. A second
process sends synthetic full snapshots to the independent, previously paired
Navdy RFCOMM test server at 5 Hz. It compares raw JSON and zlib level 1. Thus the
test measures additional traffic and daemon costs, NOT the exact net cost of
replacing USB inside the real bridge. No live navigation, warning or vehicle
command is sent through the benchmark.

Resource samples use /proc CPU counters, RSS, online CPUs and MemAvailable.
The first ten seconds of each phase are excluded from steady resource summaries.
Traffic CPU and maximum RSS use waited-child rusage; CPU includes startup.
Log analysis and bulk transfer do not run concurrently with resource phases.
The monitor's command line contains its child command, so the initial substring
classifier also labeled the monitor as snapshot.py. Those ambiguous per-process
samples are excluded, not counted as traffic CPU. smaps_rollup reads failed, so
this live trial has no usable PSS observations. Historical route PSS above comes
from logged procLog data and is separate. Sample-start-relative USB age is also
discarded because the status file is read later; the final direct read verifies
freshness instead.

The first trial failed before any benchmark payload: after SDP and a 30-second
idle phase, RFCOMM connect returned `Host is down`. Cleanup restored Navdy's
original scan mode and comma's original radio-off policy; USB kept working.
This failure is retained, not discarded as a successful throughput test.

A second trial rechecks the already-authorized peer's connectable state before
each phase and begins traffic promptly after SDP. This changes connection timing;
even success does not establish the original failure's cause or solve production
automatic reconnection.

### Completed measurements

| Encoding and run | Traffic duration | Verified / produced | Traffic CPU, one core | Child peak RSS | RTT median / p95 / maximum |
| --- | ---: | ---: | ---: | ---: | ---: |
| Raw A | 60 s | 299 / 300 | 2.17% | 10.59 MiB | 101.7 / 233.6 / 386.3 ms |
| zlib A | 90 s | 450 / 450 | 2.56% | 10.74 MiB | 46.6 / 118.4 / 173.7 ms |
| Raw B | 60 s | 300 / 300 | 2.25% | 10.58 MiB | 78.8 / 187.4 / 327.6 ms |
| zlib B | 90 s | 450 / 450 | 2.62% | 10.75 MiB | 44.9 / 126.7 / 177.6 ms |

Both compressed runs sustained approximately 5 Hz, with all 900 snapshots verified
and applied fresh, zero rejected payloads and zero freshness-expiry clears. Raw
A coalesced one unsent snapshot; raw A/B had 7/3 receiver freshness-expiry clears.
Thus successful raw byte delivery alone does not establish smooth state display.
Compression reduced the approximately 2,318-byte JSON to 1,027 bytes before
framing, with a modest measured increase in CPU time. RTT is a complete echo
round trip, not one-way HUD rendering latency.

bluetoothd RSS was 3.402 MiB and btattach RSS 0.398 MiB throughout traffic phases.
Their sampled steady CPU increments were zero at this clock-tick resolution;
this is not a claim of zero startup, kernel, interrupt or radio cost. Adding those
RSS values to the child peak gives about 14.5 MiB of process RSS, NOT unique PSS
or an exact system-memory delta. A production bridge may share its interpreter
and replace ADB work rather than add an independent process.

System mean busy time was 22.8% before traffic, 24.0-24.6% during traffic and 23.7%
with the radio idle. The final radio-off USB baseline rose to 55.1%, even though
the benchmark had exited. Only CPUs 0-3 were online in these offroad tests.
Consequently, coarse before/after system CPU cannot isolate Bluetooth cost from
background work and the observer itself. Do not extrapolate it to the eight-core
onroad workload or claim an exact whole-device percentage-point increase.

All 465 resource samples in the second campaign reported the existing bridge
connected over USB, and its display sequence advanced in every phase. This is a
short offroad experiment alongside USB, not an onroad migration or a test of all
real TBT, alerts, commands, phone audio and ambient-light operations together.

### Restoration

Fresh verification after the campaign confirmed the same boot and source HEAD,
offroad and not engaged. The production USB bridge retained its original PID,
and its sequence advanced by 15 over three seconds, with about 20 ms status age.
Navdy's main app and connection service retained their original PIDs. The phone
was still connected; no claim is made about an unobserved ambient BLE session.

The temporary test server was gone, Navdy scan mode was restored to its original
20, and existing bonds were preserved. Both comma Bluetooth services were
inactive, no HCI device or persistent enable flag remained, and the dedicated
test DEX and temporary resource-runner scripts were removed. No reboot, APK
replacement, DM change, vehicle-control change or Git publication was performed.

### Assessment

The inspected routes do not show resource exhaustion attributable to Navdy, and
the measured compressed Bluetooth snapshot process is small compared with the
existing onroad workload. Resource cost alone does not rule out migration. The
remaining practical blockers are automatic reconnection, expiry/clearing and
real-message integration under concurrent onroad load, not raw bandwidth.
Neither these tests nor average CPU headroom certify driving-system safety.

## Evidence integrity

Private local artifacts, with SHA256:

- `hl-resource-yesterday.json.gz`: `26510560da692e7a17ab57d3cba6c39942b6341051c94547ace9ec4764ad5871`
- `hl-resource-details.json.gz`: `e1fd231160035dae2ac65495c58a6b3490933a9fca85b44323728745a8f55412`
- `hl-navdy-resource-results-r1.json.gz` (failed connection and restoration): `8087c80b6bd9e9697400db13a1a63dd51e560d950adc0f20189d3fa9d191d93b`
- `hl-navdy-resource-results-r2.json.gz`: `af369d1e67125345e3b4a05cdeca6ac8d486cf802eb058da5edcaf41a10d2107`

The warning detail extract and final-state JSON are retained in the same private
directory. These contain operational identifiers and must not be published as
public attachments.
