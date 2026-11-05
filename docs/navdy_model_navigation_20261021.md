# Navdy Model Navigation

## Scope

Extend WIP-FR `06ee2975dfe2c794f81ebc78ff954e9a5071034d` with a separate,
bounded phone-to-HUD-to-comma model-navigation channel. Legacy display TBT,
OBD speed, ambient, USB-first/Bluetooth policy and vehicle control are unchanged.
The exact protocol is in `tools/navdy/NOO_PROTOCOL.md`.

Phone native guidance observations retain their original sequence and timestamp.
Whole route geometry is offered on generation/reconnection and re-offered on
fresh native observations with bounded backoff when verified receiver-cache
evidence is absent. A socket write is not a HUD acceptance acknowledgement.
Matching receiver cache receipts suppress redundant full transfers, not guidance.
Geometry is checked by
SHA-256, point count and coordinate bounds, then cached separately from the small
live feedback. The comma ACK carries at most one 1024-character chunk. Partial,
stale, cancelled, disconnected or mismatched generations cannot enable N16 input.
The complete exchange time is charged to both guidance and independent GPS age.
No heartbeat or retransmission creates a new observation. Re-offers retain the
original sequence/time. Legacy peers without cache receipts retain the bounded
30-second maximum backoff plus observation cadence and transfer time.

Carrot's selected Navdy source publishes only verified route geometry paired
with canonical native maneuvers. Legacy Navdy numeric display codes are not
relabelled as model maneuvers and Mapbox does not silently replace this source.
Optional phone GPS requires its own original fix time and heading; otherwise
the existing verified comma GPS sources remain in use.

## APK Boundary

The accompanying phone/HUD changes are maintained in the separate local Navdy
project, not installed by Git Pull. They require same-signer APK updates without
uninstalling or clearing data. Existing installed APKs were pulled and hashed
before patching. No keys, private route captures or proprietary APKs belong here.

The initial phone producer supports verified Naver native guidance. TMAP's
existing 400-metre preview is not a proven full route and is not accepted as
such. Unsupported native maneuvers and oversized routes fail closed. These
limitations must not be reported as complete support for every navigation app.

Native phone diagnostics identified the user's active approximately 406-km
route as 5627 points, `RouteGuiding`, `VALID`, `RouteOn`, and `Right`. The initial
4096-point gate rejected it. Navdy's complete-route limit is now 8192 points;
the 65536 encoded-byte/64-chunk/32-KiB ACK bounds are unchanged. No route points
are truncated, decimated or replaced by destination chords. Other navigation
sources retain their previous 4096-point bound.

## Verification

- Comma isolated candidate, fresh ignition-OFF/noOutput guard: 1217 tests and
  146 subtests passed in 70.94 seconds after the reader/freshness and whole-route
  capacity corrections.
  The suite includes native map rasterization, N16 input packing, Navdy,
  Jetlink faults, GM manual hold/display speed, NNFF and vehicle tuning.
- The new tracker/reader tests cover duplicate source age, independent position
  age before initial assembly, cross-source replay, sidecar deletion and the
  race where new geometry arrives immediately after old input expires.
- Phone host tests: 88 passed. HUD host tests: 22 cases / 845 assertions passed.
  These are distinct suites and not an aggregate real-device result.
- Native APK preservation checks cover all unrelated DEX classes, resources,
  signatures and API-21 compatibility for new HUD helpers.

Actual route transport and installed APK checks are recorded separately after
deployment. Jetson is disconnected/off at the user's request: no native Jetson
inference, sustained driving-model timing or road validation is claimed here.
No forced-onroad, CAN actuation or Panda firmware change is part of these tests.

## Independent Review And Reader Recovery

Claude Code performed two independent read-only source reviews. The first found
that reader-only expiration could permanently reject a still-valid bridge
sidecar, and that sampling time before a lock or file read could make fresh
feedback appear to come from the future. The second confirmed those fixes and
found a remaining 100-ms polling-gate gap. Its exact pending-frame sequence
failed on the isolated comma candidate before correction and passed afterward.
Claude did not execute the tests; native test results are a separate check.

Each complete bridge assembly now has a local random generation token, written
to the sidecar before its corresponding outer feedback. Readers import geometry
only when the tokens match. Bridge clear/reassembly rotates the token even if
route identity and geometry are unchanged. A reader-only timeout can recover
the unchanged bridge cache with a new, fresh source sequence. Tokens do not
renew observations or bypass sequence high-water marks, cancellation or the
two-second deadline. They are local consistency markers, not authentication.

Readers take the clock inside the lock and after I/O, read waiting feedback
before expiration, and bypass the polling interval once when an existing value
would expire. Unchanged files and replayed source sequences still expire.
Specific rejection reasons survive subsequent status/snapshot calls.

The first complete post-review native run passed 1228 tests and 146 subtests in
70.36 seconds. The subsequent polling-gap correction passed all 59 focused
model-navigation tests. The final native regression, including the actual
`bridge.run()` sidecar-before-feedback/clear/reassembly regression, passed
1231 tests and 146 subtests in 70.05 seconds. These are isolated candidate tests,
not an inference or driving validation.

A third Claude review found that another freshness check could clear the value
between clock samples, disabling the value-dependent polling bypass. The gate
now uses the deadline crossing since the last poll, which survives that clear
and permits at most one extra poll. Its exact reproducer failed before the fix;
60 focused tests and the final native 1232-test / 146-subtest regression passed
(76.12 seconds). An independent reviewer also checked 32 reader/cache scenarios.

Same-signer installed APK hashes were verified on the actual phone (bridge and
Naver base APKs plus both original splits). The actual Navdy ART helper suite
passed 22 cases / 912 assertions using `app_process`; plain `dalvikvm` lacked
the SystemClock JNI initialization and is not counted as a passing test.
These helper tests do not establish delivery through the running HUD app.

At this checkpoint the phone reports fresh native guidance and route writes,
but the actual Bluetooth HUD response still contains `modelNavigation: null`.
The transport is connected. End-to-end route delivery is therefore not yet
validated and must not be described as working based on socket-write counters.

## Live Receiver Diagnosis

An independently built, same-signer diagnostic HUD APK was installed and its
complete installed bytes matched SHA-256
`112ffd3e4a9485eabadc1bfa3e6d9fbc09af967c15738cc661df2b3e639ee994`.
Only its helper DEX differs from the preceding candidate outside signature
metadata. Diagnostics are globally limited to one numeric/reason-only log line
per five seconds; no route, location, producer identifier or title is logged.

The running app confirmed `guidance_future`, with original phone observations
810 and 829 ms ahead of the HUD receipt wall clock. At one sample, 53 model
observations had reached the store, 10 had been accepted and 43 rejected.
Legacy navigation remained fresh while the model component was null. This
establishes timestamp rejection, not absent delivery or a rendering defect.

The existing phone time-sync opcode carries full milliseconds, not rounded
seconds. It captures wall time before the shared write lock; the HUD later
applies the received timestamp through a main-thread event queue without delay
compensation. These are source-confirmed delay opportunities; the exact split
of the measured offset between sender/transport/receiver is not yet measured.
The 100-ms future gate and two-second TTL remain unchanged. A sender clock
sync correction is being tested; no working end-to-end result is claimed here.

Deployment also encountered a separate 150-MiB `/tmp` capacity limit. Two APK
copies and partial transfers filled it. Only this task's APK staging files were
moved into a private `/data` staging directory, restoring over 144 MiB of free
temporary space. Exact-block reconstruction and whole-file hashes guard future
installs; truncated copies were never installed.

The diagnostic HUD helper also passed the actual Android ART run: 25 cases and
937 assertions, with the host-only monitor test explicitly skipped. Five
read-only clock brackets subsequently put HUD wall time 931-1048 ms behind
comma wall time. These are two-device clock brackets, not Bluetooth latency.

## Sender-Only Clock Experiment

A bridge-only same-signer candidate (`68f195b01517`) passed 126 host tests and
was installed under a fresh ignition-OFF/noOutput guard. All three installed
Naver APK hashes remained unchanged. The sender samples time inside its actual
socket write lock; periodic synchronization follows the original native
observation and requires a newer observation afterward. It does not invent a
new native timestamp or increase the receiver's TTL/future tolerance.

This was not a complete fix. The HUD still reported `guidance_future` with a
299-ms delta. A 180-second capture contained 176 samples, only 98 with matching
route/instruction provenance ready, and four separate route generations.
Verified whole geometry reached the comma (5572 points, 24756 encoded bytes),
and isolated zero-speed TBT packing produced the expected 47 float32 inputs.
This is input-path evidence, not live model inference or continuous availability.

The first collector compared published coordinates against float64 source
values at 1e-9 tolerance. `NavRoute.Coordinate` is float32, so that comparison
incorrectly reported mismatches; subsequent collectors compare exact float32
wire values without changing route geometry or the schema.

The experiment also included one test-harness error: a shell foreground-service
start was sent to the ordinary bound navigation service. Android killed that
process with `START FOREGROUND SERVICE ANR`; its native exporter/listener binding
recovered it. That manual start is removed from the installer. This event is
not evidence of a bridge code crash, nor may this interval establish uninterrupted
transport. Package replacement and the later ANR are separately recorded.

A separate reviewer reproduced a more important receiver issue: a delayed wall
clock rollback could make a truly 3001-ms-old observation appear only 111 ms old.
Therefore sender-only synchronization is diagnostic, not a release solution.
The next candidate uses a bounded receiver-originated challenge and the
receiver's monotonic age, accepting only genuinely new native observations
captured after the phone receives that challenge. It must not relax the
two-second deadline or replay cached guidance to obtain availability.

## Monotonic Challenge Device Checkpoint

Same-signer bridge `15d1177cca1f` and HUD `b41f741fba75` were installed and their
complete installed bytes verified. The HUD ART suite passed 35 cases / 1955
assertions, with only the host clock-interception case skipped. A separate
actual-APK Wire fixture encoded a 130-byte challenge using the installed
Sender/NotificationEvent/NavdyEventUtil/Wire/NavdyEventWriter and successfully
decoded it through the frozen phone reader. These are isolated ART checks,
separate from the running Bluetooth observation below.

The fifth Claude Code read-only review (session
`9609fbb1-6a27-47c5-9ade-7058e585caaa`) agreed that nonce age bounds original
capture age conservatively. Its unseen-session withdrawal claim was based on
an earlier source revision and did not reproduce against frozen `b41`.
Frozen relay/sender tests also verified the eight-credit queue bound, connection
identity reset and non-renewal by withdrawal. Credits are not reclaimed merely
because a queued challenge expires. Source reviews and synthetic queue doubles
do not prove delivery through the actual TaskManager.

An independent cross-language suite exercised 55,409 assertions with a synthetic
5572-point/25-chunk route. It stayed ready after assembly at 100-ms each-way
delay and 200-ms delay plus jitter; 300-ms delay plus jitter reproduced assembly
starvation. A seven-second transport stall produced a 20.5-second readiness
outage. These are timing simulations, not measured device throughput.

The first actual v2 180-second run had 149 ready samples out of 176. After
initial assembly at 9.3 seconds, there were still 18 unavailable samples and
multiple generations. All ready samples matched all 5646 published coordinates
at their exact Float32 wire precision, and isolated TBT packing produced the
expected right-turn command in 47 float32 values. The active route changed when
guidance was resumed; these counts must not be compared as one unchanged route.

A subsequent 180-second high-frequency capture had 881 ACK updates, 594 ready
and 18 missing model components. Missing intervals correlated with 30-second
whole-route reoffers; the maximum interval between new observed sequences was
3.21 seconds. This is not a continuous-readiness pass. The HUD reported no v2
timestamp rejections. Socket write duration alone does not measure air delivery.

An isolated ART benchmark used a SHA-verified cached 5627-point/25116-character
route, not a synthetic live input. Across 30 interleaved pairs after warmup,
full `acceptV2` took p50 115.049 ms / max 175.625 ms; small guidance took p50
1.948 ms / max 4.215 ms. Route decode plus SHA took p50 101.755 ms / max
158.250 ms. All 66 observations including warmup were accepted. This identifies
measured receiver cost, not the complete cause of every Bluetooth delay.

## Package Replacement Recovery

After replacing the bridge APK, the existing Naver process continued creating
native observations but logged `binder_unbound`; the bridge received none.
The exporter now handles binding death, null binding, connection timeout and
Binder errors with one owned registration, bounded retry and stale-callback
rejection. Rebinding does not replay or re-date an old native observation.

Naver-only `07a4986fb300` changed only the exporter family in `classes44.dex`;
the original two split APKs and 9688 other ZIP payloads were preserved. Its 36
host tests passed. Bridge-only `0c0391a6db0d` then added receipt-slot release,
strictly increasing same-session capture time and consistent position rejection;
159 host tests passed. The rejected four-second full-route reoffer experiment
was not installed; reoffer traffic remained bounded by the prior schedule.

Both exact APK updates were installed without uninstall/data clear under fresh
ignition-OFF/noOutput guards. Naver remained PID 3606 across bridge replacement;
new bridge PID 5800 automatically received fresh native guidance without a
manual Naver restart or shell foreground-service start. A following 60-second
capture kept the same bridge PID, advancing fresh accepted observations from
54 to 109. This validates this device's package-replacement recovery, not all
Android versions or full Bluetooth availability.

At this checkpoint the comma remained offroad, Panda was `noOutput`, and no
usable GPS position was present. Jetson was disconnected/off; `mapsd` was not
running under its existing onroad-only policy. No live map input, inference,
physical Navdy display, actuation or road-performance pass is implied.

## Verified Cache And Recovery Follow-Up

The phone cache-ACK checkpoint is `1bc9dc737bbb39c95c7a3cc2322336ba8888040211e1e33839ea713f4f529dba`;
its initial HUD checkpoint is `3a324e263819888213b8255be14cd161105c7a9995b413072fcef0944b419703`.
Both were installed in phone-first order without data clearing and their whole
installed APK hashes were verified. The Naver `07a4986f` base and original two
splits remained unchanged. This package replacement again recovered without
a manual Naver restart. Original signers were preserved.

HUD challenges optionally acknowledge only the exact verified route identity,
hash, point count and encoded length. A matching phone cache lease lasts at
most two seconds; null/mismatched evidence requests a bounded fresh-observation
reoffer. The lease never advances capture/sequence or enables model input.
The phone also claims proof/receipt credits before dispatch, preventing a fast
reverse response from recreating an old slot before write completion.
Four independent gated-socket tests reproduced one dropped/eight intended
credits before the fix and zero dropped/eight recovered afterward; failures
invalidate the old epoch. This is a writer fixture, not an air-link test.

On comma, only a previously complete verified immutable route may be quarantined
across a guidance-only gap, for a fixed ten seconds. It remains unavailable
until a newer fresh native sequence supplies an exact matching chunk/manifest.
Cancellation, invalidity, identity changes, stale link and disconnect discard it.
Restoration rotates the sidecar token. A synthetic 5572-point/25-chunk jitter
replay reduced a 53-second recovery outage to 150-ms fail-closed gaps; initial
assembly under that jitter still fails closed. These are not device timings.

The web no longer unconditionally reports Navdy as lacking a full route. It
requires current matching source/UInt64 route identity and original deadlines
from map and model producers. PNG bytes are hash-bound to their metadata;
browser images/readiness expire. Old network previews and identity-free status
cannot become Navdy readiness. Missing Jetson/GPS remains unavailable, not ready.

Verification at this checkpoint:

- Phone: 172 host tests, no failures/errors/skips; four independent preflush
  race checks also passed. Only allowed classes in `classes2.dex` changed.
- HUD: 39 host cases / 2226 assertions; actual ART 38 cases / 2223 assertions
  with the host-only clock monitor skipped. Relative to installed `b41`, only
  `classes5.dex` changed; all other non-signature payloads were unchanged.
- Actual installed HUD APK state/Sender/NotificationEvent/NavdyEventUtil/Wire/
  NavdyEventWriter to the frozen new phone reader: nine cases passed, largest
  encoded frame 427 bytes. No substitute HUD helper was packaged in that fixture.
- Isolated comma native full regression: 1297 tests and 146 subtests passed in
  79.82 seconds. Separate navigation/web regression: 290 passed in 18.52 seconds.
  An initial four test-fixture failures used an already imported real messaging
  attribute; patching both import references fixed the fixture, not production IPC.
- Browser logic: 54 Node tests passed. Ten existing aiohttp test warnings remain.

All native tests used an isolated candidate and fresh ignition-OFF/noOutput
guards. Final published commit, installed runtime and live availability are
recorded below after the final read-only Claude review.

## Nonce Retention And Expired ACK Corrections

The initial cache-ACK checkpoint was not a continuous-availability pass: a
90-second observation had 447 ACK updates and zero ready route samples, even
though the phone continued producing fresh native observations. Diagnostic HUD
`77b88743d017` retained rejection counters without changing acceptance policy.
It recorded one full-route rejection with `invalid_challenge`, nonce absent,
and no recorded nonce age; two subsequent full routes were accepted. This does
not prove that the observed rejection occurred before the nonce deadline.

Separately, deterministic source tests reproduced premature eviction: a
receipted but unconsumed nonce could be reused at 1600 ms, before its 2000-ms
validity deadline. HUD `f6d5367f8cf61d7d566371e36d20c2036f148262a1f305207083f780105cce8f`
retains that slot until consumption, invalidation or expiry. An unreceipted
slot still cannot be freed by expiry alone. The eight-slot bound, at least
200-ms issuance cadence and two-second freshness contract remain unchanged.
Independent tests accept a valid delayed proof through 1999 ms and reject it
at 2000 ms, with or without the pre-consumption pump.

The exact same-signer APK was installed under a fresh ignition-OFF/noOutput
guard and pulled back for whole-file SHA-256 verification. Relative to `77b`,
only `classes5.dex` changed; 7049 other non-META-INF ZIP entries matched.
Host tests passed 44 cases / 2365 assertions; actual Navdy ART passed 43 cases /
2362 assertions, excluding only the host clock-interception test. These ART
helper tests do not measure live Bluetooth delivery or model inference.

Claude Code's completed read-only review
`90e859cb-b67c-4168-b4a7-def16965c127` reproduced an expired-tail recovery defect:
charging a complete ACK round-trip can push an otherwise valid repeated response
past its two-second deadline, and the ordinary bounds rejection discarded the
quarantined geometry. A follow-up review
`9291ac2a-fd01-4dc7-bd6a-13c3877d63ea` found the related earlier-deadline case:
new round-trip jitter can expire that response before the tracker's previous
deadline. Both are availability defects; the affected paths fail closed rather
than publishing stale guidance.

The final correction accepts neither tail as fresh guidance. It only preserves
already complete geometry, with retention bounded by the earliest previous/tail
deadline and current time. Before/on/after-deadline regression cases expanded
to 262 passing tests, including malformed, conflicting and newer-stale inputs.
Claude's focused follow-up `999c7f03-277b-41f9-bd41-578ac8cc155f` found no remaining
reproducible defect in these two corrections. Claude read source/tests only;
native execution and live observations below are separate parent-run evidence.

The exact installed `f6d536` APK's state/Sender/NotificationEvent/NavdyEventUtil/
Wire/NavdyEventWriter path passed nine actual ART encoding/phone-reader cases,
largest frame 427 bytes. The fixture contained no replacement production HUD
helper. A 60-second Bluetooth observation before the comma update had all 298
ACK samples ready, no missing model component, and phone full-route writes
unchanged at 24. A startup nonce-absent rejection remained in diagnostics, but
no new rejection occurred during this interval. This is bounded stationary
evidence, not a long-duration or driving guarantee.

TMAP remains separate: its existing display guidance/400-m road preview is
preserved, but no verified whole-route model producer is enabled. A raw native
vertex-array API does not prove whole-route coverage. Do not claim TMAP NoO
support from the Naver transport result.

The final isolated native comma regression passed 1513 tests and 146 subtests
in 101.96 seconds. The separate native web/map suite passed 290 tests in
18.71 seconds; the browser-side Node suite passed 54 tests. These runs used
the final quarantined-route and preview-provenance corrections above. They do
not substitute for an installed running-process check or Jetson inference.

After the user switched real phone guidance to TMAP, the bridge reported
`TMAP raw guidance active=true` but `selected_source_unsupported` for model
navigation. This is the expected unsupported-producer boundary, not a successful
TMAP NoO result. The previously received Naver route was not reused as TMAP.
