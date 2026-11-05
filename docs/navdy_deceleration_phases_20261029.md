# Navdy Deceleration Badge

VTURN/ROUTE preparation and unused-completion behavior below is superseded by
the sender filter in `curve_applied_only_20261030.md`. The original APK's 300 ms
white completion confirmation is separate from hiding unapplied curves.

## Contract

The Navdy badge sits to the right of ACC set speed. It uses the same pure
`deceleration_state()` verdict as the C4 New UI. This reports the speed-cap
progress, not measured braking, actual vehicle speed, or permission to accelerate.

- Waiting: solid white while a valid reason is not yet constraining the cap.
- Decelerating: orange `#FF9500` / white, one-second cycle, 60% orange.
- Complete: solid green `#20D86A` when this reason reaches its final cap, or the
  selected internal set speed already meets that endpoint.
- Disengaged, longitudinal inactive, reverse, offroad, invalid/stale critical
  inputs and alerts: hidden. Driver pedal override downgrades to white.
- ROUTE/VTURN completion is confirmed for 300 ms locally, white until then.
  Source/phase changes, clear and monotonic time reversal reset animation state.

The existing Navdy `setSpeedKph` display uses `vCruiseCluster` calibration.
The verdict deliberately uses `vCruise`, the same internal target as C4/control;
neither speed value is changed. Each display animates locally, so physical blink
edges need not be synchronized across the radio link.

## Optional Wire Field

The existing `navdy.openpilot.v1` full snapshot gains one integer `decel`.
Zero clears it. Otherwise `code = source_id * 4 + phase` with phase 1/2/3 for
waiting/decelerating/complete. Source IDs are stable:

| ID | Source | Label |
| --- | --- | --- |
| 1 | cam | CAM |
| 2 | section | SECTION |
| 3 | bump | BUMP |
| 4 | police | POLICE |
| 5 | waze | WAZE |
| 6 | road | ROAD |
| 7 | atc | TURN |
| 8 | atc2 | TURN |
| 9 | hda | CAM |
| 10 | hda_section | SECTION |
| 11 | hda_bump | BUMP |
| 12 | school | SCHOOL |
| 13 | vturn | VTURN |
| 14 | route | ROUTE |

Unknown/absent codes clear rather than retain a previous green badge. Legacy
MODEL/TURN sources are not newly generated or assigned IDs. Do not recycle IDs.
The USB and Jetson relays pass the full JSON unchanged; Bluetooth serialization
likewise retains the new field. No new socket, packet, heartbeat or subscription.
The existing 200 ms display tick, retry policy, TTL and phone reconnection remain
unchanged. A missing radio packet alone does not clear before the existing TTL;
an explicitly received zero/missing field clears immediately.

Compact JSON grows by 10-11 bytes, at most 55 raw bytes/second at the existing
5 Hz cadence. Compression may shift matches, so the tests also bound the actual
compressed size difference. These are serialized-byte checks, not a measurement
of physical radio latency or long-drive stability.

## Validation

- C4 New UI: 144 desktop tests pass after extracting the shared selector.
- Navdy display/bridge/reconnect/BT pipeline/transport/vehicles/lanes: 630 pass,
  including 145 new verdict/freshness/overhead tests.
- Producer/schema/curve/navigation: 272 pass, including 80 settings x 40
  synthetic inputs with unchanged preexisting control publication fields.
- Full Navdy suite: 1222 pass, 58 fail on this Mac. Running the inherited suite
  with exact pre-change `display.py` from `1e47d6df` produces the same 58 failures:
  five missing native Params dependencies, 53 preexisting log-storage tests
  blocked by the real free-space reserve (about 1.6 GiB available). Production
  reserve checks were not disabled. This is not a full-suite pass.
- `tools/navdy/deceleration_fixture.py` exports 42 full snapshots through the
  production Python encoder (14 sources x 3 phases) for Android-side checks.
- A separate 1344-input comparison with exact pre-change `display.py` preserves
  all original payload fields; raw growth is 10-11 bytes, compressed growth
  5-8 bytes for those cases. This is not a universal compression bound.
- Claude's read-only review found the explicit Jetson host-bundle include list
  missing the new shared module. It is now included. Two bundle tests pass,
  including an isolated import from only exported dependency paths without C4 UI.
- Bundle, Navdy relay selection and management regression checks: 20 pass.
- Exact pinned Navdy v172 DEX inspection resolves the review's lock concern:
  `WirelessLink$1.apply` calls the receiver synchronously inside the same owner
  thread, so its read-only diagnostics lock is reentrant. Bluetooth posts the
  dispatch before waiting; it does not hold the owner lock while waiting on UI.
  `WirelessLink$3` runs expiry/CLEAR from the main Handler, not the watch thread.
- Actual wired service ingress uses `ClientRunnable$1.run` ->
  `applyUsbReceived` -> `WirelessOwner.usb`, which assigns USB ownership before
  the receiver callback. Unowned legacy debug broadcasts deliberately do not
  show the badge. These are pinned-artifact call-path checks, not radio tests.
- Layout review caught the initial overlap with the right BSM icon, then a
  second overlap with the translated navigation footer. Both are covered in
  final APK layout checks; checking only the ACC number row is insufficient.

## Android Build

The paired source is `hud_patches/v173-deceleration-badge/` in the local Navdy
repository. Final badge bounds in the existing 640-pixel layout are
`(390,297)-(438,312)`, 48x15 px, text up to 14 px with native width/height fitting
and 3 px horizontal padding. This matches the original auxiliary-speed number's
x390/y297 anchor and width; its old 30 px height would overlap navigation, so
the badge is shorter. ACC set, BSM and navigation positions are unchanged.
It sits right of ACC set speed and entirely left of BSM. It clears
the section-camera view and translated route footer starting at y313. Other navigation and RPM neighbors
are checked against the exact baseline DEX. Host-rendered preview inspected;
its font/rasterizer is a substitute, not physical HUD evidence.

Clean `build/aux-position/` passes 648 host assertions, including 42 production
snapshots, state/expiry tests, API21 checks and exported-hook/layout checks.
Only `classes2.dex` changes: two original receiver methods and one new helper.
Other 7,061 APK payloads are identical; classes2 has 65,439 method references,
below its 65,536 limit. Manifest/version, transport, ambient and phone-reconnect
classes are unchanged. Independent parent checks confirm the matching source
manifest/APK hash and unchanged other DEX, manifest and resource payloads.

Unsigned APK SHA256:
`ec9d629ce6675e7524fe93ae1f30244b9bc09af8f0cdee36fda5240b5af12690`.
Do not deploy `test1`, `candidate*`, `release` or `adjacent`: they predate the
final matching build. Those preparation results predate device deployment;
the later native checks are recorded below.

The user corrected the first x480 placement: the badge belongs directly beside
ACC set speed, not beyond BSM, and finally requested the original auxiliary-speed
number's position. The original layout test had incorrectly enabled
legacy `automaticAccActive` data not sent by Carrot. Final tests use the actual
production contract and confirm the baseline hides that old target area by
default. An explicit legacy automatic-active payload hides the new badge to
avoid collisions without changing the old renderer. Current Carrot payloads
do not carry that legacy flag; all 42 encoded fixtures verify this.

No steering, braking, navigation selection, Bluetooth transport, phone automatic
reconnection or ambient lifecycle logic is edited. Native Android and
physical HUD evidence must be reported separately from desktop tests. Deploy
the prior C4 producer/schema changes together with the shared helper and sender;
old producers deliberately yield no badge.

## Native Checks On October 29

The final APK was signed with the existing certificate and installed with
`pm install -r`, preserving application data and pairing. Actual installed SHA256:
`fc0460b05dc2ef7fa4c54da56babd12a70b81360b0bc09246cbb7c0230b1b22f`.
The previous v172 APK is retained privately for recovery. All non-signature
payload bytes match the final unsigned build. The first post-install monitor
stopped on `stale_status` during normal service re-advertisement; the APK install
and hash check had succeeded. No repeat install or pairing reset was performed.
The existing Bluetooth path subsequently reconnected automatically.

On the actual Navdy API22 / ART2.1.0 environment, an isolated process loaded the
installed APK's actual badge class and passed 33,569 assertions, including native
Canvas/Typeface pixel and bounds checks. All 42 codes and phase/TTL/reset edges
were exercised; 47 PNGs were collected and the 14x3 atlas inspected. The probe
never invokes the receiver, transport or owner and never attaches a live window.
Thus this proves native software rendering, not live engaged optical appearance.
The original OpReady icon ends at y297 where the badge starts: zero gap, zero
area overlap, not a positive clearance margin. Physical readability remains open.

On Comma, an isolated source overlay ran the full Navdy suite plus C4 New UI
and candidate tests: 1,466 passed, one unrelated inherited test failed.
`test_reader_real_null_model_is_not_discard` overwrites a file in place twice
within the filesystem's timestamp tick. A bounded 30-run check reproduced the
same stamp collision in unchanged production code (26/30) and the overlay
(28/30), with identical `navigation.py` SHA256. Production uses atomic file
replacement rather than that test's in-place overwrite. This is not a claim
that the full suite passed; neither the reader nor its freshness policy changed.
