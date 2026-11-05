# Navdy surrounding-vehicle display restoration

## Scope

The WIP-FR Navdy mapper previously used only `radarState.leadOne/leadTwo`.
Wayon/sunnypilot at `b7660d1d403938c464d4f14f842449e3f0e64ebd`
also read radar objects for adjacent-lane display. Its receiver already accepts
up to eight `navVehicles`; the installed v166 HUD does not require an APK update.

Subscribe to the existing `liveTracks` publication in the Navdy bridge only.
Prefer `displayPoints` when `hasDisplayData` advertises that stream, including
an explicitly empty list; otherwise use `points`. No second CAN parser,
radar publisher change, steering/braking change, model change or safety change.
Only the outgoing `navVehicles` list and diagnostic vehicle count are extended.

## Display Policy

- Preserve published lead positions and longitudinal highlighting.
- Add fresh radar objects in the ego/adjacent lanes within 2-80 m; select nearest
  extras while reserving lead slots, then draw far-to-near, at most eight total.
- Bound tracking to 64 objects. Require two distinct fresh snapshots; smooth
  coordinates with alpha 0.55. Radar ID jumps must reconfirm.
- Clear extra objects on a fresh disappearance, empty snapshot, radar error,
  time rollback, flipped-coordinate change, disconnect, inactive display,
  missing geometry or age greater than 350 ms. Never extend stale data's life.
- Published tracks have already had RadarTrackFlip applied. Convert the lateral
  convention once, not twice. Deduplicate physical lead IDs and nearby
  camera-only lead matches. Display streams can have independent ID namespaces;
  associate those by bounded lane/position/velocity matching, not ID equality.
  Changing the selected stream resets confirmation.
- Use Wayon-style lane/path fallback and perspective yaw. Current RadarData
  does not contain Wayon's raw target width, so width compensation is not
  reproduced. Raw radar objects are not guaranteed to be actual vehicles.

All state is private to the display overlay. No control message is published.
Bluetooth-only telemetry, USB diagnostics, watchdogs, ambient power policy,
current-speed policy and navigation input are unchanged.

## Verification Before Deployment

- Mac isolated test environment: 633 Navdy tests passed, excluding the native
  ambient module. Ruff and `git diff --check` passed. Three optional pytest
  configuration warnings do not indicate test failures.
- Actual Comma, temporary source overlay using installed schemas/native
  dependencies: all 649 Navdy tests passed, including the ambient tests.
  Initial test runs lacked pyserial; selecting the installed pydeps path fixed
  that test-environment error without installing production dependencies.
- Independent review caught missing displayPoints support and its independent
  ID namespace; corrected with authoritative-empty, colliding-ID, geometry,
  velocity and source-switch regressions. Final independent review passed
  43 focused tests with no remaining scoped blocker.
- Five retained full-log segments were replayed privately on the Comma without
  publishing messages or sending synthetic telemetry to the HUD. Among 1,780
  sampled frames, 133 passed existing active-display gates; four gained extra
  vehicles, including left and right lanes. Maximum count increased from one
  to three. Every non-vehicle payload field compared equal. Maximum encoded
  frame was 2,224 bytes, below the existing 32 KiB bound. Overlay p99 was
  0.307 ms across this mixed active/inactive replay, not a worst-case bound.
- Synthetic malformed/freshness/error/disengagement/coordinate/duplicate/
  eight-object cases cover inputs not present in that limited recording.
- A separate Android ART process loaded the actual installed HUD APK and
  invoked its `OpenpilotPathView.readVehicles`: all eight objects including
  adjacent lanes were accepted and an empty list cleared them. No UI injection,
  APK replacement or production receiver restart was used for this probe.

The replay is not physical object ground truth or a driving validation. Parked
installation and Bluetooth recovery cannot confirm optical display during an
engaged drive or guarantee long-duration radio/ambient stability. Existing
radio failures observed before this change are not claimed fixed here.

## Deployment Contract

Publish a single-parent commit only to `traverse/WIP-FR`. Require a matching
installed base and fresh offroad, ignition-OFF and controls-not-allowed state.
Preserve generated DBC/build files. Fast-forward the device, compare source
hashes, and restart only `navdy_bridge`. Keep the installed APK and Bluetooth
policy. Record process identities and bounded link observations privately.
