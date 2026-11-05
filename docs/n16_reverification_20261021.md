# Desk Re-verification, 2026-10-21

This is the earlier desk audit. See the later
[WIP-FR navigation integration](wip_fr_navigation_integration_20261021.md)
for producer/consumer corrections and their separate validation boundaries.

## Conclusion

The current installation is not feature-complete for the requested N16/custom
Jetson deployment. Native inference works with a prepared N16 engine, and the
comma's restored custom source remains installed, but the production Jetson HUD
package is older and lacks the custom ambient UI and phone-media integration.
Cold engine builds and source-safe real navigation input also remain unresolved.

The user confirmed vehicle-disconnected desk operation. Tests did not publish
driving messages, change Panda firmware, engage controls, or install a production
model. No Git commit, push, or permanent device-source update was performed.

## Confirmed Installation Mismatch

- Comma: `WIP-FR`, `a4b7fe3c5553c03ca960daba4846bb9834051269`,
  with a clean tracked working tree before and after testing.
- Jetson current/SOURCE_COMMIT:
  `84087a5b78118acc40234bfd8ef64235421f1aab`.
- Actual host files differ from the comma's renderer, model structures and HUD
  reader. Inspection confirmed no `PhoneMediaInfo`, no ambient drawing method,
  no `PhoneMediaReceiver` in the HUD entrypoint and no supplied lane-assist asset.
- Consequently, a healthy Jetlink connection or active HUD service does not
  mean the new display/music implementation is installed on that host.
- Remedy requires a complete, verified host package with rollback and matching
  update identity. Do not fix this by overwriting a few live files that the
  signed updater can subsequently replace.

## New Corrections

1. The staged N16 optional-input validator accepted complex map/navigation
   arrays, discarded their imaginary values during float32 conversion, and
   accepted finite float64 values that overflowed float32. It now rejects
   non-real dtypes and non-finite converted values before assignment. Regression
   tests cover stale destination clearing and warning-free rejection. The
   existing fallback/invalid-output policy is unchanged.
2. A longitudinal test still expected safety flags `515`, before the confirmed
   manual-hold bit was added. Its exact expectation is now `1539`, including
   `MANUAL_AUTO_HOLD`. Runtime/safety policy was not changed to satisfy the test.
   The corrected test ran against installed native interfaces from a temporary
   test overlay, not by modifying the installed checkout.

## Actual Device Measurements

| Check | Result and boundary |
| --- | --- |
| Installed Cinque, comma IPC + physical USB + TensorRT | 1,200 valid replies with matched frame IDs; mean 31.5379 ms, p95 36.0019 ms, max 104.9287 ms |
| Cinque deadline observations | Three replies exceeded 50 ms: frames 0, 2 and 994. Server total stayed about 19.4-20.0 ms. This does not isolate transfer/scheduling/IPC as the cause |
| Cinque reset | First output reproduced exactly, maximum absolute difference 0 |
| N16 prepared engine, native Jetson, corrected input helper | 1,200 valid finite iterations; mean 26.0208 ms, p95 26.2078 ms, max 30.1843 ms; none above 50 ms |
| N16 optional-input phases | Absent, synthetic active map/TBT, absent again; reset reproduced first output exactly |
| HTTP health/settings/Jetson/remote support/media/navigation | Six read-only endpoints returned HTTP 200; required custom settings names were present |

These are synthetic inference inputs, not camera/warp/modeld/DM coexistence or
closed-loop driving. The N16 recheck used a strict cached-only loader; no build
was attempted. It does not resolve the earlier empty-cache GPU allocation
failure. The USB timings include ordinary offroad CPU policy and concurrent
read-only status requests, not a strict onroad 20 Hz certification.

## Regression and Rendering Coverage

Counts below describe separate suites; overlapping runs are not a unique total.

| Group | Result |
| --- | --- |
| N16/runtime/navigation regression after input fix | 490 passed, 14 skipped: thirteen native-IPC cases unavailable on macOS, one missing reference checkout |
| Custom Navdy, ambient/lifecycle/navigation, power, GM hold/display/NNFF and live specifications audit | 1,133 Python tests and 146 subtests passed, three JS tests passed; seven native-gated cases deferred in that run |
| HUD/cluster/media transport/remote support on Mac | 297 passed; two Linux-only SEQPACKET tests could not run on macOS |
| Comma-native DM, phone risk, media and SEQPACKET suites | 226 passed, including those two Linux socket cases |
| Comma-native longitudinal settings, NNFF notices and alert presentation | 96 passed, 1,040 subtests passed; temporary corrected test used real installed interfaces with fake Params |
| Web HUD/vision availability/remote-support UI contracts | 51 passed |
| Native raylib preview at 1920 x 480 | Both themes rendered Korean title/artist, synthetic PNG album art, progress, speed, cruise and BSM; local vs serialized Jetson-format inputs were pixel-identical |

The previews used the current checkout on macOS with raylib 5.5.0.4, not the
older installed Jetson renderer. They verify rendering and serialization, not
TURZX output or backlight brightness. Both rendered images were visually
inspected; the album art was an intentionally synthetic solid-color test image.

The custom-feature audit also confirmed an existing limitation: live vehicle
mass/center parameters are not scoped to vehicle fingerprint. Saved Traverse
tuning can carry into a different vehicle. This is not a new N16 regression;
do not assume those saved values are safe for an arbitrary vehicle.

## Physical Boundaries

Navdy was not connected; its saved policy was Bluetooth-only and discovery was
timing out. No TURZX panel was attached and no current phone-media record existed.
Therefore phone Bluetooth delivery, Navdy OBD/display/sleep/ambient behavior,
real panel visibility/backlight, and physical braking/holding/NNFF steering were
not validated. No source presence or unit-test pass establishes those outcomes.

The real route producer and original route/instruction freshness contract remain
unwired. This audit does not claim N16 navigation behavior with a live phone.

## Restoration and Evidence

The N16 native test paused only the Jetson inference/HUD services, held the
updater lock and used an independent systemd restoration hook with a 250-second
deadline. It completed successfully; production services resumed and the USB
management address was rediscovered after re-enumeration.

Final fresh checks confirmed offroad/noOutput telemetry, `jetSON READY`,
unchanged Cinque v2 selection/current release, no experiment processes and no
tracked comma-source changes. Evidence and preview images are private under
`.analysis/n16-jetson-20261021/`; temporary test dependencies live only under
`/tmp/hl-pytest-reverify` on comma. The production Python environment was not
modified.
