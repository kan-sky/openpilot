# Navdy solid and dashed inner lanes (2026-10-25)

## Scope and contract

The Comma C4 renderer reads `carState.leftLaneLine` and `rightLaneLine`.
For an inner model lane with raw probability greater than 0.3, nonnegative
codes ending in zero are dashed and other codes are drawn solid. Mirror that
display policy in the Navdy sender; do not change the vehicle/vision producer
or lane-change control policy. Documented color/type codes 0..38 are accepted
except reserved type 9. Invalid, missing or low-confidence inputs are unknown.
Raw code 0 remains dashed, including the schema's default zero. This is not a
new independent claim that a camera positively classified that marking.
Reserved/out-of-contract codes (9, 19, 29, 39, or 40+) intentionally become
unknown here, unlike C4's broad nonzero-remainder solid fallback. The current
producers and sampled drive use documented codes; this is not exact parity
for malformed or future undocumented codes.

The installed v169 APK already implements `navLaneLeftType` and
`navLaneRightType` as **strings**: `solid`, `dashed`, `unknown`. No APK change
or reinstall is required. Unknown/missing types retain its existing dashed
fallback. Do not send `centerSolid` or `centerDashed`, which would also change
the color. Far lane types, road edges, colors, geometry, vehicles and all
control fields stay unchanged. Double-line codes follow C4's solid convention;
this feature does not add a second stroke or change the Navdy layout.

Types accompany every active valid-geometry snapshot. The native renderer
resets missing types to unknown when full geometry arrives, keeps types with
geometry on vehicle-only updates, and clears both on invalid/empty geometry,
inactive state or null input. Existing 500 ms service freshness and receiver
deadlines remain unchanged. No old type is cached in the sender.

## Source and recorded evidence

The pinned APK SHA256 is
`d1268a7bebbecd34e4ca48f1abac2518138f5048ab14d912c9eda53758c26651`.
Its actual `OpenpilotStateReceiver` passes the original parsed JSONObject to
`OpenpilotPathView`; `readLaneType` accepts the strings above and `drawLane`
uses a null PathEffect only for solid types. Other lanes independently reset
the shared Paint state. REA Android analysis was unavailable because the local
JADX MCP provider was not configured; the exact APK was inspected with the
existing pinned dexlib/baksmali tools instead.
Both receiver and renderer class dumps came from that same pinned APK.

Eight retained recent-drive qlogs contain 5,344 valid carState samples:
4,440 pairs (0,0), 195 (0,1), 454 (1,0), and 255 (1,1). Both sides really
change in the source data. These counts alone do not prove optical recognition
accuracy or synchronize a physical Navdy screen with the C4 screen.

A separate regression replay used 1,756 retained display snapshots with a
synthetic 3x3 lane-code matrix: 15,804 comparisons, 11,160 style-bearing cases.
Removing only the two added keys produces the exact prior JSON, including
geometry, vehicles, order and scalar values. Compressed requests add 15..30
bytes (mean 22.49) on those fixtures. At the unchanged 5 Hz this is at most
150 additional compressed payload bytes/second in this sample, not a measured
radio-capacity or stability guarantee. The genuine lower-level Bluetooth
stalls in `navdy_radio_stall_20261024.md` are separate and remain unresolved.

## Validation status

- Parent reran the complete Navdy suite: **1,130 passed, five deselected** on
  macOS. The exclusions are the existing native DBC cases unsupported in that
  desktop environment, not new failures. Nine local renderer-verifier/runner
  self-tests also pass. Ruff and diff checks pass for the sender.
- Claude reviewed the fixed sender, actual receiver/renderer, isolated replay
  and deployment batch. Parent verified the findings against actual code and
  device output. Add a pre-merge interpreter/import/origin check and target
  blob hash verification; Android 5 needs echo markers and toolbox ls columns.
- Actual installed v169 renderer: **38/38 frame checks and 38/38 decoded PNG
  checks pass** in a separate shell-uid-2000 app_process, unattached 320x100
  Canvas. This covers all left/right style pairs, independent risk tint,
  missing/invalid fields, full/partial geometry, clearing and restoration.
  All 13 bitmap resources load. Existing pixels outside the lanes match.
- The initial pixel oracle incorrectly assumed opaque lanes. APK
  `onSizeChanged` uses a vertical alpha 85-to-255 shader; the corrected oracle
  checks that gradient (two alpha-unit rounding tolerance), continuous solid
  coverage, transparent dash gaps and equal risk tint on shared painted pixels.
  The fixture/JAR/APK did not change to obtain the pass.
- HUD PID 2581 and phone connection service PID 2601 remained running. APK,
  pairing, phone registry and Bluetooth-stack configuration hashes match.
  Own helper process, files and ART cache were cleaned. Android 5's unsupported
  `settings list` printed usage with exit zero: its hashes are **not** evidence
  of settings-table preservation. The runner now uses four named `settings
  get` queries, independently confirmed readable; this revised snapshot was
  not used for a second native capture. Existing config/PID evidence is separate.
- Runtime commit `c806b80c` was pushed to `traverse/WIP-FR` and fast-forwarded
  on comma-db5ce68d after sustained fresh offroad/ignition-OFF/inactive guards.
  Native Cap'n Proto lane-field smoke checks pass. Tracked bytes and original
  generated/untracked files were preserved. Only the Navdy bridge restarted,
  PID 912103 to 967871; carrot_man stayed 741567. Initial Bluetooth reconnection
  took 7.488 seconds, with received/applied sequence 5 and zero USB updates.
- Subsequent 60-second observation: 270/296 samples connected, sessions
  663 to 664, one observed peer reset followed by reconnection. Final received
  and applied sequence both 259; Bluetooth-only, zero USB updates. Offroad
  traffic has no active lane-style payload. This is not a radio-stability pass
  or evidence that the style bytes caused that pre-existing stall.

No APK install, phone reconnect policy, TTL, display cadence, lane geometry,
steering or braking change. Physical screen/road confirmation and the separate
Bluetooth root-cause investigation remain outstanding. Private APKs, recorded
inputs, PNGs and device logs are not included in Git.
