# Traverse Speed Display Parity

## Scope and Source

Display calibration port for `CHEVROLET_TRAVERSE`. The speed subtask was
source-only; the parent subsequently staged native tests and selectively
installed the five runtime files on the parked Comma, alongside the separately
documented Navdy lifecycle work. No commit or push is included in this turn.
Existing unrelated working-tree changes are preserved.

Reference: local `wayon/Sunnypilot` at
`b7660d1d403938c464d4f14f842449e3f0e64ebd`, not the dirty
`codex/my-traverse-source` working tree. Reference paths below are under
`opendbc_repo/opendbc/car/gm/` (the root `opendbc` is a symlink):

- `cluster_speed.py`: the existing empirical Chevrolet display table and both
  interpolation directions. Introduced in `8963e98584`.
- `carstate.py`: converts `vEgo` to `vEgoCluster`, and raw stock ACC
  `cruiseState.speed` to `cruiseState.speedCluster`.
- `carcontroller.py`: `get_acc_dashboard_speed_kph`, introduced by
  `1b9a66241eac520c74279de1a810eee5c2830ec4`, inversely converts the displayed
  target before packing the replacement stock ACC dashboard message.

The copied conversion module has an exact Python AST match to the maintained
reference, excluding comments. Its samples are inherited empirical data, not
new measurements taken during this work. Application is intentionally narrower
than Wayon: only Traverse receives this calibration, not all GM platforms.

## Findings and Changes

1. Current Carrot GM state parsing lacked the Wayon mapping. Current-speed
   display usually fell back to chassis `vEgo`; stock ACC display fell back to
   raw `ACCSpeedSetpoint`. Port the mapping to the two display fields only.
   For example, raw 95 km/h corresponds to displayed 100 km/h; raw 66 to 70.
2. Carrot's default `SpeedFromPCM=2` retained a local cruise target and displayed
   it even when stock Traverse ACC owned the set speed. With `pcmCruise`, select
   `cruiseState.speedCluster` for `vCruiseCluster` while ACC is available. Keep
   the existing `vCruise` target and all mode-specific control selection intact.
3. With openpilot longitudinal control, the outgoing 0x370 dashboard speed
   lacked Wayon's inverse conversion. Display SET 100 is encoded as 95 on this
   Traverse, while SET 70 is encoded as 66. Other dashboard fields are unchanged.
4. Carrot could send a transient desired/planner speed to the stock dashboard
   while Comma showed SET. The Traverse HUD command now uses `vCruiseCluster`,
   matching the displayed SET rather than an instantaneous slowdown. This only
   selects `hudControl.setSpeed`; no planner target or actuator is replaced.

Preserved: wheel speeds, `vEgoRaw`, filtered `vEgo`, `aEgo`, standstill logic,
`wheelSpeedFactor`, existing `vCluRatio`, existing +/-0.5 km/h cluster hysteresis
and low-speed cutoff, steering/gas/brake calculations, safety and bus routing.
As before, Carrot's SET/resume/gas-sync logic can consume `vEgoCluster`; after
this port those existing paths see the corrected displayed current speed.
This is not a new global scaling of measured speed or closed-loop targets.

## Changed Paths

- `opendbc_repo/opendbc/car/gm/cluster_speed.py` (new, reference conversions).
- `opendbc_repo/opendbc/car/gm/carstate.py` (Traverse display fields).
- `opendbc_repo/opendbc/car/gm/carcontroller.py` (0x370 display inverse).
- `openpilot/selfdrive/car/cruise.py` (stock Traverse displayed SET selection).
- `openpilot/selfdrive/controls/controlsd.py` (Traverse HUD SET selection).
- `opendbc_repo/opendbc/car/gm/tests/test_display_speed.py` (new focused tests).
- This report.

## Navdy Integration Handoff

No Navdy speed conversion was changed. The current `openpilot/selfdrive/navdy/display.py`
already prefers the correct fields:

- `vEgoKph`: `carState.vEgoCluster` in m/s, converted once to km/h.
- `setSpeedKph`: `carState.vCruiseCluster` already in km/h.
- `actualAccSetKph`: `carState.cruiseState.speedCluster` in m/s, converted once
  to km/h; this remains the separately reported stock ACC observation.

Do not apply the conversion table again in the Navdy bridge or APK. Comma's
standard and Mici HUDs already select `vEgoCluster` / `vCruiseCluster`, so no
renderer edit is required. Card publishes the cruise helper's two fields as
`carState.vCruise` / `carState.vCruiseCluster`.

## Validation and Limits

Run the focused suite with:

```sh
/tmp/navdy-wireless-py312-20261019/bin/python -m pytest -q -c /dev/null -p no:cacheprovider --confcutdir=opendbc/car/gm/tests opendbc/car/gm/tests/test_display_speed.py
```

Tests execute the real Python CAN packer/parser, CarState, interface hysteresis,
Kalman filter, schema, cruise helper and GM controller. Params uses in-memory
storage; realtime constants and Bluetooth command input are isolated. The HUD
selection test executes the exact source AST block, not the controlsd process.
Host result: **129 passed in 1.14 seconds**; `compileall` and `git diff --check`
also passed. No native `params_pyx` is required by this isolated suite. The same
test command can run against a staged complete checkout on Comma without CAN
transmission, service restart or persistent Params writes; it does not start
`card`, `controlsd` or a messaging publisher.

- Every inverse table sample, interpolation, zero/reverse and both display units.
- Actual synthetic CAN input to raw/current-display/stock-SET fields.
- 590-sample acceleration/deceleration/stop sequences on Traverse, Bolt EUV and
  Volt: only the two Traverse display fields differ from the pre-port state path.
- Stock/op-long, all four PCM settings and metric/imperial cruise-helper paths.
- 24 controller scenarios over 100 ticks each: returned actuators and all CAN
  frames other than 0x370 identical to the pre-port encoding; decoded 0x370
  fields other than `ACCSpeedSetpoint` identical too.
- Other GM dashboard conversion and non-Traverse controlsd selection unchanged.

The combined Traverse/Navdy staged suite also passed 309 tests on Comma's native
Python runtime, without starting card/controlsd or publishing CAN. Runtime
source hashes and the existing branch/HEAD were checked before selective
installation, with per-file backups. The parked Comma was rebooted and SSH,
fresh manager services and all installed speed-source hashes were verified.
This loads the new preimported modules; it does not exercise card/controlsd or
claim physical speed matching while the vehicle is offroad.

This is synthetic desktop/native validation, not live CAN replay, MCU safety testing,
physical cluster matching or road validation. The source table contains repeated
raw values: 26/27 both map to 25, 29/30 to 28, and 38/39 to 36. Wayon chooses the
lower display value in the inverse direction; exact round trips in those bins
are impossible from these observations alone. Existing hysteresis and CAN
quantization also remain. Speeds outside the table use Wayon's extrapolation;
no new calibration or accuracy claim is made for those ranges.
