# Live vehicle specifications

## Settings contract

Carrot Web: Carrotpilot > Driving Control > Vehicle Steering > Vehicle
Specifications (`STEER_VEHICLE_SPECS`).

| Parameter | Unit | Input range | Default |
| --- | --- | --- | --- |
| `VehicleMass` | Curb weight, kg | 500-5000, integer | Installed vehicle curb weight |
| `VehicleCenterToFrontRatio` | Front axle to CG distance / wheelbase | 0.10-0.90, decimal | Installed vehicle ratio |

The ratio is not CG height or front axle load. Enter 0.70 directly, not 70.
The ratio buttons move by 0.01 times the selected multiplier. Direct input
also accepts finer decimals, such as 0.735.
Mass overrides exclude the standard 136 kg payload; it is added exactly once.
The default button restores the actual installed `CarParams` specifications,
including any existing vehicle calibration, rather than hard-coded Traverse
values. The verified device baseline is 2090 kg curb / 2226 kg including payload
and ratio approximately 0.70. Defaults are derived per device, not copied to
unrelated vehicles. Without readable vehicle identification these controls are
hidden rather than displaying made-up defaults.

The initial zero-sentinel / integer-percent version is superseded. Legacy zero
or missing values resolve to the installed baseline, and legacy ratios 10-90
convert to 0.10-0.90. On paramsd startup, these migrations persist actual values
without replacing valid custom settings. The web also resolves old stored
values while waiting for that startup. New API/profile inputs reject zero and
integer percentages; the native ratio key is now FLOAT. Old backups with such
values must be converted before restoring. Native registry defaults of zero
remain internal migration placeholders, not UI defaults or physical values.

These are physical model inputs, not cosmetic steering-feel settings. The bounds
and ramp rates are input safeguards, not evidence that every allowed combination
is safe for a vehicle. Use measured specifications. Drivers must not operate the
web controls while driving.

## Runtime path

1. The web API and profile/backup restore service validate finite integer values
   before coercion: integer mass and finite decimal ratio.
2. `paramsd` reads both keys every 0.5 seconds. It moves toward the requested
   values at up to 100 kg/s and ratio 0.02/s. Returning to default follows
   the same ramp. Scheduler gaps cannot produce an unbounded publisher step.
3. Inertia and front/rear tire stiffness scale from the unchanged baseline,
   preserving its original stiffness calibration. `paramsd` updates the Kalman
   globals without replacing the estimated state or covariance. It clears old
   rewind history when physical constants change.
4. The applied snapshot is published in `liveParameters.vehicleDynamics`.
   `controlsd` updates its existing `VehicleModel`; `LateralPlanner` updates its
   geometry factors from that same message. Neither independently polls Params.
   Controllers, engagement checks and CAN safety policy are unchanged. Baseline
   selections restore the exact original physical constants, avoiding float32
   display-rounding drift.

The web value is the requested target, not a live indication of the intermediate
ramp value. `CarParams` remains the baseline; logs of `liveParameters` contain the
applied physics snapshot. A 0.40 to 0.70 change takes at least about 15 seconds once
read, rather than taking effect in a single control frame.

Missing, stale or invalid live snapshots leave consumers on their last accepted
constants. This does not establish fault-time continuity: message loss/recovery
or a process restart can change what the next accepted snapshot contains.
Existing validity and lateral-readiness checks remain in effect.

Old recordings without the added field retain their original `CarParams` model.
`REPLAY` does not read local tuning settings. Recomputing paramsd from an old
recording does not reconstruct historical live tuning.

## Deployment boundary

Deploy the schema, native Params registry, publisher and both consumers together
and rebuild normally. Do not selectively hot-copy only the web UI or controller.
Once this version is running, editing these two settings does not require a
restart. An unreadable key in an older native Params registry does not crash the
learner, but that is compatibility handling, not a substitute for rebuilding.

On 2026-10-20, the complete 11-file runtime change was installed as a selective
overlay on the parked/offroad target's `WIP-FR` checkout at `5378d124cb`.
Existing device-specific source differences and calibration were preserved;
unrelated pending power-policy changes were excluded. Git publication was not
performed. In this initial installation, both settings remained zero, preserving
the installed vehicle baseline. The decimal/default revision is described below.

The normal AGNOS `scons --minimal` build passed and the device rebooted
successfully. A full developer build first failed on unrelated installer font
link symbols. Native verification also caught a stale cached Params extension;
the minimal build was repeated with cache disabled and the native keys verified.
The build regenerated an existing manually installed Panda firmware artifact.
This was caught before restarting the manager; the original artifact was
restored byte-for-byte, its SCons target query passed, and its signature matched
the connected Panda before and after reboot. No Panda flash was performed.

## Verification

- 312 focused Python tests pass: web validation, restore/profile handling,
  canonical publisher, default restoration, scaling, real Cap'n Proto
  serialization, consumer methods, existing steering-ratio and engagement gates.
- 12 additional production control-path tests pass with only native Params
  isolated for the macOS test environment.
- 242 existing JavaScript settings tests pass.
- Actual Carrot Web assets were served on loopback with in-memory settings:
  saving 2090 kg and 70%, resetting to zero, and stepping 0 -> 10% -> 0 passed.
  Desktop and 390 x 844 mobile layouts were visually checked. This fixture did
  not connect to a vehicle or write real device Params.
- Target-native standalone checks passed for Params keys, Kalman global updates,
  Cap'n Proto snapshot roundtrip, vehicle-model updates and baseline restoration.
  They used isolated in-memory objects without publishing CAN or changing the
  installed calibration. The target lacks pytest, so the full Python suite was
  not run there.
- Post-reboot checks verified all 11 deployed source hashes, eight preserved
  calibration/control/power/firmware paths, offroad state, controls disallowed,
  and no required manager process stopped.
- The actual device's Carrot Web displayed both settings. Saving zero through
  its HTTP API passed; mass 499 was rejected with HTTP 400 and left zero stored.
  Actual-device screenshot: `output/playwright/live-vehicle-specs-device.jpg`.
- Private deployment evidence is retained in
  `.analysis/live-vehicle-specs-20261020/device-evidence/`; device backups and
  logs are under `/data/live-vehicle-specs-20261020-01/`.
- Native MPC numerical behavior, physical steering response and real driving
  with live edits remain unverified. Passing isolated Kalman update checks does
  not establish closed-loop stability or driving safety.

## Actual-value / decimal revision

- 323 focused Python tests pass, including native-independent Cap'n Proto
  baseline resolution, zero/percent migration, preservation of custom values,
  decimal API roundtrip, live ramping and exact baseline restoration.
- 260 web tests pass, including decimal control metadata, 0.01 stepping,
  rejection of out-of-range manual input rather than silent clamping, and
  display of 0.70 / 0.735 without percent notation.
- Selectively installed all 13 runtime files, rebuilt with the normal AGNOS
  minimal target and cache disabled, verified native FLOAT persistence and
  isolated learner/vehicle-model updates, then rebooted successfully.
  Original Panda firmware and all eight protected paths were preserved.
- Actual settings now persist 2090 kg and 0.70, with unchanged baseline physics.
  Actual web API rejects zero and old percentage input. The browser's + button
  changes 0.70 to 0.71; its default confirmation names 0.70 and restores it.
  The final values are back at the installed vehicle baseline.
- Deployment evidence for this revision is kept separately under
  `/data/live-vehicle-specs-20261020-02/`.
- This remains a local/device overlay, not a Git publication or road test.

## Integrated Recovery, October 21

The local-only publication boundary above is historical. The complete decimal
settings implementation is now included with the accumulated WIP-FR recovery.
Restore the previously verified Traverse platform baseline of 2090 kg curb
weight and 0.70 center-to-front ratio; the standard payload remains 136 kg.
The display-speed, NNFF and manual-hold changes must be deployed with it.

Local review confirms a fresh Traverse CarParams has mass 2226 kg and ratio
approximately 0.70. Stored 2090/0.70 settings preserve the exact baseline,
without a startup ramp. These values are restored project configuration, not
a new measurement or road-performance validation.

Remaining limitation: persisted settings are not keyed by vehicle fingerprint.
When moving the device to a different vehicle, review/reset both values against
that vehicle's specifications; automatic migration currently cannot distinguish
old vehicle defaults from deliberately retained user values.
