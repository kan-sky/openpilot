# Traverse Experimental Trailblazer NNFF Substitute

## Requested Behavior

The user requested that the existing Carrot Web NNFF toggle select the shipped
Trailblazer neural feedforward model on their Traverse. This is an explicit,
experimental cross-vehicle substitution, not a claim that either EPS response
or real-vehicle steering stability has been validated.

The installed WIP-FR catalog had 117 models and no Traverse model. The normal
Traverse lookup returned None with NNFF already saved as 1. Neither comma's
native neural feedforward table nor the GM nonlinear table covered Traverse.

## Implementation

- Only CHEVROLET_TRAVERSE may use CHEVROLET_TRAILBLAZER.json as a substitute.
- A valid native Traverse model takes precedence if one is later added.
- Substitution is opt-in at the loader and allowed by the interface only when
  the existing NNFF Param is enabled. NNFFLite remains independent and unchanged.
- The donor filename must exist exactly. No fuzzy donor guessing or matching
  to a different donor firmware variant is introduced.
- Empty catalogs return no model rather than testing membership in None.
- The startup model label is CHEVROLET TRAILBLAZER (substitute).
- Carrot Web's Korean/English descriptions identify this as an unvalidated
  experimental substitute, with next-onroad/restart application semantics.
- Vehicle fingerprint, geometry, torque tune, longitudinal settings, control
  code and Panda safety/firmware are not replaced by Trailblazer settings.

Donor SHA256:
`2a8dfbfb840e76e4711e59a646dcce7550772b31139c94d06e75edb05b7c5301`.
The existing 18-input/1-output model file is unchanged.

## Verification

`opendbc_repo/opendbc/car/tests/test_nnff_substitute.py` contains 18 unittest
cases. All passed on the Comma against staged source and again against installed
source after reboot, with no skipped cases.

Coverage includes ON/OFF, explicit/absent donor, empty catalog, native model
precedence, no unrelated substitute, unknown EPS firmware, unchanged selections
for the 117 existing model filenames, comma-native precedence and NNFFLite
preservation. Twenty-seven synthetic full-length inputs produce finite outputs.
An isolated GM interface produces identical complete CarParams dictionaries
with the toggle off/on, including the Traverse fingerprint and safety settings.

The real LatControlTorque class is constructed using a CarParams reader, then
stepped with synthetic inputs: active with model data calls NNFF at least three
times; inactive outputs zero without NNFF evaluation; missing model data takes
the existing non-NNFF fallback. Outputs remain finite and within the existing
normalized torque bound. An initial test fixture incorrectly passed a builder
instead of the controller's normal reader; correcting the fixture resolved that
test-only error without changing production controller code.

All test Params are in-memory, and CAN parsing/transmission is stubbed out.
There is no model/control IPC publication or physical steering command.
These are execution/contract tests, not real-log accuracy or closed-loop vehicle
validation. Finite/clamped outputs do not establish correct steering response.

## Device Deployment

Applied to comma-db5ce68d, WIP-FR HEAD
`5378d124cb836b654611c6c4e21094b61e187774`. Before stopping manager, fresh
device/Panda/peripheral telemetry showed offroad, ignition false, controls
disallowed, noOutput, no faults and 11.792 V. After manager cleanup, a direct
Panda health read still showed ignition/controls false, no faults and 11.814 V.

Only two production files were installed:

- `opendbc_repo/opendbc/car/interfaces.py` SHA256
  `8ba245f526c14d39bc53ef0b0bdd0f5fa49f4e9944ae70988c1ceff3602a4b01`.
- `openpilot/selfdrive/carrot_settings.json` SHA256
  `68b5cd2f2aa79fffc69eecc7b57d67071058386d835751a3bca6f61d863bfbd4`.

Backup: `/data/traverse-nnff-20261020-01/backup`. With updater stopped and the
repository update lock held, old overlay markers were saved there to prevent
a stale staged tree replacing the selective installation. Automatic Git pull
was off. Twenty-six other modified/protected files, including the power fix,
Navdy settings, controller, donor model and Panda firmware, retained their hashes.
No Git commit/push, branch switch, firmware flash or network change was performed.

New boot: `0dd49bf1-00c6-4f8e-a4fe-4bb3654bc4bf`. Postboot hashes, branch and
HEAD matched. Across 60.03 seconds / 12 samples, UI PID 53384, hardwared PID
53392 and pandad PID 53388 stayed unchanged. Telemetry remained fresh, valid,
offroad and fault-free with controls disallowed; observed voltage was
11.638-11.913 V.

The live `/api/settings` response contained the new NNFF descriptions. Under
fresh offroad/noOutput guards, actual `/api/param_set` requests set NNFF to 0
and then 1. The installed interface initializer read the real saved value:
0 loaded no donor; 1 loaded the donor. The original NNFF=1 was restored, and
these two setting changes are intentionally recorded in web setting history.
NNFFLite=0 and the 4320-minute power timer were preserved.

Private evidence is under
`.analysis/traverse-nnff-deploy-20261020/device-evidence/`.
No forced onroad session, physical steering, road test or visible startup
notification check was performed. The next normal onroad initialization uses
the saved toggle; actual vehicle suitability remains unverified.
