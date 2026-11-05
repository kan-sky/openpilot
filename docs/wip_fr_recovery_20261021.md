# WIP-FR Integrated Recovery

## Confirmed Regression

A read-only audit found the device still on `5378d124`, with a reflog reset at
2026-10-20 17:27:21 KST. The initiator of that reset is unproven.
Several earlier installed tracked files matched that old commit exactly:
Traverse NNFF substitution, manual-hold/controller/safety integration,
dashboard speed conversion, Navdy Bluetooth/lifecycle/flicker integration,
Jetson USB Internet discovery and Carrot Vision host-only ICE.
New untracked helper files alone did not activate their missing callers.

The power repair and live vehicle-specification overlay survived. The signed
Panda artifact and its last reported signature matched the verified manual-hold
firmware, although its safety source had reverted. Do not infer working manual
hold from a firmware signature or NNFF from an enabled setting alone.

## Recovery Contract

Deploy a single committed fast-forward of WIP-FR, including the complete live
vehicle-specification implementation and the restored Traverse baseline.
Preserve current settings, generated models, pairing data and unrelated files.
Back up and separately retain the old tracked overlay and only the untracked
files that conflict with the incoming commit.

Stop the manager only after fresh offroad, no-output, ignition-off telemetry.
The current bench device has no vehicle harness and about 5 V power. Back up
native artifacts before the minimal build. Preserve the already-flashed audited
Panda artifact only when the restored safety source matches its known source;
verify its actual signature without sending control commands.

Remove the old overlay activation flags so an older finalized checkout cannot
replace the restored source at reboot. Keep automatic Git pull disabled as
already configured. Verify the actual branch, hashes, native registry, model
lookup, generated safety flags, settings and required processes after reboot.

## Local Verification

- 340 GM display/manual-hold tests pass, including native desktop C safety
  hooks and the restored physical-baseline regression.
- Independent review: 83 dynamics/API tests, 143 NNFF/control tests with
  146 subtests, and 245 web-settings tests pass.
- 230 Navdy/Jetson tests pass on macOS; five additional DBC tests require the
  missing native Params module and are deferred to target-native validation.
- Physical steering, stationary brake holding, Navdy optical output and
  Jetson hardware operation are not established by these tests.

Private deployment evidence is kept under
`.analysis/regression-audit-20261021/`; device backups and execution logs are
kept under `/data/wip-fr-recovery-20261021/`.
