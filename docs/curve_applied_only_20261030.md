# Applied-only curve badges

## Contract

The October 30 user request supersedes the waiting/unused-complete behavior
for VTURN and ROUTE in `c4_deceleration_phases_20261029.md` and
`navdy_deceleration_phases_20261029.md`. Other reasons retain their phases.

- Filter VTURN/ROUTE before choosing the single displayed reason unless the
  existing `is_applied` check confirms that this source owns the current cap.
  Continuous curvature calculation alone is not a displayed event.
- Applying an unfinished cap: orange/white blinking at the existing 1 Hz.
- Applied cap reaches its own known endpoint: green after the existing 300 ms
  confirmation, with orange during confirmation instead of ready-white.
- Releasing the curve cap, overriding it, disengaging or losing valid/fresh
  data hides it. There is no remembered completion after release.
- Unknown curve endpoints may blink while applied, but never imply completion.
- Filtering before selection allows a pending CAM or other valid reason to
  remain visible even when an unapplied curve has a lower candidate speed.

Actual vehicle speed is not a completion input. These are display states of
the published speed constraint, not proof of physical braking or compliance.
No producer, curvature calculation/filter, planner, vehicle control, protocol,
packet rate, Bluetooth, phone reconnection, ambient or layout changes.

## October 30 Local Verification

- Updated C4/shared verdict tests failed in 47 cases before the production
  change, including the straight-road-like held-cap oscillation.
- 467 focused producer/schema/shared/C4/Navdy/equivalence tests pass. The
  existing equivalence suite includes 3,200 synthetic producer steps against
  its pre-badge baseline; existing calculation fields remain identical.
- Another 70 curve-calculation/display tests and 64 bridge tests pass.
- C4 production drawing calls for 200 samples per curve source stay empty
  while the unapplied cap is 175.442 and its endpoint alternates known/unknown.
  Applied blinking, completion, release and pending-camera priority pass.
- Ruff on changed Python files and `git diff --check` pass.
- Navdy production Java helper builds into an API21-compatible DEX035 APK;
  648 host assertions and 42 historical encoder compatibility fixtures pass.
  These fixtures also retain compatibility with older waiting codes; the new
  sender no longer emits waiting codes for curves.
- New unsigned Navdy artifact:
  `~/Documents/navdy/hud_patches/v173-deceleration-badge/build/curve-applied-only-20261030/Hud-v173-deceleration-badge-aligned-unsigned.apk`
  SHA256: `a183767c7b4643265ad07dae6320f5ad771f2c6672ea22f21f4a5fce0a50b3c4`.
  Only `classes2.dex` differs from the prior v173 unsigned APK; all 7,061 other
  ZIP entries are byte-identical. Canonical verification preserves original
  transport methods and bounds the existing receiver hooks.

Desktop producer tests stub unavailable Params/msgq/hardware I/O; C4 uses a
drawing recorder and Navdy host tests use graphics shims. An initial attempt
to run the broader bridge suite through that producer harness failed because
its hardware stub lacks `hardware.hw`; the unchanged bridge suite passes all
64 tests without that harness. This was a test-environment issue, not a reason
to modify production bridge code.

The October 30 implementation was local only: no installation, signing or
publication was performed then. This explains why a later reboot still loaded
the old display behavior; it was not evidence of a rollback.

## Publication Checks

On November 1, the four Python files were rechecked against WIP-FR
`420bdc03d0b272d226206b254da2bf88e31d6e36` before publication. The installed
Comma still had that commit and lacked the applied-only filter. The desktop
suites pass 601 tests: 344 verdict/UI/source checks, 145 producer/curve/
equivalence checks and 112 bridge/display checks. Ruff and diff checks pass.
The real Comma's old code reproduces 47 affected verdict/draw failures in an
isolated process. One additional relocated-test failure is a fixture path
error, not a BSM regression; installation checks must run from the normal
repository paths. No synthetic state was sent to live vehicle services.

The sender filter is sufficient to clear unapplied VTURN/ROUTE on the existing
Navdy APK (`decel=0`); a phone or Navdy update is not required for that fix.
The separate, uninstalled Java candidate described above only changes the
300 ms completed-curve confirmation color from white to orange. Do not claim
that APK is installed from the Comma change. Native probes/installers pinned
to the older v173 APK remain historical. Neither source/unit checks nor an
offroad process restart establish physical HUD or road validation.
