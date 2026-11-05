# WIP-FR Publication Audit

## Confirmed State

The device was authenticated directly on the local network. It was running
WIP-FR at `7c280cbaad8b51bd1ec60a8ef80ebdb66fb66b24`. GitHub WIP-FR was still
`a4b7fe3c5553c03ca960daba4846bb9834051269`: six N16-related commits had not been
pushed. The navigation page was also installed as two modified tracked files
and three untracked runtime files. This was a publication omission.

The audit compared 142 files with the intended source, including the recent
Traverse speed, manual-hold, NNFF, power, vehicle-dynamics, Navdy, Jetson and web
changes. All 142 matched. The current reflog has the N16 fast-forwards and no
subsequent reset. This does not reproduce the earlier October 20 rollback.

Mapbox key presence, Navdy navigation selection, the openpilot/longitudinal
toggles, vehicle specifications and the device-local N16 selection file were
preserved. No credential values are included here. Fresh device/Panda messages
reported offroad, ignition off and no-output safety. No inference/control
process was started, no settings were changed and no reboot was requested.

At inspection Jetlink reported waiting and USB enumeration contained no NVIDIA
host. Installed N16 files are not evidence of a presently connected Jetson.
This audit does not establish live N16 inference or physical Navdy output.

## Publication Scope

Publish the six existing commits and the already-installed navigation UI/API
with its tests. Preserve the committed live Jetson model reporting; do not
publish the unrelated local reverse diff that restores a hardcoded Cinque
label. Preserve all original worktree changes in their existing checkout.

The navigation API supports bounded address search and destination management.
It does not change the selected navigation source. Navdy/external destinations
cannot be overwritten by web requests. The optional map-to-N16 input pipeline
is still explicitly `not_connected`; token setup/search is not NoO integration.
An uncommitted experimental navigation backend is excluded from this release.

The longitudinal regression expectation includes the already-implemented
MANUAL_AUTO_HOLD safety flag. This is a test-only correction, not a new change
to the controller or flashed firmware.

## Verification

- N16/protocol/navigation/server/settings desktop run: 682 pass, 20 skip.
  Skips require unavailable native modules or platform-specific facilities.
- Additional power/Navdy/vehicle-specification/GM/USB checks: 685 pass.
  Five real-DBC ambient cases cannot import native `params_pyx` on this Mac;
  an initial attempt fails on that missing dependency, then those five cases
  are explicitly deselected. They are not counted as passes.
- The separate NNFF substitute suite also cannot collect without native
  `params_pyx`; matching deployed source is not a new runtime NNFF validation.
- Web navigation, live model status and vehicle-specification tests: 41 pass.
- A disposable Git fixture verifies that staging only identical deployed
  files permits a normal fast-forward while preserving unrelated files.
- `git diff --check` passes.

The focused tests do not establish road safety, actuator response, sustained
20 Hz inference, native Panda firmware operation or physical display output.

## Device Reconciliation

Before reconciling Git metadata, require the expected device/branch/HEAD, fresh
offroad/no-output telemetry, exact matching runtime file contents and an empty
existing index. Back up the five deployed navigation files and original index.
Use a normal fast-forward only; do not reset, clean or replace runtime files.
Verify unchanged settings/runtime hashes and the running web-server PID.

A separate finalized update at `a4b7fe3c` still has its ready marker. Launcher
installation also depends on `.overlay_init` and newer Git metadata, so the
ready marker alone does not prove it will replace the current checkout. This
old candidate must be invalidated with a backup before any future reboot;
do not remove the whole staging directory or assume a live rollback occurred.

Private audit evidence is kept outside the repository. No Mapbox token,
pairing data, device settings snapshot or generated native/model artifact is
published.
