# Pearl driver-monitoring eyes (2026-10-19)

The user selected the pearl cartoon-eye design and requested implementation for
both C3 and C4. The shared `DriverPreview` now draws two warm pearl eyes
(`#f4eddf`), dark pupils and small highlights directly on the existing onroad
view. The old camera rectangle, border and camera image are removed. Existing
C3/C3X and compact C4 placement remains unchanged; the widget is display-only.

## Data and presentation

- Existing `driverStateV2` left/right driver selection follows `isRHD`.
  Changing sides discards the previous expression immediately.
- Pupil offsets are a bounded, mirrored illustration of `faceOrientation`,
  smoothed with an 80 ms time constant. The model does not publish ocular gaze
  coordinates: this must not be described as precise eye tracking. Invalid or
  uncertain head pose centers the pupils.
- Each eyelid follows its own `leftBlinkProb` / `rightBlinkProb`, with rapid
  closing and softened reopening. There are no random or periodic decorative
  blinks in the production UI. Blink display requires finite probabilities,
  eye confidence above 0.65 and sunglasses probability below 0.9. These are
  presentation gates only; monitoring thresholds are not modified.
- Unknown eyes are gray short marks without pupils. `DM EYES?` distinguishes
  unreliable eyes from a tracked face; an individual unknown eye stays gray.
  Face loss is `DM FACE?`, stale/invalid data is `DM CHECK`. `DM FACE` means
  face recognition, not a claim of attention or safety.
- Existing distracted/lockout states use warm warning color and eyebrows with
  `DM ALERT` / `DM LIMIT`. Existing interaction monitoring/camera fallback is
  a steering-wheel symbol with `DM HANDS`, not simulated live eyes.
- Existing valid/alive/current-session/500 ms message checks remain. A model
  frame ID that makes no progress for 500 ms also clears the expression even
  if the same model frame is republished with a new message timestamp.
- Alerts, offroad and ordinary disengagement hide the character and reset its
  expression. `AlwaysOnDM` visibility is preserved. Closing is idempotent.

`DriverPreview` is now a plain `Widget`: it creates no driver-camera VisionIPC
client, camera shader, EGL image or video texture. The real monitoring model and
camera continue operating as before. Offroad camera setup/preview is unchanged.
There are no Params, model/runtime, process scheduling, monitoring policy,
engagement, control or sound changes.

## Upstream validation (historical)

The following results were recorded by upstream commit `d029bf3e`; they are not
new native validation of the October 21 backport.

75 focused tests pass, including both C3 and C4 widget sizes, real cereal list
readers, head-pose bounds, independent blink/confidence gates, RHD selection,
stalled-frame recovery, freshness boundaries, warning priority, alert hiding,
camera-free construction and resource-free close. The focused UI/sound/HUD
regression run passes 180 tests, with the native IPC timeout case deselected on
Windows. Desktop adapters replace OS/IPC/Params and widget hosting, not the eye
presentation or tested sound logic. Ruff and whitespace checks pass.

Desktop Raylib renders exercise the actual production eye renderer for front,
left, closed eyes, lost face, warning, wheel fallback, stale data and alert
hiding at 2160 x 1080 and 536 x 240. Surrounding HUD elements are schematic;
the DM inputs are synthetic. Local render evidence and the reproduction script
are under `.analysis/dm-eyes-design/` and are not distributed.

This validates desktop layout and state handling, not physical C3/C4 display
readability, real blink/pose alignment, native device CPU savings or road use.
Branch publication is separate from installation on a vehicle. No vehicle
processes have been restarted or files installed for this change.

## Bounded backport (2026-10-21)

Backported onto `52782e1397efedabfd6db7a17c34371d098a5b2f` in the shared
`publish-wip-fr-nav/beta` worktree. The two production eye modules are copied
verbatim from `d029bf3e`. Existing C3/C4 call sites already use this shared
preview, so no parent view changes are required. The Wayon phone renderer in
`mici/onroad/driver_state.py`, its regression tests and all driver-monitoring
policies remain untouched. There are no controls, settings, model or device
changes, and no commit or push is included in this backport.

The audit Python environment has no `pyray`; before the port, the focused
preview/Wayon run had 45 passes and 13 fixture errors from that missing module.
The imported preview tests now use test-only Raylib value types and record the
production draw calls, with widget hosting, text measurement and UI state also
stubbed as in upstream. Real cereal readers are still exercised. This checks
state transitions and draw geometry, not native rendering or font metrics.
Additional cases cover long draw gaps, independent unknown/closed eyes, and
read-only phone/distraction/lockout packets without changing engagement.

The final focused run passes 160 tests: 87 preview tests, eight unchanged Wayon
driver-state tests, 17 DM alert tests and 48 monitoring context/cadence tests.
The two production modules match `d029bf3e` byte-for-byte. A baseline diff also
confirms no changes to either driver-state module, the Wayon driver-state tests,
the monitoring directory or `soundd.py`. Scoped whitespace checks pass.

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=.:third_party/jetlink:opendbc_repo \
  /tmp/hl-wip-fr-audit-20261021/bin/python -m pytest \
  --noconftest -p no:cacheprovider --rootdir=. -c /dev/null \
  openpilot/selfdrive/ui/tests/test_dm_preview.py \
  openpilot/selfdrive/ui/tests/test_wayon_driver_state.py \
  openpilot/selfdrive/monitoring/test_dm_alerts.py \
  openpilot/selfdrive/monitoring/test_dm2_context.py \
  openpilot/selfdrive/monitoring/test_dm2_cadence.py -q
```

The additional `test_dm2.py` / `test_monitoring.py` policy run cannot collect
because `setproctitle` is missing. Ruff is also unavailable in the audit
environment and on PATH. These are not counted as passed checks; no packages
were installed or shared test environment files changed for this backport.

No native Raylib window, IPC, physical C3/C4 display, real blink/pose alignment,
native CPU savings or road behavior is validated here. No device access was
performed.
