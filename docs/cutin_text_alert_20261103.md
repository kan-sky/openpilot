# Cut-in Text Notice

The existing `radarCutin` event now shows two lines for two seconds:

- `끼어드는 차량 감지`
- `전방 차량을 확인하세요`

It remains a warning-only informational event, with the existing `radarCutin`
one-shot sound (`prompt.wav`). It does not enable or disable control, change
cut-in detection, select leads, or alter braking/steering. Existing candidate
promotion and duplicate suppression remain unchanged. The notice has lower
priority than driver-monitoring and collision warnings.

The C4 presentation uses a 48 px title and 30 px subtitle. Real font-metric
tests verify one line per label in both 476 px and 536 px text layouts.

## Verification

The selected presentation, Mici layout, audio asset/player, and cut-in tracker
tests pass: 356 tests and 1,217 subtests on the desktop. The sound test repeatedly
provides the event for two seconds at 20 Hz and verifies one chime followed by
silence, rather than repeated playback. Event tests preserve safety priorities
and reject any no-entry or disable event classification.

Native installation verification additionally checks the actual event import,
AlertManager expiry and FCW preemption, and UTF-8 Cap'n Proto serialization in
isolated Params/IPC without publishing messages. These checks do not establish
physical screen/speaker output or real-driving cut-in detection performance.

The runtime change is only `events.py` plus the Mici presentation JSON. Other
pending changes, model files, controls, settings and vehicle firmware must not
be included in deployment. Preserve unrelated device edits, require fresh
offroad/ignition-off state, back up the affected files, fast-forward to the exact
published commit, and verify the new process generation after a normal restart.
