# C4 commanded-stop timer

## Scope

WIP-FR base: `297372d01720d2799d5a58fed71dc646d25c15c8`.
This is a display-only change. It does not change events, sounds, CAN state,
steering availability, braking, driver monitoring, or Navdy transport.

The GM parser deliberately clears `cruiseState.standstill` for Traverse.
Recorded stopped longitudinal control consequently had none of the three
signals previously accepted by the timer. A persistent `belowSteerSpeed`
alert also prevented the timer from being displayed.

## Display Contract

Keep existing parking, brake-hold, soft-hold and stock-resume paths. Additionally
recognize a commanded stop only when all of these are true:

- `carState`, `selfdriveState`, and `carControl` are valid, alive, from this
  onroad session, and no more than 250 ms old with no future timestamp.
- The vehicle reports standstill and absolute speed at most 0.1 m/s.
- The gear is drive, sport, low, eco, or manumatic; neither pedal is pressed.
- Selfdrive is enabled and active; car control is enabled with longitudinal active.
- Longitudinal control is `stopping` with a finite negative acceleration command.

A confirmed hold at standstill may replace only the low-speed steering notice
or an empty alert. Existing hold/resume alerts remain eligible. Other alerts,
critical conditions and communication errors retain priority. Moving or losing
the commanded-hold conditions hides this additional timer. Short invalid gaps
retain the existing clock policy without displaying stale state.

The timer reflects the commanded stop, not an independent measurement of brake
force. It must not be used to infer physical brake performance. Low-speed
steering availability and event/audio generation remain unchanged.

## Verification

- Before the fix, four new positive reproduction tests failed and 85 passed.
- After the fix, 408 focused UI/alert tests and 1,212 subtests pass, including
  real Cap'n Proto inputs, enum comparisons, sound-only alert transitions,
  invalid/stale input, pedal override, gear, moving state, and other warnings.
- Ruff passes for changed code; the pre-existing `draw_texture` TID251 violation
  in the alert renderer is excluded. Whitespace checks pass.
- Private recorded route `00000196--ae79e6abb2`, segments 2-4, was replayed
  offline through the old and new timer methods at 20 Hz UI ticks. The 5,623
  recorded service messages produce 3,748 evaluated UI frames.
- The old timer appears on 78 frames; the new one appears on 1,136. The additional
  1,058 frames are the commanded stop previously hidden by `belowSteerSpeed`.
  This run lasts 52.85 seconds without timer interruption. The original 78
  hold frames remain visible. No new timer appears while moving, and no
  input message is modified.

Raw logs and replay results stay outside Git. This is source and recorded-input
verification, not a deployed UI, optical display, braking, or road test.
