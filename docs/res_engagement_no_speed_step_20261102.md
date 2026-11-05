# RES engagement without an extra speed step

## Cause

GM's `CarState.update_button_enable()` requests non-PCM engagement on the
RES/+ **press** edge. `VCruiseCarrot._prepare_buttons()` emits the ordinary
speed adjustment on **release**, or periodically during a long hold. By that
time `carControl.enabled` can already be true. Checking only the release-time
enabled state treats the engagement gesture as a second speed-up request.

On the unchanged installed revision `71092d09`, isolated tests reproduce
80 -> 85 with the configured 5 km/h basic step, and 80 -> 90 in other button
modes or after a long hold. The initial 39 new cases yield 35 failures and
4 passes; the failures include delayed feedback and stored-speed selection.

## Scope

- Latch only a physical RES/+ press that arrives while disabled and is marked
  `buttonEnable` by the existing vehicle interface.
- Run the existing resume-speed selection once on that press. Existing saved
  speed restoration and first-use current-speed initialization remain intact.
- Consume speed adjustments from the rest of that same hold/release, even if
  engagement succeeds, fails, or is subsequently cancelled.
- A separate press after engagement retains the configured short/long step.
- Leave release-triggered SET, dedicated SET/RES buttons, VW speed-only buttons,
  Bluetooth gestures, gas-tap behavior and engagement permission unchanged.
- Do not modify CAN, Panda, steering, acceleration limits, or braking logic.

## Verification

The candidate passes 419 selected native tests: cruise-button handling, gas
tap and pedal-release behavior, navigation SET speed, dedicated VW buttons,
Bluetooth gates, longitudinal settings, VCruiseHelper and Toyota PCM mode 0.
Tests include actual GM `update_button_enable()` plus full
`VCruiseCarrot.update_v_cruise()` with in-memory Params and input messages.
Longitudinal engagement and pedal operations are not sent to the vehicle.

Runs use a distinct `OpenpilotPrefix`, offroad/ignition checks and a separate
candidate directory. Source selection is asserted before importing tests.
Sixteen unrelated legacy simulator tests are excluded: their existing caller
omits the required `carrot` planner argument, already reproduced on the prior
unmodified revision. This is not a claim that the entire repository suite passes.

Evidence and deployment receipts are retained privately under
`~/Documents/res-button-20261102/` and `/data/res-button-20261102/`.
Actual physical RES operation and road behavior require separate validation.
