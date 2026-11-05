# Wayon-style C4 stop timers

Display-only change on WIP-FR, based on `00abd46e4aeefe7b3b1c65ebd5aaff700e048149`.
Reference: local Wayon/sunnypilot `da8cda353ec2a16bcc6d3d870f54137b809af3ff`,
`selfdrive/ui/mici/onroad/alert_renderer.py` and `status_timers.py`.

## Behavior and adaptation

- While onroad and stationary, a fresh `carState.parkingBrake` displays the
  original red parking icon and elapsed MM:SS, also when there is no alert.
- Actual `brakeHoldActive` or `softHoldActive == 2` displays the original green
  hold icon and elapsed MM:SS. Soft-hold preparation (1) does not claim active hold.
  Stock cruise standstill also tracks elapsed time behind other warnings, but
  displays it only with the existing `resumeRequired/warning`.
  This adapts Wayon's presentation to Carrot's actual state fields: Traverse
  currently suppresses `cruiseState.standstill`, so copying only the original
  resumeRequired branch would miss its manual auto-hold.
- No new event, engagement, braking, auto-hold, audio or model behavior.
  Existing manual hold, Bluetooth/phone reconnection and offroad UI remain intact.
- Other selected alerts, including unnamed startup/communication errors, take
  priority. Parking replacement is restricted to no-alert/parking no-entry;
  user-disable and critical alerts are not replaced. Hold replacement is limited
  to no-alert or the three existing hold warnings with a matching active state.
- Monotonic elapsed time continues behind other warnings. Offroad, a new onroad
  session, fresh movement or release resets it. Invalid/stale inputs hide it
  immediately; at most one second of missing samples can retain the start time.
- Reference dimensions: 74px icon, 62px time, 18px gap; hold has a 150px dark
  gradient. Zero letter spacing matches the current font renderer. Very long
  elapsed times shrink to leave the persistent side BSD indicators unobscured.
- While a stop timer is visible, the overlapping Carrot speed panel, cruise
  animation, wheel-side ancillary text and optional VISION info card are hidden.
  Steering warning icon, traffic signal/countdown, eGPU/Jetlink state, DM and both
  BSD strips are retained. The signal/countdown uses the free upper-left area;
  ordinary clock/date/debug text shares the suppressed wheel-side area.
  Normal content returns on release, movement, data loss or a different alert;
  the HUD cache key includes this transition so no stale panel is reused.

## Typography

All 108 reference JSON entries exist in the current 109-entry table. Every
shared `titleFontPx` and `subtitleFontPx` matches; no table rewrite is needed.
Existing exact DM mappings, critical escalation wording, diagnostic details and
bounded fit-to-screen remain. The timer has its own label so it cannot corrupt
the ordinary alert font/layout cache. This does not add unsupported Sunny events.

## Validation

- Pure transition and real renderer-method tests cover timing, fresh inputs,
  warning priority, session resets, actual Carrot state mapping, font restoration,
  cache repaint and persistent BSD indicators.
- Actual native timer widget is rendered in 536x240 and 476x240 content areas,
  both icons, at 01:23, 101:43 and 4320:00; single-line/bounds assertions.
- Existing alert typography/policy, offroad transition and overlay tests are
  included in the regression batch. Mac needs `--noconftest -o addopts=`;
  the overlay state test needs native Params and is verified separately on C4.
- Native desktop timer images are isolated-widget previews, not full camera/HUD
  screenshots or optical C4 panel validation. No CAN/control publisher, forced
  onroad or physical brake actuation is used. Actual vehicle timer transitions
  and full optical panel appearance still require a parked user check.
