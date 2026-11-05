# C4 Wayon-style offroad presentation

## Source and scope

Reference: local Wayon/sunnypilot `da8cda353ec2a16bcc6d3d870f54137b809af3ff`,
`selfdrive/ui/mici/layouts/home.py`, `main.py`, and `ui_state.py`.
The original checkout was read only; its unrelated dirty changes were preserved.
Target: carrotpilot WIP-FR, based on `55b8d251e7`.

- Before a drive: `안녕하세요!` / `안전한 주행 되세요`.
- After a drive: `수고하셨습니다`, distance (km) and duration, retained in memory
  for ten minutes. A new drive clears the previous summary. UI restart loses it.
- Screen still sleeps after the existing 30-second offroad timeout. The summary
  does not hold the screen or vehicle power on for ten minutes.
- Wayon smoothstep black overlays: 0.55-second surface transition, 0.85-second
  wake, and two seconds fading out **before** the existing sleep deadline.
  Existing 2.5-second onroad delay is not shortened. Moving/engaged onroad
  presentation bypasses the blackout.
- Welcome tap opens settings. The previous home (version, IP, network, web,
  experimental-mode long press) remains the next offroad swipe page, unchanged.
  Active offroad alerts and onboarding retain precedence. Plot, HUD-camera
  preference, recording commands and bookmark callback are retained.
  As in Wayon, tapping the road view no longer opens the welcome page. Plot
  selection remains available through ShowPlotMode; onroad/debug are no longer
  manual offroad-swipe pages. Moving/engaged plot changes also skip the blackout.

## Door and trip inputs

Device wakefulness polls the existing DoorMonitor's atomic
`/dev/shm/params/d/OffroadDoorOpen` output. Only a new closed-to-open edge while
both ignition and started are false resets the display timeout. A file already
present when the UI starts is a baseline; a first file created afterward can
wake it. Missing/malformed samples never reset a previously observed door edge.
An open door does not keep the screen awake forever. This uses the existing GM
offroad CAN support, not an inference that every vehicle supports door wake.
No CAN sender, new subscriber, process, Params key or BT behavior was added.

Like Wayon, distance integrates cluster speed with vEgo fallback, but only for
fresh, valid current-session CAN samples. Stale samples and gaps over one second
add no distance, including at shutdown. Duration uses monotonic elapsed time.
Consequently this is a UI session summary, not an odometer/route-log substitute.
Long summary values shrink within their columns rather than being truncated.

Power-off thresholds, Panda, controls, driving/DM models, Navdy/phone Bluetooth,
reconnection, ambient policy and sound assets are untouched.

## Verification

- Focused pure-state and real-handler tests cover door baseline/edge/errors,
  wake/sleep cancellation, fade boundaries, stale data, trip reset/expiry,
  short ignition cycles, plot exit, moving-onroad blackout cancellation.
- `preview_offroad.py` renders the actual native widget at logical 536x240,
  including long values and half/fully black fades, without vehicle publishers.
- Desktop home/debug imports require unavailable native msgq/Params extensions;
  do not describe their collection failure as a full-suite pass.
- Physical door-to-panel timing and a real post-drive distance comparison require
  vehicle observation. Native desktop output is not physical panel evidence.
