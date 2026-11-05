# Navdy signed-speed display dropout

## Verified Evidence

- USB diagnostics were collected through the dedicated ADB server while the
  display bridge remained in `bluetooth_only` mode. No USB telemetry was sent.
- Installed HUD APK matches the v164 signed artifact exactly:
  `9ad7dc0ff175d6de2d0a1fb09d01f06188971d2555a9269d2737b6d97c9af429`.
  Main HUD PID was unchanged since the prior deliberate installation.
- The persistent ambient journal shows repeated known-gear/unknown transitions
  during the latest retained driving session. Its reverse-OFF commands receive
  successful module protocol responses. Recent door-close fade/OFF is normal.
- Native recorded-state analysis: 629 carState samples, all message-valid and
  CAN-valid, none CAN-timeout; maximum logged carState gap 103 ms. The GM adapter
  intentionally signs wheel speeds, then applies its speed filter. There are
  169 negative filtered samples, including 65 with standstill true and 64 with
  raw speed exactly zero. Minimum is -1.2078565 m/s during reverse movement;
  stopped values oscillate down to tiny negative floating-point magnitudes.

The Navdy display mapper incorrectly required `vEgo >= 0` to accept *all*
vehicle state. Negative values therefore erased gear, availability and set
speed despite valid, fresh CAN data. This is separate from Bluetooth expiry.
Device wall clocks changed during boot, and the Android/Comma clocks are not
proven synchronized: no exact subsecond cross-device causal timing is claimed.

## Correction

Accept finite signed speeds with absolute magnitude below 100 m/s. Missing,
malformed, nonfinite and out-of-bound values remain invalid. Keep existing CAN
validity, timeout and service-age checks, immediate disengagement, reverse
suppression, and nonnegative displayed-speed sanitization. No vehicle-control,
model, Panda, Bluetooth TTL, queue, pairing or ambient policy changes.

## Verification

- New regression tests fail in five cases before the correction. They exercise
  sign crossings in D/N/P/R, reverse suppression and invalid/boundary inputs.
- Bridge integration also verifies zero crossing does not bypass subsequent
  expiry or CAN-invalid rejection.
- Native Comma execution of both pure mappers on all 629 recorded structs:
  460 outputs identical; differences confined to the 169 negative-speed frames.
  Unknown gear counts fall from 173 to the five actual recorded unknowns;
  gear transitions fall from 45 to nine. Reverse active visuals remain disabled.
- This replay publishes no messages and sends no commands to the car or HUD.
- Local focused suite: 571 passed; five native DBC cases excluded on macOS.
  Existing cpp pytest configuration warnings are unrelated.
- Ruff passes for the mapper and its tests. Five existing E701/E702 findings
  in the bridge test file reproduce identically at the base revision; the new
  integration test adds none. `git diff --check` passes.
- Independent source review found no actionable defect in the correction.

## Remaining Ambient / Link Uncertainty

This is **not** a claim that every ambient OFF or radio interruption is solved.
With fresh device/panda onroad state the bridge still refreshes ambient state,
even when speed rejection had hidden widgets. The old generic `output blocked`
journal does not distinguish reverse, vehicle timeout and shutdown. Unknown
gear preserves a reverse latch; it does not independently request OFF/ON.
Current offroad Bluetooth stability cannot establish driving stability.

Existing bounded bridge diagnostics retain future source/receiver transitions
and link failures. Preserve those and the USB journal rather than extending
stale-state TTL or disabling the ambient watchdog. Raw device logs/APK and
recorded frames remain private, outside this repository.

## Deployment

Published single-parent correction `b0ddb09b` to WIP-FR and fast-forwarded the
matching Comma after repeated fresh offroad/ignition-OFF/no-output guards.
Native post-install pure mapping checks passed all 31 cases. Restarted only
navdy_bridge, PID 74265 -> 113733. No whole-device reboot or APK installation.
HUD PID 12405 and UI/pandad/hardwared/carrot_man/jetlinkd PIDs are unchanged.
Generated DBC edits, N16 pin and Bluetooth configuration were preserved.

The deployment script's final comparison mistakenly compared complete Android
`ps` rows (which include changing memory values). It failed after installation
and successful reconnection. A separate read-only verification compared stable
process identity and passed; the deployment was not repeated.

Final 90.088-second offroad observation sampled 450 distinct bridge updates:

- Bluetooth session 41 unchanged; all display acknowledgements accepted.
- Receiver clears 207 -> 207, no new link failures, skipped snapshots zero.
- USB telemetry updates remain zero while USB diagnostic reads succeed.
- Maximum sampled RTT 134.68 ms, receiver age 134.01 ms.
- Runtime mapper SHA256:
  `7cfcfba931320f96b4c0f7b71f5da16de185715a48dc7c626277d6650974c91a`.

Backup: `/data/navdy/backups/signed-speed-20261022-46foay4d`.
Private verification output: `/tmp/hl-navdy-usb-20261022/verify-result.txt`.
This is offroad protocol/process validation, not optical HUD, physical ambient
or driving validation. Independent ambient interruption remains unresolved.
