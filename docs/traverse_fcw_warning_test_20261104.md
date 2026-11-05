# Traverse FCW warning diagnostic

Private WIP-FR diagnostic, not a public/general vehicle setting. Not installed or
physically validated. This is a forward-collision-warning request, NOT a proven
seat-only vibration command and NOT an emergency-braking test.

## UI and scope

- Carrot settings: System / Sound, after engagement sound volume.
- `FcwWarningTest`: 1 requests up to two seconds, 0 cancels only the test.
- The controller must acknowledge before the API reports ON. The row rereads
  actual transient state and returns to OFF. A profile, backup restore or undo
  cannot activate it; it is not saved to Params or included in fingerprints.
- Use the comma's local-network IP. Existing trusted-LAN, exact-origin and
  no-forwarded-request checks apply; this is not exposed through a cloud proxy
  or the Tailscale address. This does not add internet authentication to Carrot.

## Controller boundaries

Only the Traverse camera/SDGM openpilot-longitudinal configuration instantiates
the diagnostic. Every controller update requires valid non-timeout CAN, P/D/L,
finite nonnegative speed, no gas, no stock ACC, and all openpilot
enable/lateral/longitudinal flags false. Parking brake and standstill are NOT
required. Moving manual-driving tests are permitted. A 500 ms continuous
qualification precedes acceptance; a controller gap over 150 ms cancels it.

A tmpfs request is tied to a random controller-instance token and must arrive
within 250 ms. Status older than 500 ms cannot authorize a request. A restart,
unsafe transition or completion invalidates the token. Maximum pulse is two
seconds with five seconds cooldown. Repeated ON cannot extend the pulse.
Malformed requests and file failures fail closed. Normal FCW remains ORed with
the test; cancelling never overrides a genuine host FCW.

The existing dashboard message cadence and packer are reused. No independent
CAN sender, safety allowlist change, steering/braking/acceleration changes,
forced engagement or forced onroad mode are introduced.

This supersedes the initial parked-only draft following the user's clarification
that the requested test is an FCW warning, not a brake command. Moving tests
require a controlled test area and a passenger operating the UI, never the
driver. The original incident establishes received stock FCW, not independent
seat actuation or complete ECU behavior. A parked ECU may ignore the request;
even a moving request does not establish physical warning behavior until tested.

## Verification

- 547 local Python tests pass: timing/guards/restart/replay, HTTP restrictions,
  settings compatibility, existing GM display speed and manual auto-hold tests.
- The moving D/L readiness tests fail on the initial parked-only implementation
  and pass after this revision. Lease, cancellation, restart and failure cases
  run in both stopped and moving state; stopping and moving again never extends
  the test deadline.
- Whole-controller comparison with the real CAN packer at rest and 54.5 km/h
  changes only the
  dashboard FCW field during the test. All other frames and actuator outputs
  match the baseline; an existing FCW produces identical complete output. The
  comparison covers timeout, explicit cancellation, enable/lateral/longitudinal
  engagement and CAN timeout, including restoration of ordinary dashboard output.
- 25 JavaScript tests pass: actual page commit/poll functions and existing
  settings controls/dialogs. These are DOM-stub/VM tests, not a physical screen.
- `git diff --check` passes. Existing unrelated local changes are preserved.

No vehicle CAN was transmitted, no device service was restarted, no firmware
was changed, and no install/push was performed for this diagnostic. Desktop
encoding/guard tests do not prove physical warning behavior or vehicle safety.
