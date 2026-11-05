# Parking approach photographs

## Reason and scope

The last four recognition jobs contained three stationary exterior images each.
Direct inspection found no clearly legible nearby pillar number; the distant
marking was too small/noisy to transcribe. Saved model answers and published
analysis agreed, so no recognized text was lost in result processing.

This change adds approach views, not an inferred parking location or an app UI.
Vehicle controls, driving/DM models, Navdy and camera ownership are unchanged.

## Optional capture

The private Wayon config must enable all three of `snapshots_enabled`,
`parking_multiframe_enabled` and `parking_approach_enabled`. The new approach
flag defaults to false. Disabling it stops the child and clears its history.

- Only consume an already-running wide camera with fresh device/car state.
- Drive/reverse/low/sport, moving at 0.2 m/s through 25 km/h, thermal green and
  memory below 80% are required to start/sample.
- The low-priority child is restricted to CPUs 0-3, nice 19 and one encoder
  thread. Camera connection and JPEG work cannot block the uploader's main loop.
- Sample at most once per two seconds. Work over 100 ms backs off ten seconds.
  A stalled child is terminated; retries are bounded to once per 30 seconds.
- Retain at most 24 JPEGs, 60 seconds and 8 MiB in the ring. Each frame is at most
  1 MiB. Private tmpfs copies add up to 8 MiB plus one in-flight frame, in addition
  to the worker's decoder/runtime memory. This is not an 8 MiB total-RSS claim.
- High speed, stale input, non-green thermal status, memory at least 85%, a new
  drive or disabled config discard the session. Parent death terminates the child.
- At offroad transition, stop the child and select up to six frames: adjacent
  pairs from the beginning, middle and end. A clock discontinuity rejects them.

Long idling before switching off may age out all approach images. No new camera
request, image upload or AI inference is performed by this child while driving.

## Transport and analysis

Up to six approach images and three existing stopped images are uploaded only
under the existing fresh offroad camera guard. They carry an explicit event ID,
index/count, phase and millisecond capture timestamps derived from sensor EOF.
`parkedAt` is the last stopped
photo's capture time, not a measured shift-to-P timestamp. Approach captures older
than 120 seconds or inconsistent with the wall clock are discarded.

The matching Cloudflare patch publishes only complete, consistent groups. The
server waits two minutes, rejects partial/mixed/duplicate groups, preserves phase
and timestamps in the model prompt, and does not treat identical images as two
independent pieces of evidence. Unknown text remains unknown. Approach signs are
nearby route clues, never proof of the exact parking bay.

Deploy Cloudflare metadata support and the server collector before enabling the
new Comma flag. Disable the flag before rolling back either receiver. The old
three-photo collector path remains supported. Phone UI is not changed.

## Verification

- Host focused tests: 86 passed.
- Comma staged native Python camera/uploader/policy/capture tests: 322 passed.
- Production-baseline workerd + D1 + KV integration tests: 20 passed.
- Windows server's actual isolated venv: 50 tests passed, real schema validator.
- Comma saved-image encoder probe: 25 native 1344x760 NV12 conversions, warm
  median 73.710 ms, warm maximum 80.403 ms; first conversion 103.601 ms. Probe peak
  RSS was 74,328 KiB including its extra test arrays, not a live-worker plateau.
- Isolated launcher test read actual offroad state, exited without capturing,
  removed its temporary spool and passed the offroad guard before/after.
- Final review added invalid-config cleanup and crash/stale/high-speed rejection.
  A native probe also exposed normal offroad card shutdown; fresh offroad device
  state now closes existing history without authorizing any new capture.

These are source, synthetic-input and offroad-device checks. Actual moving
capture, concurrent 20 Hz model performance, next-parking image coverage and
recognition accuracy still require a real subsequent session. More photos cannot
recover a sign that is out of view, blurred or too small to read.
