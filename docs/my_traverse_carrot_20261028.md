# My Traverse Next on Carrot

## Scope

Port the existing authenticated Wayon Cloud integration to WIP-FR, preserving
the phone's separate GMOne vehicle controls, widgets, timers and background
services. No steering, braking, model, Panda or Navdy Bluetooth policy changes.
The device must already have its private `/data/wayon_cloud/config.json`.
No enrollment, public camera endpoint or automatic SSH authorization is added.

## Feature Paths

| Feature | Carrot integration | Verification |
| --- | --- | --- |
| Vehicle and Comma status | Bounded telemetry, qualified power provenance, fresh/stale distinction | Fresh authenticated cloud timestamps and phone rendering |
| GPS and trips | Existing GPS sources plus completed route qlogs | Actual short and long recorded routes; device-scoped cloud list/detail |
| Driving analysis | Automation, comfort, longitudinal, monitoring, system, timeline and events | Actual native schemas; partial coverage is explicit |
| Event blackbox | Bounded signal archive and playable road MP4, existing packaged ffmpeg | Real recorded video remux and three-frame native decode; no synthetic incident uploaded |
| Panoramic live | Wide and driver H264 streams, phone WebGL panorama | Actual 1344x760 streams and phone display |
| Live save | Photo and 10/30-second dual-stream clips | Cloud upload and phone gallery/photo/clip playback |
| Parking photos | Guarded paired capture, four-second exposure warmup | Actual paired capture and upload |
| Impact and door events | Persistent bounded queues, retry and existing cloud push contract | Native tests and live armed sensor sampling; physical events not triggered |
| Ambient manual/auto/profile | Existing Navdy override contract, validation and cloud acknowledgment | Actual phone AUTO command acknowledged; physical light application not claimed |
| GMOne controls and widgets | Existing independent phone/server path retained | Source/contracts and phone status only; no engine/lock/window actuation |

## Camera Ownership

- Only fresh offroad/ignition-off/controls-disabled states permit camera work.
- Reject driver view, high memory use, unsafe thermal state and stale state.
- An exclusive lease and three-second owner/PID marker coordinate the requests.
- The manager owns the camera processes; old ownership must finish before the
  normal onroad camera starts. Remote capture does not hold `IsTakingSnapshot`,
  change `UseWideCamera` or prevent the normal ignition transition.
- CPU power saving is suspended only while a fresh camera request exists.
- Panorama has a bounded session, heartbeat, archive buffer and control queue.
- Carrot's producer leaves the VisionIPC `valid` member unset. Photos check
  positive finite EOF age <=0.5 seconds instead; model validity is unchanged.

## Data Honesty

Offroad telemetry uses a slower heartbeat than onroad. The phone freshness rule
matches that cadence without presenting missing checks as healthy. A 5V device
input is not presented as the vehicle battery terminal voltage.

Analysis uses sampled qlogs. Scores are heuristics, not a diagnosis of actual
driving safety. Missing `longitudinalPlanSP` context is marked unsupported for
unexplained-braking assessment. Work budgets can yield a partial report; its
observed interval, read/unread segments and partial reason are exposed.

Blackbox MP4s retain complete selected source segments and seek to the requested
window. Timing is approximate nominal segment alignment, not frame-accurate
synchronization. Missing/oversized/corrupt media is reported rather than being
labelled playable. A Comma ambient acknowledgment means settings were stored;
it does not prove Navdy or the physical LEDs applied them.

## Cloud and Client

The production Cloudflare Worker differed from the dirty local source. Preserve
all production bindings/modules and patch only authorized trip union/detail,
health-cache behavior and relay lifecycle. The deployed worker SHA256 is
`67eccbf0151a79e428244b0e68bc9d800e79215c87d11d64e0b75442188633e8`.
The relay selects only OPEN peers, ignores replaced senders and handles late
close callbacks without closing the new session. Auth and live/SSH namespaces
remain unchanged. Retained-CLOSING mock reproduction demonstrates a possible
failure path, not the established cause of the original interruption.
No key, production settings, GPS coordinates, images or driving logs belong in Git.

My Traverse Next keeps its package and original signing certificate, and is
updated with `adb install -r`. Its binary WLP1 heartbeat runs every three seconds.
Reconnection clears stale decoder readiness and cancels old photo jobs; late
results from an old connection cannot be sent over a new connection.

## Evidence and Limits

Final native full candidate: 506 tests passed, including packaged-ffmpeg
checks. Android release unit tests: 60 passed; client JavaScript: 46 passed;
scoped Cloudflare trip/cache regressions: 19 passed; relay regressions: 14 passed.
The exact relay class also passed real workerd/Miniflare checks for the OPEN
constant, binary heartbeat, 1.1 MB payload, server-side close callbacks,
viewer/device replacement, ambient notification, reopen and device loss.
Forced hibernation and concurrent upgrades were not tested.
The phone is updated to version 30 (`2.0-preview.21-carrot-live`) without
uninstalling or clearing data. Claude reviewed source and failure cases, while
native/device execution was performed separately.

The final 70-second native observation received 1,400 wide and 1,401 driver
frames, spanning 69.94 and 69.95 seconds. This is encoder delivery, not optical or
network frame rate. The phone rendered the panoramic view, then reopening it
worked again. Closing the view stopped both camera and encoder processes.
The actual phone trip-detail page rendered the new analysis and its moments.

The three-segment real route parsed completely in about three seconds. A
93-segment recorded route reached the 30-second work limit after 33 complete
segments and was correctly partial. It is not a full long-drive analysis claim.

Actual live photo upload was 1,081,321 bytes; 10.35-second and 30.25-second clips
were 3,936,509 and 11,443,606 bytes. Both saved clips replayed on the phone.
The private packaging probe used real recorded qcamera/log files and a clearly
synthetic test window, never uploaded as a detected incident. Native remux
produced a 526x330 H264 MP4, and a three-frame rawvideo decode succeeded.

Not verified physically: ignition handover, road behavior, an actual impact,
door transition push, and ambient LED output. Remote engine/door/window actions
were intentionally not issued. Initial live reconnection failures were observed;
bounded recovery regressions and subsequent device checks are not an overnight
or mobile-network stability guarantee.

## Final Capture and Recovery Correction

A later version-29 photo attempt exposed another defect: generating the JPEG
took about 0.32 seconds, but its 871 KB WebSocket message reached the relay only
after the heartbeat timeout. Head-of-line blocking is plausible, not proven as
the network root cause. A separate confirmed defect kept the Cloud WebSocket
open after local stream EOF, leaving the phone displaying frozen LIVE video.

Version 30 sends photos directly to the existing authenticated HTTPS capture
endpoint, leaving the live WebSocket for small controls and heartbeat. CORS
preflight was verified (204, POST and authorization/content-type allowed).
The server derives device identity from the key, supplies receipt time/layout
defaults and returns the saved ID. Only that acknowledgment is shown as saved.
Both cameras must be freshly decoded at capture and upload start. Uploads have
a 20-second abort bound; an interrupted upload may still have committed server
side, so an automatic retry is not issued.

Video older than three seconds disables saves and shows delayed state. Stalled
video triggers bounded recovery without resetting the six-retry budget merely
because the server reports LIVE. Local EOF now closes only the owning live
relay session, with replacement/late-close protection. The 12-second device
heartbeat, ignition/thermal/memory guards and SSH policy are unchanged.

Actual version-30 phone tests saved two photos (first 944,074 bytes, upload ACK
in 1.17 seconds), kept the live session across both saves, and opened the new
photo from the Cloud gallery. A later temporary video delay was correctly
marked and recovered; no uninterrupted-network guarantee is made. A separate
real Cloud/device test deliberately omitted viewer heartbeat: the viewer got
1012 after 16.42 seconds including camera startup; a new session's first message
arrived in 0.77 seconds, and explicit close released camera ownership. Closing
the final phone view left no camera/encoder process or active lease.
