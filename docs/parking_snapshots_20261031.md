# High-quality offroad parking snapshots

## Scope

The existing guarded offroad snapshot path now preserves native wide-camera
resolution up to 1928 pixels without upscaling, and encodes ordinary wide
snapshots as JPEG Q92, 4:4:4. Driver snapshots remain 960 pixels maximum/Q72.
Existing brightness correction, camera ownership, ignition/state freshness,
memory/thermal checks, bounded capture timeout and final lease release remain.
No driving, DM, navigation, Navdy or app UI changes are included.

Enable `parking_multiframe_enabled: true` in the private
`/data/wayon_cloud/config.json` to capture three exterior frames at least one
second apart on the first successful snapshot after uploader startup and after
each onroad-to-offroad transition. Subsequent periodic snapshots are single
high-quality frames. The normal configured snapshot interval is unchanged.
Driver is uploaded once per burst. All capture requests are released before
image upload; a fresh guard is checked before encoding and before each POST.

The existing Cloudflare snapshot API stores the original photos. The separate
ParkingVision server consumes only complete three-frame wide bursts and uses
the owner's official Codex ChatGPT login to transcribe visible signs. It never
uses driver images or infers a floor/bay from an unlabelled number. Results are
private machine-read clues, not an exact parking-location claim.

## Verification

- On Comma, staged source plus existing camera/uploader tests: **266 passed**.
  New cases cover native width/no upscaling, three fresh spaced frames, aborted
  bursts on ignition/state change, lease release, Q92/4:4:4 encoding, legacy
  UTC `...Z` timestamps, one-shot cadence and upload-time guard rejection.
- Real guarded capture uploaded three native **1344 x 760** images with capture
  times one second apart. JPEG sizes **361741 / 351492 / 340275 bytes**, luma
  quantization table first value 3 and all sampling factors 1x1 confirm Q92/444.
  End of capture/upload: request released, camera not busy, elapsed 31.9 seconds
  including cloud transfer. This is not a camera startup-latency benchmark.
- The private server's scheduled S4U task automatically fetched that batch,
  completed OAuth inference and produced **zero tool actions**. With no legible
  parking sign in the current view, the result was empty, not a guessed number.
- A separate six-frame recorded approach clip also completed through OAuth.
  It read a directional `301` sign, not proof of the parked floor or bay.
- Claude performed three static review rounds. Timestamp compatibility,
  boundary deduplication, per-group failure isolation and periodic-analysis
  overuse were corrected. Tests and native execution were run by Codex.

Private images/results stay out of Git. Server code and operating instructions
are in Wayon `cloudflare/wayon-cloud/parking/README.md`.

## Limitations and rollback

The current burst is stationary/offroad, not a pre-parking moving frame buffer.
Better encoding cannot reveal signs outside the image. Three frames from one
view are not independent viewpoints. Grouping relies on capture times rather
than an explicit cloud burst ID; clock jumps/incomplete uploads may skip a job.
Extra burst photos use existing gallery entries and more cloud storage.

Setting `parking_multiframe_enabled` false disables bursts while preserving the
quality upgrade. Reverting these two scoped source files restores the old photo
quality. The uploader is configured `spawn=True`, so restarting only its managed
process loads fresh camera code; a manager or whole-device reboot is unnecessary.
Recheck the offroad guard and camera ownership before any restart. Production
configuration and source must be backed up before deployment.
