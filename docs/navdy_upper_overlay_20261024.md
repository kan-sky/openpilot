# Navdy upper lane/vehicle overlay (2026-10-24)

## Distinguish this incident from radio stalls

The user reports only the engaged upper lanes/vehicles disappearing, or lanes
remaining without vehicles. In the retained latest-drive journal interval,
Bluetooth session 553 is constant with no exchange failures or receiver-clear
increments. This does not resolve the separate lower-level delivery stalls
described in `navdy_radio_stall_20261024.md`; those still recur offroad.

The installed v169 APK's `OpenpilotPathView.updatePayload` requires all four
inner-lane/path arrays. An empty required array clears the entire upper view,
including its vehicle array. The scalar speed/ACC fields are updated separately.
Ordinary lane/car drawing has no independent short-lived rendering timer.

## Confirmed display defects and changes

1. `_model_points` formerly discarded an entire path if any later x coordinate
   stopped increasing. Actual valid model output has a far trajectory tail
   turning back: one example reaches 22.0129 m and then returns to 21.8587 m,
   while current lane arrays remain valid. That makes the bridge send empty
   required arrays and the native upper renderer hide itself.
2. Lead and surrounding-track display required the time-based predicted path
   to extend to each vehicle. A short stopping trajectory excluded vehicles
   whose positions and lane boundaries were otherwise current and supported.

Keep only the current frame's contiguous increasing path prefix. Stop at an
invalid/non-increasing sample; do not sort the tail, bridge invalid gaps, cache
old geometry, or extrapolate. Preserve the old treatment of sub-millimeter
negative origin jitter only while the accepted x remains within 1 mm of the
origin and lateral displacement from that point is at most 1 mm. Three retained
otherwise-valid standstill samples require this narrow exception. A missing or
fully degenerate path still fails closed.

Beyond current path coverage, vehicles must be supported by current lane
boundaries/probabilities and use the same existing no-path projection as lanes.
Do not substitute a lane-center reference. Existing width fallback still needs
a supported path. Keep confirmation, radar errors, 350 ms track freshness,
500 ms service freshness, range/count bounds, duplicate suppression and
disengagement clearing. This only changes Navdy display mapping, not model
output, steering, braking, navigation feedback, Bluetooth or phone reconnect.

## Recorded-input verification

Replay uses retained rlogs, sorted by producer monotonic timestamp, carrying
only display-relevant fields. Sample active state at approximately 5 Hz and
inactive state at 1 Hz. These are not the exact historical bridge ticks or an
optical recording. Raw logs, APK and replay snapshots remain private.

| Check | Before | After |
| --- | ---: | ---: |
| All samples / active samples | 1,756 / 1,240 | same |
| Active samples missing required geometry | 410 | 0 |
| Active samples with at least one vehicle | 228 | 683 |
| Additional vehicles while old geometry was already valid | - | 88 samples |
| Previously valid geometry changed (including far lanes/edges) | - | 0 |

All non-`nav*` fields and all inactive payloads compare identically. The replay
finds 482 samples with more vehicles, not 482 independent physical cars.
Compressed display-request size including protocol header averages 697 to
839 bytes; maximum is 1,034 bytes in both versions. This is a payload estimate,
not measured radio capacity, ACK overhead or a Bluetooth-stability guarantee.
The original 5 Hz schedule, two-in-flight bound and eight-vehicle limit remain.

Parent regression suite: 970 tests passed; five existing native DBC cases are
excluded on Mac because its native Params binding is unavailable. Ruff and
diff checks pass. New cases cover folded/equal tails, invalid gaps, tiny origin
jitter, lane/vehicle alignment at 10/30/51/52/70/80 m, unsupported lanes,
duplicates, stale inputs and preserved scalar fields. Final source commit is
`16036146c13eff2860ac3d85176d49c3c069e3b7`.

## Verification limits

### Actual installed renderer

The parent executed an isolated helper as Android shell uid 2000 on Navdy,
using the unchanged installed APK and its 13 vehicle bitmap resources. Six
recorded old/new payloads passed native synchronous visibility/count/pixel
checks: folded-path GONE becomes VISIBLE, the surrounding-vehicle sample
changes zero to one car, and the lead sample changes zero to two. Parent
inspected PNGs and verified their hashes. HUD/phone service PIDs, APK and
pairing/configuration were unchanged; no new HUD start appeared in the logcat
diff. Bluetooth remained the data owner. Claude reviewed the exact production
diff, helper, actual renderer references and guarded runner before execution.

This is a separate unattached 320x100 software canvas with no Looper loop,
not a production screen recording or animation-timing test. No live receiver
or simulated vehicle publication is used. Printed isolation flags are contract
constants; process/hash/log checks are the independent evidence. An initial
argument-order error stopped before helper class loading. Moving `--nice-name`
after `/system/bin` succeeded without source changes.

### Runtime scope

This fixes demonstrated current-input display rejection. It does not guarantee
that every detected vehicle is visible: normal confidence/range/freshness gates
remain. A completely unusable current model still clears geometry. No stale
object retention is introduced to conceal radio loss. Physical onroad viewing
and the separate phone-connected radio-stall investigation remain distinct.

The existing projection can still shift laterally beyond approximately 51.4 m
when changing path coverage toggles its path-aware formula. Lanes and cars
remain aligned; this is not an empty-array clear. A synthetic 70 m comparison
reproduces the shared shift, but this drive has no corresponding optical
evidence. Do not broaden this fix into an unvalidated projection redesign.

## Device application

Pushed to `traverse/WIP-FR` and fast-forwarded the identified Comma after fresh
offroad/ignition-OFF/inactive guards. All changed tracked files match the commit;
existing generated DBC modifications and untracked build/model files remain.
Only the bridge restarted (834976 to 912103); carrot_man remained 741567.
The Navdy APK and both pairing/configuration hashes stayed unchanged.

The device has no pytest. Rather than install packages into the running system,
four native Python geometry smoke checks passed. The five Mac-excluded native
DBC cases were also executed directly from their unchanged AST function bodies,
omitting only pytest decorators and using the original three-row parameter
matrix. All five passed with in-memory CAN packing/parsing, no CAN publication.
This is distinct from a pytest run or real vehicle door/control validation.

The first post-restart 60 seconds had 298/298 connected samples, session 579
unchanged, no receiver-clear increase and fresh applied sequences. A later
check reached session 581 after additional offroad exchange failures, then
reconnected. Thus this display fix explicitly does NOT resolve radio stalls.
Automatic persistent diagnostics remain enabled. The isolated native helper
exited; its own temporary compiled/image files were collected and removed,
restoring Navdy free storage to the pretest 196.2 MiB. No production app restart
or phone policy change was made.
