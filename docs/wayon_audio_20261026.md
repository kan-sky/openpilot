# Wayon audio replacement

Source: `leehyuk1108/Wayon`, `codex/my-traverse-source` at
`da8cda353ec2a16bcc6d3d870f54137b809af3ff`, inspected on 2026-10-26.
The local source is `~/Documents/sunnypilot/selfdrive/assets/sounds/`.
Every imported file was compared byte-for-byte with its pinned Git blob,
not just the possibly modified working tree. The source root MIT license
is identical to this repository's retained `LICENSE` (Comma.ai, Inc., 2018).

The user explicitly selected non-verbal Wayon tones for **all** existing
sounds, including Carrot-only countdown, lane-change and auto-hold speech.
This supersedes the partial official-audio refresh from 2026-10-25.
No Wayon event, monitoring, vehicle-control or volume policy is imported.

## Mapping

Existing filenames remain compatible with soundd, web playback, NNFF notices
and reboot audio. All 114 previous WAVs are replaced, with 12 additional copies
to make the three language directories identical: 42 names per directory,
126 files total, from 11 unmodified Wayon WAVs. No old spoken WAV remains in
`sounds`, `sounds_eng` or `sounds_chs`. Changing language still works but no
longer changes spoken wording because these assets are all non-verbal.

| Target(s) | Wayon source |
| --- | --- |
| All 11 original Wayon filenames | Same named source, including distinct tizi engage/disengage |
| `audio_stopstop.wav` | `warning_immediate.wav` |
| `audio_engage.wav`, `tici_engaged.wav` | `engage.wav` |
| `audio_disengage.wav`, `tici_disengaged.wav` | `disengage.wav` |
| `audio_traffic_error.wav` | `refuse.wav` |
| `audio_car_watchout.wav`, `audio_speed_down.wav`, `radar_cutin.wav`, `Wazealert.wav`, `Wazealert2.wav` | `prompt.wav` |
| `traffic_sign_green.wav`, `nnff.wav` | `prompt_single_high.wav` |
| `audio_auto_hold.wav`, `audio_stopping.wav`, `audio_turn.wav`, `audio_turn2.wav`, `audio_lane_change.wav`, `audio_lanechange.wav`, `reverse_gear.wav`, `traffic_sign_changed.wav` | `prompt_single_low.wav` |
| `audio_1.wav` through `audio_10.wav` | `prompt_single_low.wav` |

The countdown now gives a tone rather than speaking the remaining number.
The zero-count event still selects its existing long-disengage sound. Its
trigger/reset timing has not changed. `stopStop`, including FCW/AEB users,
now plays the urgent Wayon tone, not a spoken "stop stop" phrase.

`openpilot/selfdrive/assets/sounds/wayon_audio.json` records the complete
mapping, source commit, SHA256, byte lengths and PCM properties. Every file
is a nonempty mono PCM16/48 kHz WAV, not a Git LFS pointer.

## Playback and preserved behavior

- Event conditions, priorities, repeat counts, timeout warnings, countdown
  dispatch, language selection and alert interruption rules are unchanged.
- Automatic volume (`SoundVolumeAdjust=-1`), engagement volume and DM stage
  2/3 gain floors remain unchanged. Waveform amplitude and duration do change;
  unchanged software gain does not imply identical acoustic loudness.
- `trafficStopping` remains silent with the existing two-line screen event.
  Replacing `audio_stopping.wav` does not reconnect it to that event.
- Wayon `prompt.wav` lasts 0.356 seconds; the reboot helper retains its
  0.150-second silent lead-in (0.506 seconds total).
- Web playback revalidates each WAV on its first use per page instead of
  forcing an old HTTP cache entry. The existing decoded-buffer cache remains,
  so repeated alerts do not repeatedly download. Reload an already open web
  page after deployment; an existing page's decoded buffers are not hot-swapped.
- A pre-existing PCM buffer-boundary bug was reproduced during verification:
  the cursor and completed-loop count were not advanced inside a callback,
  so the tail could repeat beyond a one-shot's end or corrupt a loop boundary.
  Advance these two counters at the boundary without changing repeat policy,
  gain, alert selection or scheduling. No Wayon player implementation is copied.

## Verification scope

The asset suite checks every target hash, complete three-language coverage,
WAV decoding, production C3/C4 asset loading and exact gain-scaled PCM.
It runs the production loader/callback methods in an isolated namespace,
without opening audio hardware, creating native Params or publishing IPC.
4096-frame callback blocks cover three clip lengths for every mapped alert,
including exact one-shot silence and continuous repeat boundaries. Real-wave
DM minimum gains and urgent interruption are also checked.

The accompanying volume, reboot and alert-presentation regressions check that
automatic gain and the silent stopping event survive this replacement.
This is local code/PCM validation, not speaker listening, road validation or
proof of installation. Git publication and device deployment are separate.

Local verification on 2026-10-26: 509 Python tests and 1,212 subtests passed;
three pre-existing missing-plugin configuration warnings remain. New asset
tests execute the production methods without native messaging imports; the
device-only `test_soundd.py` suite is not counted as a native device pass.
Ruff and diff checks pass. All soundd AST outside the four-line PCM boundary
correction is identical to the baseline, including gains and dispatch logic.
Seven web sound/volume tests and the web build also pass, including cache
revalidation, decoded-buffer reuse and recovery after a failed asset fetch.
