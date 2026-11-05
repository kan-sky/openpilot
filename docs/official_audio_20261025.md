# Official openpilot audio refresh

Historical record: superseded on 2026-10-26 by the user's
[complete Wayon tone replacement](wayon_audio_20261026.md). The old manifest
and waveform/duration assertions below describe the previous installation,
not the current assets.

Source: [commaai/openpilot master at 9fba666fd4](https://github.com/commaai/openpilot/tree/9fba666fd485983b5849dfc62550fa1fd37fe338/openpilot/selfdrive/assets/sounds),
checked on 2026-10-25. This is an audio-only import, not an upstream model,
vehicle-control, event-schema, or sound-player update.

## Mapping

Keep this branch's existing filenames so device soundd, web sound, language
selection, and tizi aliases all continue to resolve without changing events.

| Existing asset | Official source |
| --- | --- |
| engage.wav, engage_tizi.wav | engage.wav |
| disengage.wav, disengage_tizi.wav | disengage.wav |
| refuse.wav | refuse.wav |
| prompt.wav | warning.wav |
| prompt_distracted.wav | dm_warning.wav |
| warning_soft.wav | critical.wav |
| warning_immediate.wav | dm_critical.wav |

All nine names in sounds, sounds_eng, and sounds_chs are updated: 27 files,
using seven exact upstream WAV payloads. The accompanying official_audio.json
pins the source commit, LFS SHA256, byte size, frame count, and mapping.
Downloads were checked against their commit-pinned Git LFS pointers before
replacement. Actual WAV bytes are committed, not LFS pointer text.

The remaining 87 WAV files are unchanged, including Korean/English/Chinese
spoken prompts, countdowns, auto-hold, traffic, NNFF, and long-control sounds.
No files are deleted. Upstream pre_alert/critical_max/dm_critical_max are not
imported because this branch does not select those new assets. Importing their
new events, volume ramps, and eight-second escalation is outside this request.

## Behavior Preserved

No production Python or JavaScript changes. Existing alert triggers, repeat
counts, user volume, DM minimum/final gain protection, timeout behavior,
navigation announcements, and phone/Bluetooth policies stay unchanged.

The waveform and clip duration do change. prompt.wav is now 0.750 seconds
(previously about 1.506 seconds), including its shared radarCutin/systemReady
one-shot uses. The reboot chime also shares this clip and retains its 0.150-second
silent lead-in (0.900 seconds total). critical and dm_critical clips are about 0.536 seconds and keep
the existing indefinite-repeat policy. Tizi now uses the same latest engage
and disengage recordings as the official player.

## Verification

- Asset-only tests check all 27 copies against upstream hashes, valid nonempty
  mono 16-bit 48 kHz PCM, all existing sound routes, and both tizi aliases.
- The import inventory confirms all 87 non-target WAV hashes unchanged.
- The soundd one-shot and reboot-chime test expectations are updated to the new
  exact duration. Asset/reboot tests: 80 passed on macOS; Ruff passes. Production
  code is unchanged.
- Before installation, Comma's actual soundd decoder passed 105 alert/language
  variants, 1,126,284 official PCM sample comparisons, 15 one-shot checks and
  48 DM-gain/repeat checks in an isolated process without opening audio output.
- macOS cannot directly collect the native messaging-dependent soundd suite
  in this checkout (`msgq.ipc_pyx` is unavailable). Actual loader/PCM checks on
  Comma must be reported separately from the asset-only tests and from physical
  speaker/listening validation. Do not infer acoustic loudness from gain values.
