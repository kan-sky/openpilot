"""Asset checks need no native messaging modules or audio output device."""
import array
import ast
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace
import wave

import numpy as np
import pytest

from openpilot.cereal import car


ASSETS = Path(__file__).resolve().parents[2] / 'assets'
SOUNDD = ASSETS.parent / 'ui/soundd.py'
MANIFEST = json.loads((ASSETS / 'sounds/wayon_audio.json').read_text())
AudibleAlert = car.CarControl.HUDControl.AudibleAlert


def test_pinned_upstream_and_complete_language_mapping():
  assert MANIFEST['commit'] == 'da8cda353ec2a16bcc6d3d870f54137b809af3ff'
  assert MANIFEST['repository'] == 'https://github.com/leehyuk1108/Wayon'
  assert set(MANIFEST['directories']) == {'sounds', 'sounds_eng', 'sounds_chs'}
  assert len(MANIFEST['mapping']) == 42
  assert len(MANIFEST['sources']) == 11
  for source in MANIFEST['sources']:
    assert MANIFEST['mapping'][source] == source
  assert MANIFEST['mapping']['audio_stopstop.wav'] == 'warning_immediate.wav'
  assert MANIFEST['mapping']['audio_auto_hold.wav'] == 'prompt_single_low.wav'
  assert MANIFEST['mapping']['audio_lane_change.wav'] == 'prompt_single_low.wav'
  for number in range(1, 11):
    assert MANIFEST['mapping'][f'audio_{number}.wav'] == 'prompt_single_low.wav'
  assert set(MANIFEST['mapping'].values()) == set(MANIFEST['sources'])


@pytest.mark.parametrize('directory', MANIFEST['directories'])
@pytest.mark.parametrize('target,source', MANIFEST['mapping'].items())
def test_all_copies_match_pinned_wayon_payloads(directory, target, source):
  data = (ASSETS / directory / target).read_bytes()
  spec = MANIFEST['sources'][source]
  assert data[:4] == b'RIFF'
  assert len(data) == spec['bytes']
  assert hashlib.sha256(data).hexdigest() == spec['sha256']


@pytest.mark.parametrize('directory', MANIFEST['directories'])
@pytest.mark.parametrize('target,source', MANIFEST['mapping'].items())
def test_payloads_decode_as_nonempty_48khz_mono_pcm16(directory, target, source):
  spec = MANIFEST['sources'][source]
  with wave.open(str(ASSETS / directory / target), 'rb') as sound:
    assert sound.getcomptype() == 'NONE'
    assert sound.getnchannels() == 1
    assert sound.getsampwidth() == 2
    assert sound.getframerate() == 48000
    assert sound.getnframes() == spec['frames']
    pcm = sound.readframes(sound.getnframes())
  assert len(pcm) == spec['frames'] * 2
  assert max(abs(value) for value in array.array('h', pcm)) > 0


@pytest.mark.parametrize('directory', MANIFEST['directories'])
def test_all_existing_sound_routes_resolve_without_generic_fallback(directory):
  tree = ast.parse(SOUNDD.read_text())
  names = {node.elts[0].value for node in ast.walk(tree) if isinstance(node, ast.Tuple)
           and node.elts and isinstance(node.elts[0], ast.Constant)
           and isinstance(node.elts[0].value, str) and node.elts[0].value.endswith('.wav')}
  assert names.issubset(MANIFEST['mapping'])
  assert {p.name for p in (ASSETS / directory).glob('*.wav')} == set(MANIFEST['mapping'])
  for name in names:
    assert (ASSETS / directory / name).is_file(), name


@pytest.fixture(params=[(device, lang, gain) for device in ('tici', 'tizi') for lang in ('ko', 'en', 'zh-CHS') for gain in (35, 100)])
def player(request):
  device, lang, engage_gain = request.param
  namespace = {
    'np': np, 'os': os, 'wave': wave, 'AudibleAlert': AudibleAlert,
    'BASEDIR': str(ASSETS.parents[2]),
    'HARDWARE': SimpleNamespace(get_device_type=lambda: device),
    'Params': lambda: SimpleNamespace(get_int=lambda key: engage_gain),
    'cloudlog': SimpleNamespace(error=lambda message: pytest.fail(message), warning=lambda message: pytest.fail(message)),
  }
  tree = ast.parse(SOUNDD.read_text())
  constants = {'SAMPLE_RATE', 'MAX_VOLUME', 'MIN_VOLUME', 'AMBIENT_DB', 'DB_SCALE', 'VOLUME_BASE'}
  functions = {'sound_asset_dir_for_language', 'resolve_sound_path', 'linear_resample', 'dm_warning_volume'}
  # Execute the production asset table, hardware override and methods without
  # importing native IPC, creating Params, or opening a speaker stream.
  nodes = [n for n in tree.body if (
    isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in constants for t in n.targets) or
    isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name) and n.target.id == 'sound_list' or
    isinstance(n, ast.If) and 'HARDWARE.get_device_type' in ast.unparse(n.test) or
    isinstance(n, ast.FunctionDef) and n.name in functions
  )]
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Soundd')
  methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
             {'load_sounds', 'get_sound_data', 'callback', 'update_alert'}]
  exec(compile(ast.Module(body=nodes + methods, type_ignores=[]), str(SOUNDD), 'exec'), namespace)
  obj = type('AssetSoundd', (), {n.name: namespace[n.name] for n in methods})()
  obj.lang = lang
  obj.current_alert = AudibleAlert.none
  obj.current_alert_type = ''
  obj.current_volume = .2
  obj.current_sound_frame = 0
  obj.load_sounds()
  return obj, namespace['sound_list'], device, lang, engage_gain


def test_production_decoder_and_hardware_specific_engage(player):
  obj, routes, device, lang, gain = player
  suffix = '_tizi' if device == 'tizi' else ''
  assert routes[AudibleAlert.engage] == (f'engage{suffix}.wav', 1, gain / 100.)
  assert routes[AudibleAlert.disengage] == (f'disengage{suffix}.wav', 1, gain / 100.)
  assert routes[AudibleAlert.stopStop][:2] == ('audio_stopstop.wav', None)
  directory = {'ko': 'sounds', 'en': 'sounds_eng', 'zh-CHS': 'sounds_chs'}[lang]
  assert set(obj.loaded_sounds) == set(routes)
  for alert, (filename, _, volume) in routes.items():
    with wave.open(str(ASSETS / directory / filename), 'rb') as sound:
      pcm = np.frombuffer(sound.readframes(sound.getnframes()), dtype='<i2')
    expected = (pcm * volume).astype(np.float32) / 32768.
    np.testing.assert_array_equal(obj.loaded_sounds[alert], expected)


def test_production_callback_copies_full_waveforms_across_buffer_boundaries(player):
  obj, routes, *_ = player
  block = 4096
  for alert, (_, repeats, _) in routes.items():
    obj.update_alert(alert)
    obj.current_sound_frame = 0
    samples = obj.loaded_sounds[alert]
    length = len(samples)
    count = (3 * length + block - 1) // block
    outputs = []
    for _ in range(count):
      output = np.full((block, 1), np.nan, dtype=np.float32)
      obj.callback(output, block, None, None)
      outputs.append(output[:, 0])
    actual = np.concatenate(outputs)
    expected = np.resize(samples, len(actual))
    if repeats is not None:
      expected[length * repeats:] = 0
    np.testing.assert_allclose(actual, expected * .2, err_msg=f'alert={alert}')


@pytest.mark.parametrize('stage,gain', [(2, .7), (3, 1.)])
def test_new_dm_waveforms_keep_minimum_gain_and_silence_has_no_audio(player, stage, gain):
  obj, *_ = player
  obj.current_volume = 0
  alert = AudibleAlert.promptDistracted if stage == 2 else AudibleAlert.warningImmediate
  obj.update_alert(alert, f'driverDistracted{stage}/permanent')
  samples = obj.loaded_sounds[alert]
  np.testing.assert_allclose(obj.get_sound_data(len(samples)), samples * gain)
  obj.update_alert(AudibleAlert.none)
  np.testing.assert_array_equal(obj.get_sound_data(4096), 0)


def test_urgent_alert_interrupts_one_shot_without_reusing_dm_gain(player):
  obj, *_ = player
  obj.update_alert(AudibleAlert.systemReady)
  obj.get_sound_data(1)
  obj.update_alert(AudibleAlert.warningImmediate, 'driverDistracted3/permanent')
  samples = obj.loaded_sounds[AudibleAlert.warningImmediate]
  np.testing.assert_allclose(obj.get_sound_data(len(samples)), samples)
  obj.update_alert(AudibleAlert.stopStop, 'fcw/permanent')
  np.testing.assert_allclose(obj.get_sound_data(len(samples)), samples * .2)


def test_cutin_text_duration_does_not_repeat_its_chime(player):
  obj, routes, *_ = player
  assert routes[AudibleAlert.radarCutin][:2] == ('prompt.wav', 1)
  samples = obj.loaded_sounds[AudibleAlert.radarCutin]
  outputs = []
  for _ in range(40):  # Two seconds of repeated 20 Hz UI state publications.
    obj.update_alert(AudibleAlert.radarCutin, 'radarCutin/warning')
    outputs.append(obj.get_sound_data(2400))
  actual = np.concatenate(outputs)
  np.testing.assert_allclose(actual[:len(samples)], samples * .2)
  np.testing.assert_array_equal(actual[len(samples):], 0)
