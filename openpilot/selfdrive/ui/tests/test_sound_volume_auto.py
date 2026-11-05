"""Exercise production gain/PCM methods without native IPC, GPIO or audio devices."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import car
from openpilot.common.filter_simple import FirstOrderFilter


ROOT = Path(__file__).resolve().parents[3]
AudibleAlert = car.CarControl.HUDControl.AudibleAlert


def load_functions(path, names, namespace, class_name=None):
  tree = ast.parse(path.read_text())
  body = next(n.body for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name) if class_name else tree.body
  selected = [n for n in body if isinstance(n, ast.FunctionDef) and n.name in names]
  assert {n.name for n in selected} == set(names)
  exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), "exec"), namespace)
  return {name: namespace[name] for name in names}


@pytest.fixture(params=["tici", "tizi"])
def soundd(request):
  path = ROOT / "selfdrive/ui/soundd.py"
  tree = ast.parse(path.read_text())
  namespace = {"math": math, "np": np, "AudibleAlert": AudibleAlert,
               "HARDWARE": SimpleNamespace(get_device_type=lambda: request.param)}
  constants = {"MAX_VOLUME", "MIN_VOLUME", "AMBIENT_DB", "DB_SCALE", "VOLUME_BASE"}
  # Include the real C4-specific ambient threshold and curve base as well.
  nodes = [n for n in tree.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in constants for t in n.targets)]
  nodes += [n for n in tree.body if isinstance(n, ast.If) and any(isinstance(b, ast.Assign) and
            any(isinstance(t, ast.Name) and t.id == "AMBIENT_DB" for t in b.targets) for b in n.body)]
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), "exec"), namespace)
  load_functions(path, ["dm_warning_volume"], namespace)
  methods = load_functions(path, ["calculate_volume", "update_volume", "get_sound_data"], namespace, "Soundd")
  obj = type("VolumeSoundd", (), methods)()
  obj.setting = -1
  obj.params = SimpleNamespace(get_int=lambda key: obj.setting)
  obj.spl_filter_weighted = FirstOrderFilter(0, 2.5, 0.05, initialized=False)
  obj.current_alert = AudibleAlert.none
  obj.current_alert_type = ""
  obj.current_volume = namespace["MIN_VOLUME"]
  obj.current_sound_frame = 0
  samples = np.array([.1, -.2, .3, -.4], dtype=np.float32)
  obj.loaded_sounds = dict.fromkeys((AudibleAlert.prompt, AudibleAlert.promptDistracted, AudibleAlert.warningImmediate), samples)
  namespace["sound_list"] = dict.fromkeys(obj.loaded_sounds, ("unused", 1, 1.))
  return obj


class Noise(dict):
  def __init__(self, db, updated=True):
    super().__init__(soundPressure=SimpleNamespace(soundPressureWeightedDb=db))
    self.updated = {"soundPressure": updated}


@pytest.mark.parametrize("db", [0, 24, 30, 40, 54, 60, 100])
@pytest.mark.parametrize("setting,multiplier", [(-1, 1.), (0, 0.), (5, .05), (30, .3), (100, 1.), (200, 2.), (-2, 0.)])
def test_auto_and_existing_percent_gain_reach_pcm(soundd, db, setting, multiplier):
  soundd.setting = setting
  soundd.update_volume(Noise(db))
  assert soundd.current_volume == pytest.approx(soundd.calculate_volume(db) * multiplier)
  soundd.current_alert = AudibleAlert.prompt
  np.testing.assert_allclose(soundd.get_sound_data(4), soundd.loaded_sounds[AudibleAlert.prompt] * soundd.current_volume)


def test_live_switch_and_ambient_filter_do_not_learn_the_alert_itself(soundd):
  soundd.update_volume(Noise(30))
  low = soundd.current_volume
  for _ in range(200):
    soundd.update_volume(Noise(60))
  assert soundd.current_volume > low
  soundd.setting = 30
  soundd.update_volume(Noise(60))
  assert soundd.current_volume == pytest.approx(soundd.calculate_volume(soundd.spl_filter_weighted.x) * .3)
  soundd.current_alert = AudibleAlert.prompt
  soundd.setting = -1
  frozen = soundd.current_volume, soundd.spl_filter_weighted.x
  soundd.update_volume(Noise(100))
  assert (soundd.current_volume, soundd.spl_filter_weighted.x) == frozen
  soundd.current_alert = AudibleAlert.none
  soundd.update_volume(Noise(0, updated=False))
  assert (soundd.current_volume, soundd.spl_filter_weighted.x) == frozen
  soundd.update_volume(Noise(40))
  assert soundd.current_volume == pytest.approx(soundd.calculate_volume(soundd.spl_filter_weighted.x))


@pytest.mark.parametrize("name", ["driverDistracted", "driverUnresponsive"])
@pytest.mark.parametrize("stage,gain", [(2, .7), (3, 1.)])
def test_auto_preserves_dm_pcm_floor(soundd, name, stage, gain):
  soundd.update_volume(Noise(0))
  soundd.current_alert = AudibleAlert.promptDistracted if stage == 2 else AudibleAlert.warningImmediate
  soundd.current_alert_type = f"{name}{stage}/permanent"
  np.testing.assert_allclose(soundd.get_sound_data(4), soundd.loaded_sounds[soundd.current_alert] * gain)


def test_runtime_reads_volume_before_dispatching_alerts():
  tree = ast.parse((ROOT / "selfdrive/ui/soundd.py").read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Soundd")
  thread = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "soundd_thread")
  calls = {n.func.attr: n.lineno for n in ast.walk(thread) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
  assert calls["update_volume"] < calls["get_audible_alert"]


@pytest.mark.parametrize("setting,enabled", [(-1, True), (0, False), (5, False), (6, True), (100, True), (-2, False)])
def test_auto_does_not_mute_gpio_beep(setting, enabled):
  calls = []
  subprocess = SimpleNamespace(run=lambda *a, **kw: calls.append(a), DEVNULL=-3)
  functions = load_functions(ROOT / "selfdrive/controls/beep.py", ["_beep"], {"subprocess": subprocess}, "Beepd")
  obj = SimpleNamespace(params=SimpleNamespace(get_int=lambda key: setting))
  for on in (True, False):
    functions["_beep"](obj, on)
    assert calls[-1][0].startswith(f'echo "{int(enabled and on)}" ')


@pytest.mark.parametrize("key,setting,expected", [
  ("SoundVolumeAdjust", -1, 1.), ("SoundVolumeAdjust", 0, 0.), ("SoundVolumeAdjust", 30, .3),
  ("SoundVolumeAdjust", 200, 2.), ("SoundVolumeAdjust", -2, 0.), ("SoundVolumeAdjustEngage", -1, 0.),
  ("SoundVolumeAdjustEngage", 10, .1),
])
def test_web_metadata_distinguishes_auto_from_mute(key, setting, expected):
  functions = load_functions(ROOT / "selfdrive/carrot/server/features/web_sound.py", ["_param_percent"], {"Params": object})
  assert functions["_param_percent"](SimpleNamespace(get_int=lambda key: setting), key) == expected
