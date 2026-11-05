import ast
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

CLUSTER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CLUSTER))
import cluster_renderer as renderer_module
from cluster_config import cluster_theme_brightness_percent
from cluster_live import OpenpilotLiveSource, standby_state
from cluster_models import PhoneMediaInfo
from cluster_renderer import ClusterUiRenderer


@pytest.mark.parametrize('mode', [0, 1, 2, 3, 4, -1, 5, 6])
def test_alert_is_drawn_after_ambient_or_latest_dashboard(mode):
    renderer = ClusterUiRenderer(screen_mode=mode)
    calls = []
    for method in ('_clear_ambient_dashboard', '_draw_ambient_dashboard', '_clear_world',
                   '_render_world', '_draw_hud', '_draw_alert_overlay'):
        setattr(renderer, method, lambda *args, name=method: calls.append(name))
    renderer.render(standby_state(), signal_lights=(False, False))
    assert calls[-1] == '_draw_alert_overlay'
    if 0 <= mode <= 4:
        assert calls[:-1] == ['_clear_ambient_dashboard', '_draw_ambient_dashboard']
    else:
        assert calls[-2] == '_draw_hud'
        assert '_draw_ambient_dashboard' not in calls
    renderer.close()


def test_added_resources_are_released_before_native_window(monkeypatch):
    renderer = ClusterUiRenderer()
    calls = []
    renderer._window_open = True
    for name in ('_phone_media_art_texture', '_ambient_reference_texture',
                 '_ambient_cruise_icon_texture', '_ambient_lane_assist_icon_texture'):
        setattr(renderer, name, name)
    renderer._phone_media_unicode_font = 'unicode'
    renderer._ambient_fonts = {'bold': 'bold', 'regular': 'regular'}
    renderer._owned_ambient_fonts = {'bold', 'regular'}
    monkeypatch.setattr(renderer_module.rl, 'unload_texture', lambda value: calls.append(value))
    monkeypatch.setattr(renderer_module.rl, 'unload_font', lambda value: calls.append(value))
    monkeypatch.setattr(renderer_module.rl, 'close_window', lambda: calls.append('native'))
    renderer.close()
    assert len(calls) == 8 and calls[-1] == 'native'
    assert not renderer._window_open and not renderer._ambient_fonts
    renderer.close()
    assert len(calls) == 8


def test_ambient_font_ownership_survives_filter_setup_failure(monkeypatch):
    renderer = ClusterUiRenderer()
    fonts = []
    def load(*args):
        font = SimpleNamespace(texture=SimpleNamespace(id=1))
        fonts.append(font)
        return font
    def fail(*args):
        raise RuntimeError('filter')
    monkeypatch.setattr(renderer_module.rl, 'load_font_ex', load)
    monkeypatch.setattr(renderer_module.rl, 'set_texture_filter', fail)
    renderer._load_ambient_fonts()
    assert len(fonts) == len(renderer._owned_ambient_fonts) == 5
    released = []
    monkeypatch.setattr(renderer_module.rl, 'unload_font', released.append)
    renderer._close_ambient_resources()
    assert len(released) == 5
    renderer.close()


def test_phone_hook_preserves_alerts_and_latest_navigation_state():
    source = object.__new__(OpenpilotLiveSource)
    phone = PhoneMediaInfo(title='Title')
    source._phone_media_reader = SimpleNamespace(read=lambda: phone)
    state = standby_state()
    result = source._with_phone_media(state)
    assert result == replace(state, phone_media=phone)
    assert source._with_phone_media(result) is result


def test_ambient_section_failure_does_not_skip_subsequent_sections(monkeypatch):
    renderer = ClusterUiRenderer()
    events = []
    renderer._append_ambient_diag = lambda message, exc: events.append(message)
    def fail():
        raise RuntimeError('broken cover')
    renderer._draw_ambient_step('phone_media', fail)
    renderer._draw_ambient_step('phone_media', fail)
    renderer._draw_ambient_step('bsm_edges', lambda: events.append('bsm_edges'))
    assert events == ['ambient section failed: phone_media', 'bsm_edges']
    renderer.close()


@pytest.mark.parametrize('manual', [0, 1, 60, 100])
@pytest.mark.parametrize('auto', [False, True])
@pytest.mark.parametrize('theme,expected', [('light', 60), ('dark', 20)])
def test_brightness_is_theme_only_even_with_manual_and_live_input(manual, auto, theme, expected):
    tree = ast.parse((CLUSTER / 'main.py').read_text())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'resolved_usb_brightness')
    scope = {'OpenpilotLiveSource': OpenpilotLiveSource, 'cluster_theme_brightness_percent': cluster_theme_brightness_percent}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), 'brightness', 'exec'), scope)
    live = SimpleNamespace(screen_brightness_percent=lambda: pytest.fail('live brightness must not be read'))
    assert scope['resolved_usb_brightness'](manual, live, auto_enabled=auto, theme_mode=theme) == expected


def test_native_cleanup_adapter_has_not_been_replaced():
    source = (CLUSTER / 'cluster_graphics_cleanup.py').read_text()
    assert 'gbm_surface_destroy' in (CLUSTER / 'gbm_cleanup.c').read_text()
    assert 'DRM_FD' in source
    assert 'os.execve' in source


def test_ambient_requested_geometry_palette_and_music_weight():
    assert renderer_module.AMBIENT_BSM_W == 210
    assert renderer_module.AMBIENT_BSM_FADE_SECONDS == .35
    assert renderer_module.AMBIENT_BSM_DARK_COLOR == (204, 88, 0)
    assert renderer_module.AMBIENT_BSM_LIGHT_COLOR == (255, 152, 88)
    assert renderer_module.AMBIENT_CLOCK_RIGHT_X == 1875
    assert renderer_module.PHONE_MEDIA_UNICODE_FONT_PATH.name == 'Pretendard-Bold.ttf'
