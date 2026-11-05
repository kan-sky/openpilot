from openpilot.selfdrive.carrot.server import config


def test_both_checkout_layouts_migrate_without_overwriting_newer_state(tmp_path, monkeypatch):
  current, packaged, flat = (tmp_path / name for name in ('current', 'packaged', 'flat'))
  for path in (current, packaged, flat):
    path.mkdir()
  monkeypatch.setattr(config, 'CARROT_STATE_DIR', str(current))
  monkeypatch.setattr(config, 'CARROT_LEGACY_STATE_DIR', str(packaged))
  monkeypatch.setattr(config, 'CARROT_PRE_PACKAGE_STATE_DIR', str(flat))
  (current / 'web_settings.json').write_text('{"source":"current"}')
  (packaged / 'web_settings.json').write_text('{"source":"packaged"}')
  (packaged / 'setting_profiles.json').write_text('{"source":"packaged"}')
  (flat / 'setting_profiles.json').write_text('{"source":"flat"}')
  (flat / 'setting_favorites.json').write_text('{"source":"flat"}')
  (flat / 'unexpected.txt').write_text('not user state')

  config.migrate_legacy_carrot_state()
  config.migrate_legacy_carrot_state()

  assert (current / 'web_settings.json').read_text() == '{"source":"current"}'
  assert (current / 'setting_profiles.json').read_text() == '{"source":"packaged"}'
  assert (current / 'setting_favorites.json').read_text() == '{"source":"flat"}'
  assert not (current / 'unexpected.txt').exists()
  assert (flat / 'setting_favorites.json').is_file()


def test_flat_checkout_migrates_when_packaged_directory_is_missing(tmp_path, monkeypatch):
  flat, current = tmp_path / 'flat', tmp_path / 'current'
  flat.mkdir()
  (flat / 'web_settings.json').write_text('{}')
  monkeypatch.setattr(config, 'CARROT_STATE_DIR', str(current))
  monkeypatch.setattr(config, 'CARROT_LEGACY_STATE_DIR', str(tmp_path / 'missing'))
  monkeypatch.setattr(config, 'CARROT_PRE_PACKAGE_STATE_DIR', str(flat))
  config.migrate_legacy_carrot_state()
  assert (current / 'web_settings.json').read_text() == '{}'
