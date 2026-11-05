"""A raw asset must not become a dirty LFS pointer merely by checking status."""
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]


def test_lfs_filter_only_matches_pointer_backed_assets():
  def git(*args, **kwargs):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], **kwargs)
  paths = git('ls-files', '-z')
  attributes = git('check-attr', '-z', '--stdin', 'filter', input=paths).split(b'\0')[:-1]
  filtered = [attributes[i].decode() for i in range(0, len(attributes), 3) if attributes[i + 2] == b'lfs']
  for name in filtered:
    # Check size before reading: accidentally raw model blobs can be very large.
    assert int(git('cat-file', '-s', 'HEAD:' + name)) < 1024, name
    assert git('show', 'HEAD:' + name).startswith(b'version https://git-lfs.github.com/spec/v1\n'), name


def test_custom_cluster_assets_are_materialized_and_not_lfs_filtered():
  paths = list((ROOT / 'openpilot/selfdrive/assets/fonts').glob('Pretendard*.ttf'))
  paths += list((ROOT / 'openpilot/selfdrive/carrot/cluster/assets/ambient').glob('*.png'))
  paths += [ROOT / 'openpilot/selfdrive/assets/icons_mici' / name for name in
            ('ego_vehicle_custom.png', 'carrot_cruse_gap_trimmed.png', 'carrot_wheel_org.png')]
  assert len(paths) == 14
  for path in paths:
    with path.open('rb') as stream:
      magic = stream.read(8)
    assert magic.startswith(b'\x00\x01\x00\x00') if path.suffix == '.ttf' else magic == b'\x89PNG\r\n\x1a\n'
    attribute = subprocess.check_output(['git', '-C', str(ROOT), 'check-attr', 'filter', str(path)], text=True)
    assert not attribute.rstrip().endswith(': lfs'), path
