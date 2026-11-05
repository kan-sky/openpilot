"""Installed in updater/: preserve the selected custom runtime across boots."""
from pathlib import Path
import runpy


if __name__ == '__main__':
  here = Path(__file__).resolve().parent
  if (here.parent / 'HL_CLUSTER_PIN.json').is_file():
    print('cluster_jetson runtime pinned; use install_cluster.py rollback to restore upstream updates')
  else:
    runpy.run_path(str(here / 'update_host.hl-original.py'), run_name='__main__')
