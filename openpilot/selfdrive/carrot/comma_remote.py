"""Re-register the opted-in, persistent remote-access service on AGNOS boot."""
from pathlib import Path
import subprocess
import sys
import time


def main():
  helper = Path(__file__).resolve().parents[3] / 'tools/jetlink/setup_comma_remote.py'
  while True:
    result = subprocess.run(['sudo', '-n', sys.executable, str(helper), 'start'],
                            capture_output=True, text=True, timeout=130)
    if result.returncode:
      print('Remote access service unavailable: ' + (result.stderr or result.stdout)[-800:], flush=True)
    time.sleep(30)


if __name__ == '__main__':
  main()
