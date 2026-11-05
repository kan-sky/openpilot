#!/usr/bin/env python3
"""Build an isolated trial APK with existing SDK/JDK; never install or pair."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import zipfile

HERE = Path(__file__).resolve().parent


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--jdk', type=Path, required=True)
  parser.add_argument('--sdk', type=Path, required=True)
  parser.add_argument('--out', type=Path, required=True)
  args = parser.parse_args()
  out = args.out.resolve()
  out.mkdir(parents=True, exist_ok=False)
  build_tools = args.sdk / 'build-tools/35.0.0'
  android = args.sdk / 'platforms/android-35/android.jar'
  env = dict(os.environ, JAVA_HOME=str(args.jdk), PATH=str(args.jdk / 'bin') + os.pathsep + os.environ['PATH'])

  def run(*cmd):
    subprocess.run(list(map(str, cmd)), check=True, env=env)

  classes = out / 'classes'
  classes.mkdir()
  run(args.jdk / 'bin/javac', '-source', '8', '-target', '8', '-classpath', android,
      '-d', classes, *sorted((HERE / 'android').glob('*.java')))
  run(args.jdk / 'bin/jar', 'cf', out / 'classes.jar', '-C', classes, '.')
  run(build_tools / 'd8', '--min-api', '19', '--lib', android, '--output', out, out / 'classes.jar')
  run(build_tools / 'aapt', 'package', '-f', '-M', HERE / 'android/AndroidManifest.xml',
      '-I', android, '-F', out / 'unsigned.apk')
  with zipfile.ZipFile(out / 'unsigned.apk', 'a') as apk:
    apk.write(out / 'classes.dex', 'classes.dex')
  run(build_tools / 'zipalign', '-f', '4', out / 'unsigned.apk', out / 'aligned.apk')
  # A separate local test identity cannot replace or grant access to the production HUD app.
  keydir = Path.home() / '.local/share/navdy-bt-bench'
  keydir.mkdir(parents=True, mode=0o700, exist_ok=True)
  key = keydir / 'test.keystore'
  if not key.exists():
    run(args.jdk / 'bin/keytool', '-genkeypair', '-keystore', key, '-alias', 'bench',
        '-storepass', 'android', '-keypass', 'android', '-keyalg', 'RSA', '-keysize', '2048',
        '-validity', '3650', '-dname', 'CN=HL Local Benchmark')
    key.chmod(0o600)
  apk = out / 'HL-Navdy-Bluetooth-Bench.apk'
  run(build_tools / 'apksigner', 'sign', '--ks', key, '--ks-key-alias', 'bench',
      '--ks-pass', 'pass:android', '--key-pass', 'pass:android', '--min-sdk-version', '19',
      '--out', apk, out / 'aligned.apk')
  run(build_tools / 'apksigner', 'verify', '--verbose', apk)
  print(json.dumps({'apk': str(apk), 'sha256': hashlib.sha256(apk.read_bytes()).hexdigest(),
                    'installed': False, 'paired': False}))


if __name__ == '__main__':
  main()
