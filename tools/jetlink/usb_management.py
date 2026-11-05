"""Opt-in USB management network, separate from the Jetlink bulk interface."""
import argparse
import errno
import fcntl
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys

GADGET = Path('/sys/kernel/config/usb_gadget/jetlink')
NET = Path('/sys/class/net')
ENABLED = Path('/data/hl-jetson/usb-management.enabled')
BOOT_MODE = Path('/run/hl-jetson-usb-mode')
STATUS = Path('/dev/shm/hl-jetson-usb.json')
FUNCTION = 'ncm.hlmaint'
COMMA_IP = '169.254.77.2'
JETSON_IP = '169.254.77.1'
COMMA_MAC = '02:48:4c:77:00:02'
JETSON_MAC = '02:48:4c:77:00:01'
PROFILE = 'hl-jetson-usb.nmconnection'
PROFILE_UUID = 'ed72d7d1-8e48-4d39-98eb-e7bdf8461a31'


class UnsafeConfigurationError(RuntimeError):
  pass


def run(args):
  return subprocess.run(args, check=True, capture_output=True, text=True, timeout=10).stdout.strip()


def report(state, **extra):
  temporary = STATUS.with_suffix('.new')
  try:
    temporary.write_text(json.dumps(dict(state=state, **extra)))
    temporary.replace(STATUS)
  except OSError as error:
    print('USB management status unavailable: ' + str(error), file=sys.stderr)


def boot_enabled():
  # /run survives USB/manager restarts, but not an OS reboot. Never hot-apply
  # a web setting by changing the inference device's USB descriptors.
  fd = os.open(BOOT_MODE, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
  with os.fdopen(fd, 'r+b') as stream:
    info = os.fstat(stream.fileno())
    if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_mode & 0o022:
      raise ValueError('Untrusted USB boot mode file')
    fcntl.flock(stream, fcntl.LOCK_EX)
    mode = stream.read(3)
    if not mode:
      mode = b'1' if ENABLED.is_file() else b'0'
      stream.write(mode)
      stream.flush()
      os.fsync(stream.fileno())
    if mode not in (b'0', b'1'):
      raise ValueError('Invalid USB boot mode')
    return mode == b'1'


def netdev(gadget=GADGET, net=NET):
  function = gadget / 'functions' / FUNCTION
  link = gadget / 'configs/c.1' / FUNCTION
  if not link.is_symlink() or link.resolve() != function.resolve():
    raise ValueError('Management function is not linked')
  name = (gadget / 'functions' / FUNCTION / 'ifname').read_text().strip()
  if not re.fullmatch(r'[a-zA-Z0-9_.-]{1,15}', name):
    raise ValueError('Invalid management interface name')
  mac = (function / 'dev_addr').read_text().strip('\x00\r\n ').lower()
  if not re.fullmatch(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}', mac) or (net / name / 'address').read_text().strip().lower() != mac:
    raise ValueError('Management interface MAC does not match; refusing network changes')
  return name


def remove_address(name, command):
  addresses = json.loads(command(['ip', '-j', '-4', 'address', 'show', 'dev', name]))
  if any(a.get('local') == COMMA_IP and a.get('prefixlen') == 30
         for item in addresses for a in item.get('addr_info', [])):
    command(['ip', 'address', 'del', COMMA_IP + '/30', 'dev', name])


def detach(gadget=GADGET, net=NET, command=run):
  link = gadget / 'configs/c.1' / FUNCTION
  if not link.is_symlink():
    return
  if (gadget / 'UDC').read_text().strip():
    raise ValueError('Close USB owner before changing management descriptors')
  if link.resolve() != (gadget / 'functions' / FUNCTION).resolve():
    raise ValueError('Unexpected management function link')
  try:
    name = netdev(gadget, net)
    # Old AGNOS destroys this netdev at unbind. Otherwise remove only our IP.
    remove_address(name, command)
  except (OSError, ValueError, TypeError, AttributeError, subprocess.SubprocessError) as error:
    # Ownership and unbound-UDC checks above remain fatal. A missing/foreign
    # netdev or failed IP cleanup must not prevent bulk-only inference startup.
    print('USB management network cleanup skipped: ' + str(error))
  link.unlink()
  # Retain the detached function: old AGNOS kernels can fault on configfs rmdir.
  for attr, value in (('bDeviceClass', '0x00'), ('bDeviceSubClass', '0x00'),
                      ('bDeviceProtocol', '0x00'), ('bcdDevice', '0x0100')):
    (gadget / attr).write_text(value + '\n')


def configure(gadget=GADGET, net=NET, command=run):
  if (gadget / 'UDC').read_text().strip():
    raise ValueError('USB is already bound; apply management only on next offroad start')
  config = gadget / 'configs/c.1'
  if not (config / 'ffs.jetlink').is_symlink():
    raise ValueError('Jetlink FunctionFS must be linked first (interface 0)')
  if (config / 'ncm.jetlink').is_symlink():
    raise ValueError('Mobile NCM and USB management modes must not be combined')
  function = gadget / 'functions' / FUNCTION
  function.mkdir(exist_ok=True)
  link = config / FUNCTION
  if link.exists() or link.is_symlink():
    if not link.is_symlink() or link.resolve() != function.resolve():
      raise ValueError('Unexpected management function link')
  original = {attr: (gadget / attr).read_text() for attr in
              ('bDeviceClass', 'bDeviceSubClass', 'bDeviceProtocol', 'bcdDevice')}
  try:
    for attr, value in (('dev_addr', COMMA_MAC), ('host_addr', JETSON_MAC)):
      try:
        (function / attr).write_text(value + '\n')
      except OSError as error:
        if error.errno not in (errno.ENODEV, errno.EBUSY):
          raise
    # AGNOS 4.9 exposes ifname/MAC only after FunctionFS binds the UDC.
    if not link.is_symlink():
      link.symlink_to(function)
    for attr, value in (('bDeviceClass', '0xEF'), ('bDeviceSubClass', '0x02'),
                        ('bDeviceProtocol', '0x01'), ('bcdDevice', '0x0102')):
      (gadget / attr).write_text(value + '\n')
    return FUNCTION
  except Exception:
    try:
      if link.is_symlink() and link.resolve() == function.resolve():
        link.unlink()
      for attr, value in original.items():
        (gadget / attr).write_text(value)
    except Exception as error:
      raise UnsafeConfigurationError('Cannot restore the original bulk USB descriptors') from error
    raise


def activate(gadget=GADGET, net=NET, command=run):
  if not (gadget / 'UDC').read_text().strip():
    raise ValueError('Bind FunctionFS before configuring the management network')
  name = netdev(gadget, net)
  command(['nmcli', 'device', 'set', name, 'managed', 'no'])
  command(['ip', 'address', 'replace', COMMA_IP + '/30', 'dev', name])
  command(['ip', 'link', 'set', name, 'up'])
  return name


def usb_candidate(net=NET, command=run, gadget=GADGET):
  """Never send a link-local SSH attempt through Wi-Fi or a default gateway."""
  try:
    routes = json.loads(command(['ip', '-j', '-4', 'route', 'get', JETSON_IP]))
    if len(routes) != 1 or 'gateway' in routes[0] or routes[0].get('prefsrc') != COMMA_IP:
      return []
    interface = routes[0].get('dev', '')
    if not re.fullmatch(r'[a-zA-Z0-9_.-]{1,15}', interface):
      return []
    if ((gadget / 'UDC').read_text().strip() and interface == netdev(gadget, net)
        and (net / interface / 'carrier').read_text().strip() == '1'):
      return [JETSON_IP]
  except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError):
    pass
  return []


def host_profile(path=None):
  match = f'[match]\ndriver=cdc_ncm\npath={path}\n[ethernet]' if path else f'[ethernet]\nmac-address={JETSON_MAC}'
  return f'''[connection]
id=HL Jetson USB management
uuid={PROFILE_UUID}
type=ethernet
autoconnect=true
autoconnect-priority=100
{match}
[ipv4]
method=manual
address1={JETSON_IP}/30
never-default=true
ignore-auto-dns=true
ignore-auto-routes=true
route-metric=900
[ipv6]
method=disabled
'''


def host_usb_path(root=Path('/'), command=run):
  matches = []
  for interface in (root / 'sys/class/net').glob('*'):
    device = interface / 'device'
    if (device / 'driver').resolve().name != 'cdc_ncm':
      continue
    for parent in device.resolve().parents:
      try:
        ids = [(parent / attr).read_text().strip().lower() for attr in ('idVendor', 'idProduct', 'bcdDevice')]
      except FileNotFoundError:
        continue
      if ids == ['1209', '0001', '0102']:
        properties = command(['udevadm', 'info', '--query=property', '--path=' + str(interface)])
        values = dict(line.split('=', 1) for line in properties.splitlines() if '=' in line)
        path = values.get('ID_PATH', '')
        if not re.fullmatch(r'[A-Za-z0-9_.:+-]{1,200}', path) or 'usb' not in path:
          raise ValueError('USB management interface has no stable USB path')
        matches.append(path)
      break
  if len(matches) != 1:
    raise ValueError('Connect exactly one management-enabled comma USB cable before installing the host profile')
  return matches[0]


def install_host(root=Path('/'), command=run):
  if not (root / 'etc/carrot-jetlink-protected.json').is_file():
    raise ValueError('Expected protected carrot-jetson image')
  if json.loads((root / 'run/carrot-storage.json').read_text()).get('state') != 'protected':
    raise ValueError('Protected storage is not ready')
  helper = root / 'usr/lib/carrot-jetlink-storage/protected_storage.py'
  if not helper.is_file():
    raise ValueError('Missing durable identity helper')
  path = root / 'etc/NetworkManager/system-connections' / PROFILE
  if path.is_symlink():
    raise ValueError('Unexpected network profile symlink')
  expected = host_profile(host_usb_path(root, command)).encode()
  previous = path.read_bytes() if path.exists() else None
  if previous is not None and previous not in (expected, host_profile().encode()):
    raise ValueError('Existing management profile differs; preserve it for review')
  changed = previous != expected
  if changed:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as stream:
      stream.write(expected)
      stream.flush()
      os.fsync(stream.fileno())
  try:
    command(['nmcli', 'connection', 'load', str(path)])
    command(['/usr/bin/python3', str(helper), 'save'])
  except Exception:
    # Only the exact legacy profile can migrate; restore it if persistence fails.
    if changed:
      command(['nmcli', 'connection', 'delete', 'uuid', PROFILE_UUID])
      if previous is None:
        path.unlink(missing_ok=True)
      else:
        path.write_bytes(previous)
        path.chmod(0o600)
        command(['nmcli', 'connection', 'load', str(path)])
      command(['/usr/bin/python3', str(helper), 'save'])
    raise
  return dict(state='host-prepared', usb_peer=COMMA_IP, address=JETSON_IP,
              physical_usb_verified=False)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('action', choices=('boot', 'gadget', 'net', 'detach', 'install-host'))
  args = parser.parse_args()
  if os.geteuid() != 0:
    parser.error('Run through sudo')
  if args.action == 'install-host':
    print(json.dumps(install_host()))
    return
  elif args.action == 'detach':
    detach()
    return
  try:
    enabled = boot_enabled()
  except (OSError, ValueError) as error:
    report('unavailable', reason=str(error))
    if args.action == 'gadget':
      detach()  # Still refuses a bound controller or a foreign function link.
    print('USB management boot mode unavailable; keeping bulk inference: ' + str(error))
    return
  if args.action == 'boot':
    print(json.dumps(dict(enabled_this_boot=enabled)))
  elif args.action == 'net':
    if enabled:
      try:
        name = activate()
        report('configured', interface=name, address=COMMA_IP, peer=JETSON_IP)
      except Exception as error:
        report('unavailable', reason=str(error))
        print('USB management unavailable; keeping bulk inference: ' + str(error))
  elif enabled:
    try:
      configure()
      report('prepared')
    except Exception as error:
      report('unavailable', reason=str(error))
      # Unsupported NCM/MAC support must leave the original inference gadget.
      if isinstance(error, UnsafeConfigurationError) or (GADGET / 'configs/c.1' / FUNCTION).is_symlink():
        raise
      print('USB management unavailable; keeping bulk inference: ' + str(error))
  else:
    detach()
    report('disabled')


if __name__ == '__main__':
  main()
