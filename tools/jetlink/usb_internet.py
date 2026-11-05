"""Opt-in Internet routing for a pinned Jetson on the existing mobile USB link.

No USB rebind, public listeners, blanket NAT, or model/control configuration.
Only an SSH-authenticated peer receives routing; existing firewall rules stay.
"""
import argparse
import fcntl
import ipaddress
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

STATE = Path('/data/hl-jetson')
GADGET = Path('/sys/kernel/config/usb_gadget/jetlink')
NETWORK = ipaddress.ip_network('192.168.60.0/24')
GATEWAY = '192.168.60.1'
CHAIN = 'HL_JETSON_NET'
STATUS = Path('/dev/shm/hl-jetson-internet.json')
REMOTE = '/run/carrot-data/hl-network/usb_internet.py'


def run(args, timeout=15):
  result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
  if result.returncode:
    raise RuntimeError(f'{args[0]} failed: {result.stderr.strip()[-500:]}')
  return result.stdout.strip()


def interface(value):
  if not re.fullmatch(r'[a-zA-Z0-9_.-]{1,15}', value):
    raise ValueError('Invalid interface')
  return value


def peer_address(value):
  addr = ipaddress.ip_address(value)
  if addr not in NETWORK or str(addr) in (str(NETWORK.network_address), str(NETWORK.broadcast_address), GATEWAY):
    raise ValueError('Not a mobile USB peer')
  return str(addr)


def cable(command=run, gadget=GADGET, net=Path('/sys/class/net')):
  function = gadget / 'functions/ncm.jetlink'
  link = gadget / 'configs/c.1/ncm.jetlink'
  if not (gadget / 'UDC').read_text().strip() or not link.is_symlink() or link.resolve() != function.resolve():
    raise ValueError('Mobile USB function is not bound')
  name = interface((function / 'ifname').read_text().strip())
  if (net / name / 'carrier').read_text().strip() != '1':
    raise ValueError('USB cable has no carrier')
  if (net / name / 'address').read_text().strip().lower() != (function / 'dev_addr').read_text().strip('\x00\r\n ').lower():
    raise ValueError('USB MAC mismatch')
  addresses = json.loads(command(['ip', '-j', '-4', 'addr', 'show', 'dev', name]))
  if not any(a.get('local') == GATEWAY and a.get('prefixlen') == 24 for item in addresses for a in item.get('addr_info', [])):
    raise ValueError('USB gateway address missing')
  peers = []
  for row in json.loads(command(['ip', '-j', '-4', 'neigh', 'show', 'dev', name])):
    if row.get('state') and not set(row['state']).intersection({'FAILED', 'INCOMPLETE'}):
      try:
        peers.append(peer_address(row['dst']))
      except (ValueError, KeyError):
        continue
  return name, list(dict.fromkeys(peers))[:8]


def mobile_candidates():
  try:
    return cable()[1]
  except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError):
    return []


def uplink(usb, command=run):
  rows = json.loads(command(['ip', '-j', '-4', 'route', 'get', '1.1.1.1']))
  if len(rows) != 1 or not rows[0].get('gateway'):
    raise ValueError('No Internet uplink')
  name = interface(rows[0]['dev'])
  if name == usb or name.startswith(('tailscale', 'tun', 'usb', 'wg', 'lo')):
    raise ValueError('Not a physical Internet uplink')
  values = re.split(r'[|,\s]+', command(['nmcli', '-g', 'IP4.DNS', 'device', 'show', name]))
  dns = [str(ipaddress.IPv4Address(v)) for v in values if v][:3]
  if not dns:
    raise ValueError('Uplink DNS unavailable')
  return name, dns


def firewall_rules(usb, wan, peer, dns, restrict_wan):
  interface(usb), interface(wan), peer_address(peer)
  source = peer + '/32'
  forward = [['-i', wan, '-o', usb, '-d', source, '-m', 'conntrack', '--ctstate', 'ESTABLISHED,RELATED', '-j', 'ACCEPT']]
  for server in dns:
    server = str(ipaddress.IPv4Address(server))
    for protocol in ('udp', 'tcp'):
      forward.append(['-i', usb, '-o', wan, '-s', source, '-d', server + '/32', '-p', protocol, '--dport', '53', '-j', 'ACCEPT'])
  # Internet sharing must not turn the Jetson into a route to other LAN/VPN peers.
  for network in ('0.0.0.0/8', '10.0.0.0/8', '100.64.0.0/10', '127.0.0.0/8', '169.254.0.0/16',
                  '172.16.0.0/12', '192.168.0.0/16', '224.0.0.0/4', '240.0.0.0/4'):
    forward.append(['-i', usb, '-d', network, '-j', 'DROP'])
  forward += [['-i', usb, '-o', wan, '-s', source, '-m', 'conntrack', '--ctstate', 'NEW,ESTABLISHED,RELATED', '-j', 'ACCEPT'],
              ['-i', usb, '-j', 'DROP'], ['-o', usb, '-j', 'DROP']]
  if restrict_wan:
    forward.append(['-i', wan, '-j', 'DROP'])
  nat = [['-s', source, '-o', wan, '-j', 'MASQUERADE']]
  return forward, nat


class Router:
  def __init__(self, command=run, proc=Path('/proc/sys/net/ipv4/conf'), journal=Path('/run/hl-jetson-router.json')):
    self.command, self.proc = command, proc
    self.journal = journal
    self.backend = None
    self.saved = {}
    self.current = None
    if journal.exists():
      value = json.loads(journal.read_text())
      if value.get('backend') not in ('iptables-legacy', 'iptables') or len(value.get('saved', {})) != 2:
        raise ValueError('Invalid router recovery journal')
      for name, original in value['saved'].items():
        interface(name)
        if original not in ('0', '1'):
          raise ValueError('Invalid original forwarding state')
      self.backend, self.saved = value['backend'], value['saved']
      self.close()

  def ipt(self, *args):
    return self.command([self.backend, '-w', '3', *args])

  def present(self, *args):
    try:
      self.ipt(*args)
      return True
    except (OSError, RuntimeError):
      return False

  def apply(self, usb, wan, peer, dns):
    identity = (usb, wan, peer, tuple(dns))
    if identity == self.current:
      return
    self.close()
    for backend in ('iptables-legacy', 'iptables'):
      self.backend = backend
      if self.present('-S', 'FORWARD') and self.present('-t', 'nat', '-S', 'POSTROUTING'):
        break
    else:
      raise RuntimeError('No usable IPv4 firewall backend')
    for name in (usb, wan):
      self.saved[name] = (self.proc / interface(name) / 'forwarding').read_text().strip()
    forward, nat = firewall_rules(usb, wan, peer, dns, self.saved[wan] == '0')
    temporary = self.journal.with_suffix('.tmp')
    temporary.write_text(json.dumps({'backend': self.backend, 'saved': self.saved}))
    temporary.chmod(0o600)
    temporary.replace(self.journal)
    try:
      for table, parent, rules in (('filter', 'FORWARD', forward), ('nat', 'POSTROUTING', nat)):
        if not self.present('-t', table, '-S', CHAIN):
          self.ipt('-t', table, '-N', CHAIN)
        self.ipt('-t', table, '-F', CHAIN)
        for rule in rules:
          self.ipt('-t', table, '-A', CHAIN, *rule)
        if not self.present('-t', table, '-C', parent, '-j', CHAIN):
          self.ipt('-t', table, '-I', parent, '1', '-j', CHAIN)
      for name in (usb, wan):
        (self.proc / name / 'forwarding').write_text('1\n')
      self.current = identity
    except BaseException:
      self.close()
      raise

  def close(self):
    for name, old in self.saved.items():
      path = self.proc / name / 'forwarding'
      if path.exists() and path.read_text().strip() == '1':
        path.write_text(old + '\n')
    self.saved.clear()
    if self.backend:
      for table, parent in (('filter', 'FORWARD'), ('nat', 'POSTROUTING')):
        while self.present('-t', table, '-C', parent, '-j', CHAIN):
          self.ipt('-t', table, '-D', parent, '-j', CHAIN)
        if self.present('-t', table, '-S', CHAIN):
          self.ipt('-t', table, '-F', CHAIN)
          self.ipt('-t', table, '-X', CHAIN)
    self.current = None
    self.journal.unlink(missing_ok=True)


def configure_peer(gateway, dns, epoch, command=run, net=Path('/sys/class/net'),
                   cache=Path('/run/carrot-data/hl-network/network-config.json')):
  if gateway != GATEWAY or not dns or len(dns) > 3:
    raise ValueError('Invalid USB Internet configuration')
  if epoch < 1735689600 or epoch > 4102444800:
    raise ValueError('Invalid comma clock')
  dns = [str(ipaddress.IPv4Address(v)) for v in dns]
  routes = json.loads(command(['ip', '-j', '-4', 'route', 'get', gateway]))
  if len(routes) != 1 or routes[0].get('gateway'):
    raise ValueError('Comma is not on the direct USB network')
  name = interface(routes[0]['dev'])
  peer_address(routes[0]['prefsrc'])
  device = net / name / 'device'
  if (device / 'driver').resolve().name != 'cdc_ncm':
    raise ValueError('Not a CDC-NCM USB interface')
  node = device.resolve()
  if not any((p / 'idVendor').is_file() and (p / 'idVendor').read_text().strip() == '1209'
             and (p / 'idProduct').read_text().strip() == '0001' for p in (node, *node.parents)):
    raise ValueError('Not the comma Jetlink USB device')
  uid = command(['nmcli', '-g', 'GENERAL.CON-UUID', 'device', 'show', name])
  if not re.fullmatch('[0-9a-f-]{36}', uid):
    raise ValueError('USB NetworkManager profile unavailable')
  # A DHCP profile cannot set ipv4.gateway without a static address. Add an
  # explicit default route while preserving the comma-assigned DHCP address.
  ip_fields = {'ipv4.routes': f'0.0.0.0/0 {gateway} 850', 'ipv4.dns': ','.join(dns), 'ipv4.ignore-auto-dns': 'yes',
               'ipv4.never-default': 'no', 'ipv4.route-metric': '850'}
  fields = dict(ip_fields, **{'connection.autoconnect': 'yes', 'connection.interface-name': '', 'match.driver': 'cdc_ncm'})
  desired = dict(uuid=uid, fields=fields)
  def observed():
    return {key: command(['nmcli', '-g', key, 'connection', 'show', uid]) for key in fields}
  try:
    saved = json.loads(cache.read_text())
  except (OSError, ValueError):
    saved = {}
  changed = saved.get('desired') != desired or saved.get('observed') != observed()
  if changed:
    command(['nmcli', 'connection', 'modify', uid, *[part for pair in fields.items() for part in pair]])
    # Apply only IP properties live; matching/identity changes are next-boot
    # metadata, not grounds for disconnecting the current management cable.
    command(['nmcli', 'device', 'modify', name, *[part for pair in ip_fields.items() for part in pair]])
    helper = Path('/usr/lib/carrot-jetlink-storage/protected_storage.py')
    if helper.is_file():
      command(['/usr/bin/python3', str(helper), 'save'], timeout=30)
    temporary = cache.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(desired=desired, observed=observed())))
    temporary.chmod(0o600)
    temporary.replace(cache)
  # The protected image has no battery-backed clock. Correct large drift using
  # the authenticated comma before HTTPS; NTP remains enabled for normal sync.
  if abs(time.time() - epoch) > 300:
    command(['date', '-u', '--set', '@' + str(epoch)])
  return {'interface': name, 'changed': changed}


def publish(state, **details):
  temporary = STATUS.with_suffix('.tmp')
  temporary.write_text(json.dumps(dict(state=state, updated=time.monotonic(), **details)))
  temporary.chmod(0o644)
  temporary.replace(STATUS)


def daemon():
  from manage_jetson import paired_config, ssh
  def stop(*_):
    raise SystemExit(0)
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  with Path('/run/hl-jetson-internet.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    router = Router()
    try:
      while True:
        try:
          if not (STATE / 'usb-internet.enabled').is_file():
            raise ValueError('USB Internet sharing is disabled')
          paired_config(STATE)
          usb, peers = cable()
          wan, dns = uplink(usb)
          peer = next((host for host in peers if authenticated(host, ssh)), None)
          if peer is None:
            raise ValueError('No paired Jetson on USB')
          router.apply(usb, wan, peer, dns)
          result = ssh(STATE, peer, ['sudo', '-n', '/usr/bin/python3', REMOTE, 'peer',
                                    '--gateway', GATEWAY, '--dns', *dns, '--epoch', str(int(time.time()))], timeout=55)
          publish('connected', peer=peer, usb=usb, uplink=wan, peer_configuration=json.loads(result))
        except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
          router.close()
          publish('waiting', reason=str(error)[-600:])
        time.sleep(20)
    finally:
      router.close()
      publish('stopped')


def authenticated(host, ssh):
  try:
    return ssh(STATE, host, ['id', '-un'], timeout=6).strip() == 'jetlink'
  except (OSError, RuntimeError, subprocess.SubprocessError):
    return False


def start_service():
  if not (STATE / 'usb-internet.enabled').is_file():
    return
  source = Path(__file__).with_name('hl-jetson-internet.service')
  target = Path('/run/systemd/system/hl-jetson-internet.service')
  if not target.is_file() or target.read_bytes() != source.read_bytes():
    temporary = target.with_suffix('.new')
    temporary.write_bytes(source.read_bytes())
    temporary.chmod(0o644)
    temporary.replace(target)
    run(['systemctl', 'daemon-reload'])
  run(['systemctl', 'start', 'hl-jetson-internet.service'])


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  sub = parser.add_subparsers(dest='command', required=True)
  sub.add_parser('daemon')
  sub.add_parser('start-service')
  peer = sub.add_parser('peer')
  peer.add_argument('--gateway', required=True)
  peer.add_argument('--dns', nargs='+', required=True)
  peer.add_argument('--epoch', type=int, required=True)
  args = parser.parse_args()
  if os.geteuid() != 0:
    parser.error('Run as root')
  if args.command == 'peer':
    print(json.dumps(configure_peer(args.gateway, args.dns, args.epoch)))
  elif args.command == 'start-service':
    start_service()
  else:
    daemon()


if __name__ == '__main__':
  main()
