import json
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import usb_internet as internet


def test_native_configfs_nul_terminated_mac(tmp_path):
  gadget, net = tmp_path / 'gadget', tmp_path / 'net'
  function = gadget / 'functions/ncm.jetlink'
  function.mkdir(parents=True)
  (gadget / 'configs/c.1').mkdir(parents=True)
  (gadget / 'configs/c.1/ncm.jetlink').symlink_to(function)
  (gadget / 'UDC').write_text('a600000.dwc3')
  (function / 'ifname').write_text('usb0')
  (function / 'dev_addr').write_bytes(b'06:27:1c:0a:5a:0c\x00')
  (net / 'usb0').mkdir(parents=True)
  (net / 'usb0/carrier').write_text('1\n')
  (net / 'usb0/address').write_text('06:27:1c:0a:5a:0c\n')
  def command(args):
    if 'addr' in args:
      return json.dumps([{'addr_info': [{'local': internet.GATEWAY, 'prefixlen': 24}]}])
    return json.dumps([{'dst': '192.168.60.12', 'state': ['REACHABLE']},
                       {'dst': '192.168.60.13', 'state': ['FAILED']}])
  assert internet.cable(command, gadget, net) == ('usb0', ['192.168.60.12'])


@pytest.mark.parametrize('value', ['192.168.60.0', '192.168.60.1', '192.168.60.255', '192.168.35.175', '169.254.77.1', '::1'])
def test_reject_foreign_peer(value):
  with pytest.raises(ValueError):
    internet.peer_address(value)


def test_firewall_limits_peer_and_preserves_local_input():
  forward, nat = internet.firewall_rules('usb0', 'wlan0', '192.168.60.12', ['192.168.35.1'], True)
  accepts = [rule for rule in forward if rule[-1] == 'ACCEPT']
  assert len(accepts) == 4
  assert all('192.168.60.12/32' in rule for rule in accepts)
  assert forward[-3:] == [['-i', 'usb0', '-j', 'DROP'], ['-o', 'usb0', '-j', 'DROP'], ['-i', 'wlan0', '-j', 'DROP']]
  assert nat == [['-s', '192.168.60.12/32', '-o', 'wlan0', '-j', 'MASQUERADE']]
  assert any('192.168.0.0/16' in rule and rule[-1] == 'DROP' for rule in forward)
  assert not any('INPUT' in rule for rule in forward)


def test_existing_wan_forwarding_unchanged():
  rules, _ = internet.firewall_rules('usb0', 'wlan0', '192.168.60.12', ['1.1.1.1'], False)
  assert ['-i', 'wlan0', '-j', 'DROP'] not in rules


@pytest.mark.parametrize('name', ['tailscale0', 'usb0', 'tun0', 'wg0'])
def test_uplink_rejects_tunnel_and_usb(name):
  with pytest.raises(ValueError):
    internet.uplink('usb0', lambda args: json.dumps([{'dev': name, 'gateway': '1.2.3.4'}]))


def test_authentication_failure_never_trusted():
  def fail(*a, **kw):
    raise RuntimeError('host key differs')
  assert not internet.authenticated('192.168.60.12', fail)
  assert not internet.authenticated('192.168.60.12', lambda *a, **kw: 'other')
  assert internet.authenticated('192.168.60.12', lambda *a, **kw: 'jetlink\n')


@pytest.mark.parametrize('reply', ['210.220.163.82 | 219.250.36.130', '210.220.163.82\n219.250.36.130',
                                  '210.220.163.82,219.250.36.130'])
def test_native_dns_list_formats(reply):
  def command(args):
    return json.dumps([{'dev': 'wlan0', 'gateway': '192.168.35.1'}]) if args[0] == 'ip' else reply
  assert internet.uplink('usb0', command) == ('wlan0', ['210.220.163.82', '219.250.36.130'])


def test_peer_preserves_dhcp_and_does_not_rewrite_persistent_profile(tmp_path):
  net = tmp_path / 'net'
  physical = tmp_path / 'usb/device/interface'
  physical.mkdir(parents=True)
  (physical.parent / 'idVendor').write_text('1209')
  (physical.parent / 'idProduct').write_text('0001')
  driver = tmp_path / 'drivers/cdc_ncm'
  driver.mkdir(parents=True)
  (physical / 'driver').symlink_to(driver)
  (net / 'enxpeer').mkdir(parents=True)
  (net / 'enxpeer/device').symlink_to(physical)
  profile = {}
  changes = []
  def command(args, **kwargs):
    if args[0] == 'ip':
      return json.dumps([{'dev': 'enxpeer', 'prefsrc': '192.168.60.12'}])
    if args[1:3] == ['-g', 'GENERAL.CON-UUID']:
      return 'e3bc6629-c586-4700-9fbc-752a999c607c'
    if args[1] == '-g':
      return profile.get(args[2], '')
    if args[1:3] == ['connection', 'modify']:
      profile.update(zip(args[4::2], args[5::2]))
      changes.append(args)
    elif args[1:3] == ['device', 'modify']:
      changes.append(args)
    else:
      raise AssertionError(args)
    return ''
  cache = tmp_path / 'profile.json'
  def configure():
    return internet.configure_peer(internet.GATEWAY, ['1.1.1.1'], int(time.time()), command, net, cache)
  assert configure()['changed']
  assert not configure()['changed']
  assert len(changes) == 2
  assert profile['ipv4.routes'] == '0.0.0.0/0 192.168.60.1 850'
  assert 'ipv4.gateway' not in profile and 'ipv4.method' not in profile and 'ipv4.addresses' not in profile
  assert 'match.driver' not in changes[1]


class Firewall:
  def __init__(self):
    self.chains = {}
    self.hooks = set()
    self.calls = []

  def __call__(self, args):
    self.calls.append(args)
    args = args[3:]
    table = 'filter'
    if args[:1] == ['-t']:
      table, args = args[1], args[2:]
    op, name = args[:2]
    if op == '-S':
      if name in ('FORWARD', 'POSTROUTING') or (table, name) in self.chains:
        return ''
    elif op == '-N':
      self.chains[table, name] = []
      return ''
    elif op == '-F':
      self.chains[table, name] = []
      return ''
    elif op == '-X':
      self.chains.pop((table, name))
      return ''
    elif op == '-A':
      self.chains[table, name].append(args[2:])
      return ''
    elif op == '-C':
      if (table, name) in self.hooks:
        return ''
    elif op == '-I':
      self.hooks.add((table, name))
      return ''
    elif op == '-D':
      self.hooks.remove((table, name))
      return ''
    raise RuntimeError('not present')


def prepare(tmp_path):
  for name, original in [('usb0', '0'), ('wlan0', '0')]:
    (tmp_path / name).mkdir()
    (tmp_path / name / 'forwarding').write_text(original)
  return Firewall(), tmp_path / 'journal.json'


def test_cleanup_and_idempotence(tmp_path):
  fw, journal = prepare(tmp_path)
  router = internet.Router(fw, tmp_path, journal)
  router.apply('usb0', 'wlan0', '192.168.60.12', ['1.1.1.1'])
  count = len(fw.calls)
  router.apply('usb0', 'wlan0', '192.168.60.12', ['1.1.1.1'])
  assert len(fw.calls) == count
  assert (tmp_path / 'usb0/forwarding').read_text().strip() == '1'
  router.close()
  assert not fw.chains and not fw.hooks and not journal.exists()
  assert (tmp_path / 'wlan0/forwarding').read_text().strip() == '0'


def test_crash_recovery_preserves_original_disabled_forwarding(tmp_path):
  fw, journal = prepare(tmp_path)
  router = internet.Router(fw, tmp_path, journal)
  router.apply('usb0', 'wlan0', '192.168.60.12', ['1.1.1.1'])
  recovered = internet.Router(fw, tmp_path, journal)
  assert not fw.chains and not fw.hooks
  recovered.apply('usb0', 'wlan0', '192.168.60.12', ['1.1.1.1'])
  assert ['-i', 'wlan0', '-j', 'DROP'] in fw.chains['filter', internet.CHAIN]
  recovered.close()


def test_apply_failure_restores_network(tmp_path):
  fw, journal = prepare(tmp_path)
  def fail(args):
    if '-A' in args and 'MASQUERADE' in args:
      raise RuntimeError('failed NAT installation')
    return fw(args)
  router = internet.Router(fail, tmp_path, journal)
  with pytest.raises(RuntimeError):
    router.apply('usb0', 'wlan0', '192.168.60.12', ['1.1.1.1'])
  assert not fw.chains and not fw.hooks
  assert (tmp_path / 'wlan0/forwarding').read_text().strip() == '0'
