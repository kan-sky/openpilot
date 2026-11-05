import json
import socket
import subprocess
from types import SimpleNamespace as NS

import pytest

import wifi_protocol as wire
import wifi_publisher as publisher


def uid(number):
  return f'00000000-0000-4000-8000-{number:012d}'


def profile_fields(ssid='Test Wi-Fi', mode='infrastructure', hidden='no', security='wpa-psk', password='test-only-password'):
  return '\n'.join((ssid, mode, hidden, security, password)).rstrip('\n')


def source(monkeypatch, profiles, active, changed=None):
  queries = []
  active_queries = []

  def nm(*args):
    if args == ('-t', '-f', 'UUID', 'connection', 'show', '--active'):
      result = changed if active_queries and changed is not None else active
      active_queries.append(True)
      return '\n'.join(result)
    if args == ('-t', '-f', 'UUID,TYPE', 'connection', 'show'):
      return '\n'.join(f'{key}:802-11-wireless' for key in profiles)
    assert args[0] == '--show-secrets'
    queries.append(args[-1])
    result = profiles[args[-1]]
    if isinstance(result, Exception):
      raise result
    return result

  monkeypatch.setattr(publisher, 'nm', nm)
  return queries


def test_current_wifi_precedes_saved_networks(monkeypatch):
  queries = source(monkeypatch, {uid(1): profile_fields('Old Wi-Fi'), uid(2): profile_fields('Current Wi-Fi')}, [uid(2)])
  profiles = publisher.collect()
  assert queries == [uid(2), uid(1)]
  assert [p['ssid'] for p in profiles] == ['Current Wi-Fi', 'Old Wi-Fi']
  assert [p['active'] for p in profiles] == [True, False]


@pytest.mark.parametrize('failed', [profile_fields(password=''), profile_fields(security='wpa-eap'),
                                  'too\nmany\nfields\nin\nthis\nresponse', RuntimeError('query unavailable'), OSError('query failed'),
                                  subprocess.TimeoutExpired('nmcli', 2)])
def test_unreadable_saved_network_does_not_block_active_wifi(monkeypatch, failed):
  source(monkeypatch, {uid(1): failed, uid(2): profile_fields('Current Wi-Fi')}, [uid(2)])
  profiles = publisher.collect()
  assert len(profiles) == 1 and profiles[0]['id'] == uid(2) and profiles[0]['active']


@pytest.mark.parametrize('failed', [profile_fields(password=''), profile_fields(security='wpa-eap'),
                                  'too\nmany\nfields\nin\nthis\nresponse', RuntimeError('private detail'),
                                  subprocess.TimeoutExpired('nmcli', 2)])
def test_unreadable_active_wifi_does_not_publish_wrong_fallback(monkeypatch, failed):
  source(monkeypatch, {uid(1): profile_fields('Old Wi-Fi'), uid(2): failed}, [uid(2)])
  with pytest.raises(RuntimeError, match='^Active Wi-Fi profile unavailable$'):
    publisher.collect()


def test_hotspot_is_not_imported_as_client(monkeypatch):
  source(monkeypatch, {uid(1): profile_fields(mode='ap'), uid(2): profile_fields()}, [uid(1)])
  assert [p['id'] for p in publisher.collect()] == [uid(2)]


@pytest.mark.parametrize('security,password', [('', ''), ('wpa-psk', 'test-password'), ('sae', 'test-password'),
                                             ('wpa-psk', 'a' * 64)])
def test_supported_current_networks_keep_unicode_and_spaces(monkeypatch, security, password):
  ssid = '\ud14c\uc2a4\ud2b8 Wi-Fi'
  source(monkeypatch, {uid(1): profile_fields(ssid, hidden='yes', security=security, password=password)}, [uid(1)])
  profile = publisher.collect()[0]
  assert profile['ssid'] == ssid and profile['hidden'] and profile['active']
  assert profile['password'] == password and profile['security'] == (security or 'open')


def test_current_wifi_is_included_with_more_than_eight_saved(monkeypatch):
  queries = source(monkeypatch, {uid(n): profile_fields(str(n)) for n in range(12)}, [uid(11)])
  profiles = publisher.collect()
  assert len(profiles) == len(queries) == 8 and profiles[0]['id'] == uid(11)


def test_switch_during_collection_retries_instead_of_sending_stale_active(monkeypatch):
  source(monkeypatch, {uid(1): profile_fields(), uid(2): profile_fields()}, [uid(1)], changed=[uid(2)])
  with pytest.raises(RuntimeError, match='changed during'):
    publisher.collect()


def test_wifi_change_marks_the_new_profile_active(monkeypatch):
  profiles = {uid(1): profile_fields('First'), uid(2): profile_fields('Second')}
  source(monkeypatch, profiles, [uid(1)])
  first = publisher.collect()
  source(monkeypatch, profiles, [uid(2)])
  second = publisher.collect()
  assert first[0]['id'] == uid(1) and second[0]['id'] == uid(2)
  assert first != second and first[0]['active'] and second[0]['active']


def test_packet_roundtrip_keeps_credentials_private(monkeypatch, tmp_path):
  source(monkeypatch, {uid(1): profile_fields('Current Wi-Fi')}, [uid(1)])
  packet, control = tmp_path / 'wifi.packet', tmp_path / 'bootstrap.json'
  monkeypatch.setattr(wire, 'PACKET', packet)
  monkeypatch.setattr(wire, 'CONTROL', control)
  value = dict(version=1, owner='0' * 64, profiles=publisher.collect(), onroad=False)
  wire.receive(json.dumps(value).encode())
  assert wire.read_packet()[1] == value
  assert packet.stat().st_mode & 0o777 == 0o600
  assert control.stat().st_mode & 0o777 == 0o600
  public = json.loads(control.read_text())
  assert set(public) == {'onroad', 'received'}
  assert public['onroad'] is False


def test_sender_uses_existing_usb_message(monkeypatch):
  now = 100.0
  monkeypatch.setattr(wire.time, 'monotonic', lambda: now)
  # Datagram boundaries exercise the sender on macOS too; target uses SEQPACKET.
  parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
  try:
    parent.setblocking(False)
    sender = wire.Publisher.__new__(wire.Publisher)
    sender.socket = parent
    sent = []
    client = NS(t=NS(send=lambda *args: sent.append(args)), _next_seq=lambda: 42)
    value = dict(version=1, owner='0' * 64, profiles=[], onroad=False)
    payload = json.dumps(value).encode()
    child.send(wire.HEADER.pack(now) + payload)
    sender.send(client)
    assert sent == [(wire.MESSAGE, 42, [payload])]
    child.send(wire.HEADER.pack(now - 4) + payload)
    sender.send(client)
    assert len(sent) == 1
  finally:
    parent.close()
    child.close()
