from __future__ import annotations

import asyncio
import struct

from openpilot.cereal import log
from openpilot.selfdrive.carrot.server.features.viz_can import (CAN_NAME, SERVICE_INTERVALS, WANTED, VizHub, extract, mux,
                                                            pack_raw, pack_raw_fast, pack_records)


def _unpack(payload: bytes):
  assert payload[:3] == b'VC\x02'
  count = struct.unpack_from('<H', payload, 3)[0]
  assert len(payload) == 5 + 14 * count
  out = []
  for i in range(count):
    bus, addr, age, n = struct.unpack_from('<BHHB', payload, 5 + 14 * i)
    data = payload[5 + 14 * i + 6:5 + 14 * i + 14]
    assert data[n:] == b'\0' * (8 - n)
    out.append((bus, addr, age, data[:n]))
  return out


def _can_event(mono, frames):
  msg = log.Event.new_message(logMonoTime=mono, valid=True)
  msg.init('can', len(frames))
  for i, (addr, src, dat) in enumerate(frames):
    msg.can[i].address = addr; msg.can[i].src = src; msg.can[i].dat = dat
  return msg.to_bytes()


def test_records_keep_wanted_frames_with_age():
  now = 10_000_000_000
  batches = [
    (now - 30_000_000, [(0x421, b'\x01' * 8, 1), (0x421, b'\x02' * 8, 0), (0x184, b'\x00' * 8, 1)]),
    (now - 5_000_000, [(0x510, b'\x03' * 3, 2), (0x3FC, b'\x04' * 8, 0), (0x461, b'\x05' * 8, 129), (0x135, b'\x06' * 12, 1)]),
  ]
  assert _unpack(pack_records(batches, now)) == [(1, 0x421, 30, b'\x01' * 8), (2, 0x510, 5, b'\x03' * 3), (0, 0x3FC, 5, b'\x04' * 8)]
  assert pack_records([(0, [(0x184, b'\x00' * 8, 1)])], 0) is None


def test_fast_path_matches_pycapnp_path():
  raws = [
    _can_event(1_000_000, [(0x421, 1, b'\x11' * 8), (0x184, 1, b'\x22' * 8), (0x510, 2, b'\x33' * 3), (0x427, 0, b'\x44' * 8)]),
    _can_event(2_000_000, []),
    _can_event(3_000_000, [(0x135, 1, b''), (0x3FC, 0, b'\x55' * 8), (0x352, 1, b'\x66' * 12), (0x461, 129, b'\x77' * 8)]),
  ]
  now = 9_000_000
  ok, fast = pack_raw_fast(raws, now)
  assert ok
  slow = pack_records(extract(raws, log), now)
  assert fast == slow
  assert _unpack(fast) == [(1, 0x421, 8, b'\x11' * 8), (2, 0x510, 8, b'\x33' * 3), (1, 0x135, 6, b''), (0, 0x3FC, 6, b'\x55' * 8)]


def test_unexpected_layout_falls_back():
  raw = _can_event(5, [(0x421, 1, b'\x01' * 8)])
  ok, _ = pack_raw_fast([raw[:-8]], 10)                 # truncated message
  assert not ok
  assert pack_raw([raw], log, 5) == pack_records(extract([raw], log), 5)


def test_wanted_sets_cover_live_view_inputs():
  assert {0x420, 0x421, 0x426, 0x427, 0x441, 0x446, 0x350, 0x352, 0x354, 0x355, 0x135, 0x461, 0x474} <= WANTED[1]
  assert 0x3FC in WANTED[0] and 0x510 in WANTED[2]


class _Sock:
  def __init__(self, name, feed):
    self.name, self.feed, self.reads = name, feed, 0
  def receive(self, non_blocking=True):     # like msgq SubSocket: no close() method
    self.reads += 1
    return self.feed(self.name)


class _Messaging:
  def __init__(self, feed, can_raws):
    self.feed, self.can_raws, self.socks, self.drains = feed, can_raws, {}, 0
  def sub_sock(self, name, conflate=False):
    self.socks[name] = _Sock(name, self.feed)
    return self.socks[name]
  def drain_sock_raw(self, sock):
    self.drains += 1
    out, self.can_raws[:] = list(self.can_raws), []
    return out


class _WS:
  def __init__(self):
    self.closed, self.sent = False, []
  async def send_bytes(self, data):
    self.sent.append(data)


def _demux(frame):
  n = frame[0]
  return frame[1:1 + n].decode(), frame[1 + n:]


def test_viz_hub_multiplexes_services_and_can_at_fixed_cadence():
  can = [_can_event(1_000_000, [(0x421, 1, b'\x11' * 8)])]
  fake = _Messaging(lambda name: None if name == 'radarState' else name.encode(), can)   # radarState silent
  hub, viz, raw_can = VizHub(fake, log), _WS(), _WS()

  async def run():
    await hub.register(viz, 'viz'); await hub.register(raw_can, 'can')
    await asyncio.sleep(0.45)
    hub.unregister(viz); hub.unregister(raw_can)
    await asyncio.wait_for(hub.task, 1.0)
  asyncio.run(run())

  names = [_demux(f)[0] for f in viz.sent]
  assert names.count(CAN_NAME) == 1 and raw_can.sent == [_demux(viz.sent[names.index(CAN_NAME)])[1]]
  assert 'radarState' not in names and 4 <= names.count('carState') <= 6 and 1 <= names.count('selfdriveState') <= 2
  # a silent service is still read only at its own cadence (no fast retry loop)
  assert fake.socks['radarState'].reads <= 0.45 / SERVICE_INTERVALS['radarState'] + 1
  assert mux('carState', b'xy') == b'\x08carStatexy'


def test_viz_hub_slows_down_when_the_car_is_off():
  fake = _Messaging(lambda name: None, [])                 # nothing published, no CAN
  hub, viz = VizHub(fake, log), _WS()

  async def run():
    await hub.register(viz, 'viz')
    await asyncio.sleep(3.6)
    hub.unregister(viz)
    await asyncio.wait_for(hub.task, 1.0)
  asyncio.run(run())

  # ~2 s at 10 Hz, then 2 Hz: far fewer reads than 3.6 s at 10 Hz (36); CAN is not read at all once idle
  assert viz.sent == [] and fake.socks['carState'].reads <= 26
  assert fake.drains <= 22 and fake.socks['modelV2'].reads <= 22
