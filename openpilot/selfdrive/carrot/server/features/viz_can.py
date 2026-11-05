"""Data streams for the phone live-view app. The phone renders everything; the comma only forwards data.

/ws/viz      one socket with everything the app needs, multiplexed like /ws/raw_multiplex:
             [len(name)][name utf-8][payload]
             - cereal services as raw Event capnp, read from conflated sockets once per 100 ms tick
               (carState/modelV2/radarState 10 Hz, carControl 5 Hz, selfdriveState 2.5 Hz; the app smooths lanes).
             - name "vizCan": the CAN batch below.
/ws/viz_can  the CAN batch alone.

CAN batch: the GM frames the live view needs that are not in any cereal service (front-camera object tracks and types,
lead brake/turn lamps, stock lane-marking types, raw radar targets, ACC gap, outside temperature), every 100 ms:

  b'VC' u8 version=2, u16 count (LE), then count x 14-byte records:
  u8 bus, u16 addr LE, u16 age_ms LE, u8 len, data[8] (zero padded; frames longer than 8 bytes are not sent)

CPU (measured on the comma with a stored drive log, 5,100 frames/s): touching every frame through pycapnp costs
3.2 % of one core at 50 ms batches. pack_raw() reads the drained `can` events' capnp bytes directly with a handful of
numpy calls per batch (bus/address of all frames at once, data only for wanted frames): 1.5 % at 50 ms, 0.9 % at the
100 ms used here. Any layout it does not expect falls back to pycapnp.

Wakeups dominate on the comma CPU: an asyncio task that only sleeps 50 ms costs 1.3 % of one core there. So all work
shares one 100 ms tick. After 2 s without any data (car off) the tick slows to 500 ms, CAN is not read at all and only
the latest carState is checked to notice the car starting. The shared raw hub, by
contrast, polls carState at 60 Hz and retries silent services every 10 ms (+3.2 % with the car off).
Everything runs only while a client is connected. Read-only: never publishes cereal messages and never sends CAN.
"""
from __future__ import annotations

import asyncio
import json
import struct
import time
from typing import Any, Iterable

import numpy as np
from aiohttp import WSMsgType, web

VERSION = 2
INTERVAL_S = 0.1                 # CAN batch period = tick; camera objects come at 25 Hz
TICK_S = 0.1
IDLE_TICK_S = 0.5
IDLE_AFTER_S = 2.0
SERVICE_INTERVALS = {'carState': 0.1, 'modelV2': 0.1, 'radarState': 0.1, 'carControl': 0.2, 'selfdriveState': 0.4}
CAN_NAME = 'vizCan'
MAX_FRAMES = 4000
WANTED: dict[int, frozenset[int]] = {
  # bus 1 (object/camera): lane info 0x350-0x356, camera objects 0x420-0x42C / 0x440-0x44C, radar 0x460-0x47F, LHT 0x135
  1: frozenset(set(range(0x350, 0x357)) | set(range(0x420, 0x42D)) | set(range(0x440, 0x44D)) | set(range(0x460, 0x480)) | {0x135}),
  0: frozenset({0x140, 0x142, 0x3FC, 0x1F5}),   # high beam, lamps, outside air temperature, gear
  2: frozenset({0x510}),                         # ACC time gap / distance
}
ALL_ADDRS = frozenset().union(*WANTED.values())
REC = np.dtype([('bus', 'u1'), ('addr', '<u2'), ('age', '<u2'), ('len', 'u1'), ('dat', 'u1', (8,))])
_WANTED_KEYS = np.array(sorted((src << 32) | a for src, addrs in WANTED.items() for a in addrs), dtype=np.uint64)
_LANE8 = np.arange(8)


def _payload(rec: np.ndarray) -> bytes | None:
  if len(rec) == 0:
    return None
  rec = rec[:MAX_FRAMES]
  return b'VC' + bytes([VERSION]) + struct.pack('<H', len(rec)) + rec.tobytes()


# ---------- reference path (pycapnp) ----------
_FIELDS: tuple[Any, Any, Any] | None = None


def extract(raws: Iterable[bytes], log_module: Any) -> list[tuple[int, list[tuple[int, bytes, int]]]]:
  """Raw `can` events -> [(logMonoTime, [(address, dat, src), ...])] for wanted (bus, address) frames."""
  global _FIELDS
  out = []
  for raw in raws:
    with log_module.Event.from_bytes(raw) as event:
      frames = event.can
      sel = []
      if len(frames):
        if _FIELDS is None:
          fields = frames[0].schema.fields
          _FIELDS = (fields['address'], fields['dat'], fields['src'])
        addr_f, dat_f, src_f = _FIELDS
        for frame in frames:
          address = frame._get_by_field(addr_f)
          if address in ALL_ADDRS:
            src = frame._get_by_field(src_f)
            if address in WANTED.get(src, ()):
              sel.append((address, bytes(frame._get_by_field(dat_f)), src))
      out.append((event.logMonoTime, sel))
  return out


def pack_records(batches: Iterable[Any], now_ns: int) -> bytes | None:
  """[(logMonoTime_ns, [(address, dat, src), ...]), ...] -> wire payload (or None if nothing to send)."""
  rows = []
  for nanos, frames in batches:
    age = max(0, min(65535, (now_ns - int(nanos)) // 1_000_000))
    for address, dat, src in frames:
      if address in WANTED.get(src, ()) and len(dat) <= 8:
        rows.append((src, address, age, len(dat), tuple(bytes(dat).ljust(8, b'\0'))))
  return _payload(np.array(rows, dtype=REC)) if rows else None


# ---------- fast path (direct capnp reads) ----------
_LUT = np.zeros(4 * 2048, dtype=bool)                # (bus << 11 | 11-bit address) -> wanted
for _src, _addrs in WANTED.items():
  _LUT[[(_src << 11) | a for a in _addrs]] = True
_Q = struct.Struct('<Q')


def _s30(v: int) -> int:
  return v - (1 << 30) if v & (1 << 29) else v


def _can_list(raw: bytes) -> tuple[int, int, int] | None:
  """Single-segment `can` Event whose List(CanData) has 1 data + 1 pointer word per element
  -> (logMonoTime, first element word, count), else None."""
  if len(raw) < 24 or len(raw) % 8 or raw[0:4] != b'\0\0\0\0':
    return None
  nwords = (len(raw) - 8) // 8
  if int.from_bytes(raw[4:8], 'little') != nwords:
    return None
  root = _Q.unpack_from(raw, 8)[0]
  if root & 3 != 0:
    return None
  ev = 1 + _s30((root & 0xFFFFFFFF) >> 2)
  ev_dw = (root >> 32) & 0xFFFF
  pi = ev + ev_dw                                   # union payload (List(CanData)) is pointer slot 0
  if ev_dw < 1 or pi >= nwords:
    return None
  log_mono = _Q.unpack_from(raw, 8 + 8 * ev)[0]
  lp = _Q.unpack_from(raw, 8 + 8 * pi)[0]
  if lp == 0:
    return log_mono, 0, 0
  if lp & 3 != 1 or ((lp >> 32) & 7) != 7:          # list of inline composite structs
    return None
  tag_i = pi + 1 + _s30((lp & 0xFFFFFFFF) >> 2)
  if not 0 <= tag_i < nwords:
    return None
  tag = _Q.unpack_from(raw, 8 + 8 * tag_i)[0]
  count = _s30((tag & 0xFFFFFFFF) >> 2)
  if count < 0 or (tag >> 32) != 0x0001_0001 or tag_i + 1 + 2 * count > nwords:
    return None
  return log_mono, tag_i + 1, count


def pack_raw_fast(raws: list[bytes], now_ns: int) -> tuple[bool, bytes | None]:
  """-> (ok, payload). ok=False means an unexpected layout: use the pycapnp path for this batch."""
  starts, counts, ages = [], [], []
  base = 0
  for raw in raws:
    head = _can_list(raw)
    if head is None:
      return False, None
    log_mono, first, count = head
    if count:
      starts.append(base + 1 + first)               # +1: the segment-table word in front of each message
      counts.append(count)
      ages.append(max(0, min(65535, (now_ns - log_mono) // 1_000_000)))
    base += len(raw) // 8
  if not counts:
    return True, None
  joined = b''.join(raws)
  words = np.frombuffer(joined, dtype='<u8')
  blk = np.concatenate([words[st:st + 2 * c] for st, c in zip(starts, counts)])
  w16 = blk.view('<u2').reshape(-1, 8)              # per frame: addr lo/hi, busTime, src | dat pointer (4 x u16)
  addr, src = w16[:, 0], w16[:, 3] & 0xFF
  hit = (w16[:, 1] == 0) & (addr < 2048) & (src < 4)
  hit[hit] = _LUT[(src[hit] << 11) | addr[hit]]
  hits = np.flatnonzero(hit)
  if len(hits) == 0:
    return True, None
  cum = np.cumsum(counts)
  msg = np.searchsorted(cum, hits, side='right')
  local = hits - (cum - counts)[msg]
  dp_i = np.asarray(starts)[msg] + 2 * local + 1     # absolute word index of each hit's dat pointer
  dp = blk[2 * hits + 1]
  null = dp == 0
  if not np.all(null | (((dp & np.uint64(3)) == 1) & (((dp >> np.uint64(32)) & np.uint64(7)) == 2))):
    return False, None
  off = ((dp & np.uint64(0xFFFFFFFF)) >> np.uint64(2)).astype(np.int64)
  off -= (off >= (1 << 29)) * (1 << 30)
  n = np.where(null, 0, (dp >> np.uint64(35)).astype(np.int64))
  start = (dp_i + 1 + off) * 8
  if np.any(~null & ((start < 0) | (start + n > len(joined)))):
    return False, None
  keep = n <= 8
  if not keep.all():
    hits, start, n, msg = hits[keep], start[keep], n[keep], msg[keep]
  buf = np.frombuffer(joined, dtype=np.uint8)
  dat = buf[np.minimum(start[:, None] + _LANE8, len(buf) - 1)]
  dat[_LANE8[None, :] >= n[:, None]] = 0
  rec = np.empty(len(n), dtype=REC)
  rec['bus'] = src[hits]; rec['addr'] = addr[hits]; rec['len'] = n
  rec['age'] = np.asarray(ages)[msg]
  rec['dat'] = dat
  return True, _payload(rec)


def pack_raw(raws: list[bytes], log_module: Any, now_ns: int) -> bytes | None:
  ok, payload = pack_raw_fast(raws, now_ns)
  return payload if ok else pack_records(extract(raws, log_module), now_ns)


def _release(sock: Any) -> None:
  """msgq SubSocket has no close(); dropping the last reference frees it. Call close() only where it exists."""
  close = getattr(sock, 'close', None)
  if close is not None:
    try:
      close()
    except Exception:
      pass


def mux(name: str, payload: bytes) -> bytes:
  raw = name.encode()
  return bytes([len(raw)]) + raw + payload


class VizHub:
  """One poll loop shared by all live-view clients; it exists only while a client is connected."""

  def __init__(self, messaging: Any = None, log_module: Any = None) -> None:
    self.clients: dict[Any, str] = {}            # ws -> 'viz' (services + CAN) | 'can' (CAN batch only)
    self.task: asyncio.Task | None = None
    self._messaging, self._log = messaging, log_module

  async def register(self, ws: Any, kind: str) -> None:
    self.clients[ws] = kind
    if self.task is None or self.task.done():
      self.task = asyncio.create_task(self._loop())

  def unregister(self, ws: Any) -> None:
    self.clients.pop(ws, None)

  async def _send(self, kind: str, data: bytes) -> None:
    for ws, k in list(self.clients.items()):
      if k != kind:
        continue
      if ws.closed:
        self.clients.pop(ws, None)
        continue
      try:
        await ws.send_bytes(data)
      except Exception:
        self.clients.pop(ws, None)

  async def _loop(self) -> None:
    messaging, log = self._messaging, self._log
    if messaging is None:
      import openpilot.cereal.messaging as messaging
    if log is None:
      from openpilot.cereal import log
    socks: dict[str, Any] = {}
    next_read: dict[str, float] = {}
    can_sock = None
    last_data = time.monotonic()
    try:
      while self.clients:
        now = time.monotonic()
        idle = now - last_data > IDLE_AFTER_S
        got = False
        want_services = any(k == 'viz' for k in self.clients.values())
        for svc, interval in SERVICE_INTERVALS.items():
          # car off: only the latest carState is checked (cheap, conflated) to notice the car starting
          if (not want_services and svc != 'carState') or (idle and svc != 'carState') or now < next_read.get(svc, 0.0):
            continue
          next_read[svc] = now + interval - 0.02       # fixed cadence even when the service is silent
          sock = socks.get(svc)
          if sock is None:
            sock = socks[svc] = messaging.sub_sock(svc, conflate=True)
          payload = sock.receive(non_blocking=True)
          if payload:
            got = True
            if want_services:
              await self._send('viz', mux(svc, payload))
        if idle and not got:
          if can_sock is not None:                     # stop reading CAN entirely while the car is off
            _release(can_sock); can_sock = None
        else:
          if can_sock is None:
            can_sock = messaging.sub_sock('can', conflate=False)
          raws = messaging.drain_sock_raw(can_sock)
          payload = pack_raw(raws, log, time.monotonic_ns()) if raws else None
          if payload:
            got = True
            await self._send('can', payload)
            await self._send('viz', mux(CAN_NAME, payload))
        if got:
          last_data = now
        tick = IDLE_TICK_S if now - last_data > IDLE_AFTER_S else TICK_S
        await asyncio.sleep(max(0.005, tick - (time.monotonic() - now)))
    finally:
      for sock in [can_sock, *socks.values()]:
        if sock is not None:
          _release(sock)
      socks.clear(); can_sock = None


async def _serve(request: web.Request, kind: str, hello: dict) -> web.WebSocketResponse:
  hub: VizHub = request.app['viz_hub']
  ws = web.WebSocketResponse(heartbeat=20, compress=False)
  await ws.prepare(request)
  await ws.send_str(json.dumps(hello))
  await hub.register(ws, kind)
  try:
    async for msg in ws:
      if msg.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.ERROR):
        break
  finally:
    hub.unregister(ws)
  return ws


async def ws_viz(request: web.Request) -> web.WebSocketResponse:
  return await _serve(request, 'viz', {'type': 'hello', 'stream': 'viz', 'version': 1, 'services': SERVICE_INTERVALS,
                                       'can': CAN_NAME, 'canVersion': VERSION, 'canIntervalMs': int(INTERVAL_S * 1000)})


async def ws_viz_can(request: web.Request) -> web.WebSocketResponse:
  return await _serve(request, 'can', {'type': 'hello', 'stream': 'viz_can', 'version': VERSION, 'intervalMs': int(INTERVAL_S * 1000)})


def register(app: web.Application) -> None:
  app['viz_hub'] = VizHub()
  app.router.add_get('/ws/viz', ws_viz)
  app.router.add_get('/ws/viz_can', ws_viz_can)
