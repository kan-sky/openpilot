"""Bounded, data-only link history; never feeds display or vehicle state."""
from collections import deque
import copy
import math
import os
import subprocess
import time
import uuid

from openpilot.selfdrive.navdy.feedback import feedback_shape

PUBLIC_ERRORS = frozenset((
  'Navdy Bluetooth response deadline', 'Navdy Bluetooth disconnected',
  'Navdy Bluetooth disconnected during write', 'Navdy wireless v2 response deadline',
  'Navdy wireless v2 transmit deadline', 'Navdy wireless session lease expired',
  'Navdy snapshot expired before transmission', 'Navdy wireless snapshot predates its lease',
  'Navdy wireless session sequence exhausted', 'Navdy wireless link is not connected',
  'Navdy wireless v2 acknowledgement mismatch', 'Navdy wireless v2 invalid acknowledgement age',
  'Navdy wireless v2 response header/bound', 'Navdy wireless v2 unsolicited response overflow',
  'Navdy wireless acknowledgement mismatch', 'Navdy wireless size/checksum mismatch',
  'Invalid Navdy compressed response', 'Incomplete or trailing Navdy compressed response',
  'Unknown Navdy wireless encoding', 'Unsupported Navdy wireless handshake',
  'Navdy wireless v1 service not advertised', 'Ambiguous Navdy Bluetooth service',
  'Navdy requires an encrypted paired link', 'Configured Navdy is not paired; pairing is never automatic',
  'Native Bluetooth RFCOMM is unavailable on this OS', 'Navdy closed its socket',
  'Navdy USB not found', 'Multiple Navdy USB devices', 'Navdy service failed to start',
  'ADB inventory unavailable',
  'Jetson Navdy relay response timed out', 'Jetson Navdy relay write timed out',
))
ERROR_TYPES = (TimeoutError, ConnectionResetError, ConnectionAbortedError, ConnectionRefusedError, BrokenPipeError,
               ConnectionError, PermissionError, FileNotFoundError, OSError, ValueError,
               subprocess.TimeoutExpired, subprocess.CalledProcessError, subprocess.SubprocessError)
PHASES = ('connect', 'payload', 'exchange', 'feedback', 'status')
COUNTER_MAX = (1 << 63) - 1
LOCAL_EVENTS = frozenset(('bridge_started', 'connect_attempt', 'connect_failed', 'transport_closed',
                          'retry_scheduled', 'periodic_status'))


def public_error_type(error):
  # Exception subclasses can have application-controlled names, too.
  return next((kind.__name__ for kind in ERROR_TYPES if isinstance(error, kind)), 'Exception')


def public_error(error):
  # Never persist subprocess command lines, paths, peer addresses or payloads.
  if isinstance(error, subprocess.TimeoutExpired):
    return 'subprocess_timeout'
  if isinstance(error, subprocess.SubprocessError):
    return 'subprocess_failed'
  if isinstance(error, OSError) and isinstance(error.errno, int) and 0 < error.errno < 4096:
    return os.strerror(error.errno)
  message = str(error)
  return message if message in PUBLIC_ERRORS else public_error_type(error)


def selected(source, keys):
  if not isinstance(source, dict):
    return {}
  enums = {'owner': ('none', 'bluetooth', 'usb', 'jetson', 'unknown'), 'state': ('onroad', 'offroad', 'unknown')}
  return {key: value for key in keys if (value := source.get(key)) is not None
          and (type(value) in (bool, int) and abs(value) <= COUNTER_MAX
               or type(value) is float and math.isfinite(value) and abs(value) <= COUNTER_MAX
               or type(value) is str and value in enums.get(key, ()))}


class LinkDiagnostics:
  def __init__(self, emit, journal=None):
    self.emit = emit
    self.journal = journal
    self.run_id = uuid.uuid4().hex
    self.events = deque(maxlen=16)
    self.sequence = 0
    self.last_failure = None
    self.receiver = None
    self.source_state = None
    self.source_at = None
    self.receiver_at = None
    self.next_status = 0.0
    self.phase = None
    self.phase_started = 0.0
    self.timings = {}
    self.exchanges = 0
    self.replies = 0
    self.feedback_stats = {key: {'count': 0, 'lastReceivedAt': None}
                           for key in ('noObservation', 'observation', 'incomplete')}

  def record(self, kind, *, local_only=False, **fields):
    self.sequence = min(COUNTER_MAX, self.sequence + 1)
    event = {'runId': self.run_id, 'eventSeq': self.sequence, 'at': time.monotonic(),
             'wallTime': time.time(), 'kind': kind, **fields}  # noqa: TID251 -- cross-device correlation only, never freshness
    if not local_only and kind not in LOCAL_EVENTS:
      self.events.append(event)
      try:
        # Keep cloudlog's existing event cadence; new periodic/local tracing is
        # neither uploaded nor allowed to evict the original in-memory history.
        self.emit('navdy_link_event', **event)
      except Exception:
        # Diagnostics must not tear down a healthy display link.
        pass
    if self.journal is not None:
      try:
        self.journal.submit(event)
      except Exception:
        pass
    return event

  def failure(self, *, phase, transport, error, connected_at, wireless):
    self.last_failure = self.record('failure', phase=phase, transport=transport,
                                    phaseMs=max(0., time.monotonic() - self.phase_started) * 1000 if self.phase else None,
                                    errorType=public_error_type(error), reason=public_error(error),
                                    connectedAt=connected_at, wireless=self.metrics(wireless),
                                    sourceState=copy.deepcopy(self.source_state),
                                    receiver=copy.deepcopy(self.receiver))

  @staticmethod
  def metrics(wireless):
    return selected(wireless, ('protocol', 'sequence', 'rttMs', 'receiverAgeMs', 'displayStatus',
                               'inFlight', 'skippedSnapshots', 'ackAgeMs', 'ackPollDelayMs', 'txPendingBytes',
                               'txAgeMs', 'lastTxMs', 'partialWriteCount', 'blockedWriteCount'))

  def sources(self, sources, payload, *, can_valid=True, can_timeout=False):
    missing = {key: value['reason'] for key, value in sources.items()
               if key in ('deviceState', 'pandaStates', 'carState', 'selfdriveState') and not value['usable']}
    if sources['carState']['usable'] and (not can_valid or can_timeout):
      missing['carState'] = 'can_invalid' if not can_valid else 'can_timeout'
    state = {'missing': missing, 'power': payload['displayPower'], 'ambient': payload['ambientState']}
    self.source_at = time.monotonic()
    if state != self.source_state:
      self.record('source_baseline' if self.source_state is None else 'source_changed', state=state)
      self.source_state = state

  def feedback(self, response, *, transport, received_at, elapsed_ms, wireless):
    sample = self.feedback_stats[feedback_shape(response)]
    sample['count'] = min(COUNTER_MAX, sample['count'] + 1)
    if type(received_at) in (int, float) and math.isfinite(received_at) and 0 <= received_at <= COUNTER_MAX:
      sample['lastReceivedAt'] = max(sample['lastReceivedAt'] or 0, received_at)
    if not isinstance(response, dict):
      return
    current = {'wireless': selected(response.get('wireless'), ('session', 'owner', 'clears', 'usbUpdates', 'receivedSeq', 'appliedSeq')),
               'power': selected(response.get('displayPower'), ('state', 'fresh', 'generation', 'invalidations'))}
    if not any(current.values()):
      return
    def changes(state):
      if state is None:
        return None
      return selected(state['wireless'], ('session', 'owner', 'clears')), state['power']

    if changes(current) != changes(self.receiver):
      # First cumulative counters are a baseline, never newly observed failures.
      self.record('receiver_baseline' if self.receiver is None else 'receiver_changed',
                  transport=transport, receivedAt=received_at, previous=self.receiver,
                  current=current, roundTripMs=elapsed_ms, batchLatestWireless=self.metrics(wireless))
    self.receiver = current
    self.receiver_at = received_at

  def begin_phase(self, phase):
    self.end_phase()
    self.phase, self.phase_started = phase, time.monotonic()

  def end_phase(self):
    if self.phase in PHASES:
      elapsed = max(0., time.monotonic() - self.phase_started) * 1000
      sample = self.timings.setdefault(self.phase, {'count': 0, 'totalMs': 0., 'maxMs': 0.})
      sample['count'] = min(COUNTER_MAX, sample['count'] + 1)
      sample['totalMs'] = min(COUNTER_MAX, sample['totalMs'] + elapsed)
      sample['maxMs'] = min(COUNTER_MAX, max(sample['maxMs'], elapsed))
    self.phase = None

  def exchanged(self, reply_count):
    self.exchanges = min(COUNTER_MAX, self.exchanges + 1)
    self.replies = min(COUNTER_MAX, self.replies + reply_count)

  def periodic(self, *, connected, transport, bluetooth_only, retry_in, wireless):
    now = time.monotonic()
    if now < self.next_status:
      return
    self.next_status = now + 5.
    self.record('periodic_status', connected=connected, transport=transport, bluetoothOnly=bluetooth_only,
                retryInSeconds=max(0., retry_in), sourceState=copy.deepcopy(self.source_state), sourceAt=self.source_at,
                receiver=copy.deepcopy(self.receiver), receiverAt=self.receiver_at, wireless=self.metrics(wireless),
                timings=self.timings, exchanges=self.exchanges, replies=self.replies, journal=self.journal_stats(),
                feedback=copy.deepcopy(self.feedback_stats))
    self.timings, self.exchanges, self.replies = {}, 0, 0

  def journal_stats(self):
    if self.journal is not None:
      try:
        return self.journal.stats()
      except Exception:
        pass
    return {}

  def close(self):
    if self.journal is not None:
      try:
        self.journal.close()
      except Exception:
        pass

  def snapshot(self):
    return copy.deepcopy({'runId': self.run_id, 'eventSeq': self.sequence,
                          'lastFailure': self.last_failure, 'events': list(self.events), 'journal': self.journal_stats(),
                          'feedback': self.feedback_stats})
