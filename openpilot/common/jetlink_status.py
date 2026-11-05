"""Read-only display identity; independent of model/runtime imports."""
import json
import ipaddress
import math
from pathlib import Path
import time

from openpilot.common.jetlink_peer import classify_auto_peer, is_mac_peer, may_provision

LINK_STATUS = Path('/dev/shm/carrot-jetlink.json')
MODEL_STATUS = Path('/dev/shm/carrot-jetlink-model.json')
HOST_LABELS = {'jetson': 'jetSON', 'mac': 'MAC'}


def update_badge(value):
  if not isinstance(value, dict):
    value = {}
  state = value.get('state')
  if state == 'waiting_internet':
    return 'Jetson 업데이트 필요 · 인터넷 연결 대기', 'loading'
  if state == 'downloading':
    fraction = value.get('fraction')
    percent = f' {int(fraction * 100)}%' if isinstance(fraction, (int, float)) and 0 <= fraction <= 1 else ''
    return f'Jetson 업데이트 다운로드 중{percent}', 'loading'
  if state == 'verifying':
    return 'Jetson 업데이트 검증·적용 중', 'loading'
  if state == 'failed':
    return 'Jetson 업데이트 실패 · 재시도 대기', 'error'
  return 'Jetson 업데이트 확인 중', 'loading'


def _read(path):
  try:
    value = json.loads(path.read_text())
    return value if isinstance(value, dict) else {}
  except (OSError, ValueError):
    return {}


def host_label(peer, mode='usb'):
  if not isinstance(peer, dict):
    return 'Jetlink'
  if mode == 'auto':
    mode = classify_auto_peer(peer, 'usb') or 'auto'
  if mode in ('ios', 'android') and may_provision(peer, mode):
    return 'iOS' if mode == 'ios' else 'Android'
  host = peer.get('carrot_host')
  if isinstance(host, str) and host in HOST_LABELS:
    return HOST_LABELS[host]
  # Compatibility with the already-installed server, before host metadata.
  device = str(peer.get('device', '')).lower()
  if peer.get('backend') == 'trt' and 'orin' in device:
    return 'jetSON'
  if mode == 'usb' and is_mac_peer(peer):
    return 'MAC'
  return 'Jetlink'


def _fresh(path, now):
  return _fresh_value(_read(path), now)


def _fresh_value(value, now):
  try:
    if isinstance(value, dict) and 0 <= now - value['updated'] < 3:
      return value
  except (OSError, ValueError, KeyError, TypeError):
    pass
  return {}


def diagnostics():
  """Only current telemetry can report health or an address; HELLO is identity."""
  return _diagnostics(*_snapshot())


def _snapshot():
  now = time.monotonic()
  saved = _read(LINK_STATUS)
  return now, saved, _fresh_value(saved, now), _fresh_value(_read(MODEL_STATUS), now)


def _diagnostics(now, saved, link, model):
  if not saved.get('peer') and saved.get('state') not in ('connecting', 'loading', 'retrying', 'blocked') and not model.get('active'):
    return None
  result = {'label': host_label(saved.get('peer'), saved.get('transport', 'usb')), 'state': link.get('state', 'disconnected'),
            'severity': 'unknown', 'reason': '', 'addresses': [], 'temp_c': None,
            'active': bool(model.get('active')), 'fresh': False}
  if link.get('state') == 'updating' and result['label'] == 'jetSON':
    text, style = update_badge(link.get('host_update') or {})
    result.update(severity='error' if style == 'error' else 'warning', reason=text, fresh=True, active=False)
    return result
  model_error = str(model.get('error') or '')[:240] if not model.get('active') else ''
  if model_error:
    result.update(severity='error', reason=model_error)
  # A current waiting report confirms the optional host is absent, including
  # startup before modeld's first report. A remembered HELLO is not a fault.
  # Keep current model failures/active-session conflicts and stale link state
  # visible; only explicit, fresh absence is quiet.
  if link.get('state') == 'waiting' and not model.get('active') and not model_error:
    result.update(reason='Host not connected')
    return result
  if result['state'] in ('retrying', 'stopped', 'disconnected', 'waiting', 'blocked'):
    result.update(severity='error', reason=str(link.get('error') or 'Host connection unavailable')[:240])
    return result
  telemetry = link.get('telemetry') or {}
  health = telemetry.get('carrot_health') if isinstance(telemetry, dict) else None
  try:
    age = now - float(link['telemetry_updated']) + float(health['age_s'])
    if not 0 <= age < 3:
      return result
  except (KeyError, TypeError, ValueError):
    return result
  result.update(fresh=True, severity=health.get('severity', 'unknown'), reason=str(health.get('reason', ''))[:240])
  if result['severity'] not in ('ok', 'warning', 'error', 'unknown'):
    result['severity'] = 'unknown'
  temperature = health.get('temp_c')
  if isinstance(temperature, (int, float)) and math.isfinite(temperature) and -20 <= temperature <= 150:
    result['temp_c'] = temperature
  entries = health.get('addresses')
  for entry in (entries[:8] if isinstance(entries, list) else []):
    try:
      address = ipaddress.ip_address(entry['address'])
      if not address.is_loopback and not address.is_unspecified and not address.is_link_local:
        result['addresses'].append(str(address))
    except (KeyError, TypeError, ValueError):
      continue
  if model_error:
    result.update(severity='error', reason=model_error)
  return result


def badge():
  return display_status()[0]


def display_status():
  """Badge and explanation from one snapshot, without changing the badge API."""
  now, saved, link, model = _snapshot()
  health = _diagnostics(now, saved, link, model)
  status = _badge(link, model, health)
  return status, _detail(status, link, model, health)


def web_status():
  """Current web diagnostics from the same snapshot as the native HUD badge."""
  now, saved, link, model = _snapshot()
  health = _diagnostics(now, saved, link, model)
  status = _badge(link, model, health)
  peer = link.get('peer')
  peer = peer if isinstance(peer, dict) else {}
  telemetry = link.get('telemetry')
  telemetry = telemetry if isinstance(telemetry, dict) else {}
  fresh_health = bool(health and health.get('fresh'))
  return {
    'badge': status[0] if status else '', 'style': status[1] if status else 'unknown',
    'detail': _detail(status, link, model, health), 'state': link.get('state', 'disconnected'),
    'fresh': bool(link), 'health': health, 'active': bool(model.get('active')),
    'model': model.get('model') or link.get('model') or '',
    'sha256': model.get('sha256') or link.get('sha256') or '',
    'transport': link.get('transport', ''), 'device': peer.get('device', ''),
    'runtime': peer.get('runtime_version', ''), 'protocol': peer.get('protocol'),
    'engine_state': telemetry.get('engine_state') if fresh_health else None,
    'hud_connected': telemetry.get('carrot_hud_connected') if fresh_health else None,
    'frames_served': telemetry.get('frames_served') if fresh_health else None,
  }


def _badge(link, model, health):
  label = host_label(link.get('peer'), link.get('transport', 'usb'))
  if link.get('state') == 'updating' and label == 'jetSON':
    return update_badge(link.get('host_update') or {})
  if health:
    label = health['label']
    if health['state'] == 'waiting' and health['severity'] == 'unknown':
      return None
    if health['severity'] == 'error':
      return f'{label} ERROR', 'error'
    if health['severity'] == 'warning':
      return f'{label} HOT', 'loading'
    if label == 'jetSON' and link.get('state') == 'ready' and (not health['fresh'] or health['severity'] == 'unknown'):
      return f'{label} DATA?', 'loading'
  if model.get('active'):
    return label, 'active'
  state = link.get('state')
  if state == 'ready':
    return f'{label} READY', 'ready'
  if state == 'connecting':
    return f'{label} 연결 대기', 'loading'
  if state == 'loading':
    return f'{label} 모델 준비 중', 'loading'
  if state == 'retrying':
    return f'{label} RETRY', 'error'
  return None


def _text(value, limit=300):
  if not isinstance(value, str):
    return ''
  return ' '.join(''.join(c if c.isprintable() else ' ' for c in value[:limit]).split())


def _detail(status, link, model, health):
  if status is None or status[1] not in ('loading', 'error'):
    return ''
  model_error = _text(model.get('error')) if not model.get('active') else ''
  if not link:
    prefix = f'Model: {model_error} | ' if model_error else ''
    return prefix + 'Link status missing or stale (3s timeout); current connection cannot be confirmed.'
  if status[1] == 'error':
    reasons = []
    seen = set()
    for source, raw in (('Model', model_error), ('Link', link.get('error')),
                        ('Host', health.get('reason') if health and health.get('fresh') else '')):
      message = _text(raw)
      if message and message not in seen:
        reasons.append(f'{source}: {message}')
        seen.add(message)
    if reasons:
      return ' | '.join(reasons)
    if health and health.get('fresh') and health.get('severity') == 'error':
      return 'Host health error; no details reported.'
    return f"Link state: {_text(link.get('state')) or 'unknown'}; no detailed error was reported."
  if status[0].endswith(' DATA?'):
    return 'Host health data missing, stale or unknown; current health cannot be confirmed.'
  if status[0].endswith(' HOT'):
    reason = _text(health.get('reason')) if health else ''
    return f'Host: {reason}' if reason else 'Host health warning; no details reported.'
  preparation = link.get('preparation')
  if isinstance(preparation, dict):
    stage = _text(preparation.get('stage'), 60)
    message = _text(preparation.get('message'))
    fraction = preparation.get('fraction')
    progress = f' {int(fraction * 100)}%' if isinstance(fraction, (int, float)) and 0 <= fraction <= 1 else ''
    if stage or message or progress:
      return f'{stage or "Preparation"}{progress}' + (f': {message}' if message else '')
  if link.get('state') == 'connecting':
    return 'Connecting to host; handshake not complete.'
  if link.get('state') == 'loading':
    return 'Model preparation in progress (load / warm-up).'
  return _text(health.get('reason')) if health else 'Waiting; no further details reported.'
