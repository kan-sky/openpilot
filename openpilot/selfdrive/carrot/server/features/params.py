import asyncio
import json
import os

from aiohttp import web
from openpilot.common.vehicle_dynamics import validate_vehicle_spec
from openpilot.common import fcw_warning_test

from ..config import PARAMS_BACKUP_PATH
from ..services.param_changes import (
  append_param_change,
  count_changes_since,
  observe_param_values,
  param_fingerprint,
  read_fingerprint_baseline,
  read_param_changes,
  verify_param_changes,
  write_fingerprint_baseline,
)
from ..services.params import (
  HAS_PARAMS,
  INTERNAL_SESSION_PARAMS,
  READ_ONLY_PARAMS,
  ParamKeyType,
  build_params_qr_payload,
  clamp_numeric,
  ensure_qr_dependency,
  get_param_values,
  get_qr_dependency_status,
  parse_params_qr_payload,
  preview_param_restore_values,
  read_param_backup_values,
  restore_param_values_validated,
  restore_param_values_from_backup,
  set_param_value,
)
from ..services.settings import get_settings_cached
from .system import is_drive_engaged
from .remote_support import validate_request

FCW_TEST_LOCK = web.AppKey("fcw_test_lock", asyncio.Lock)


async def set_fcw_warning_test(request, value, source):
  validate_request(request, mutate=True)
  if type(value) not in (int, str) or str(value) not in ("0", "1"):
    return web.json_response({"ok": False, "error": "FCW test requires 0 or 1."}, status=400)
  value = int(value)
  if value and source != "web_ui":
    return web.json_response({"ok": False, "error": "FCW test cannot run from a profile, restore, or undo."}, status=403)
  async with request.app[FCW_TEST_LOCK]:
    try:
      fcw_warning_test.request_test(value)
      if value:
        # Only report ON once the running controller acknowledges the request.
        for _ in range(15):
          await asyncio.sleep(0.02)
          if fcw_warning_test.current_status()["active"]:
            break
        else:
          fcw_warning_test.request_test(0)
          raise ValueError("FCW test was not accepted by the controller. No queued retry.")
      return web.json_response({"ok": True, "name": fcw_warning_test.KEY, "value": value},
                               headers={"Cache-Control": "no-store"})
    except (OSError, ValueError) as error:
      return web.json_response({"ok": False, "error": str(error)}, status=409)


async def api_params_bulk(request: web.Request) -> web.Response:
  names = request.query.get("names", "")
  if not names:
    return web.json_response({"ok": False, "error": "missing names"}, status=400)

  req_names = [n for n in names.split(",") if n]
  try:
    _, _, by_name, _ = get_settings_cached()
  except Exception:
    by_name = {}

  values = get_param_values(
    [n for n in req_names if n != "DeviceType"],
    {n: by_name.get(n, {}).get("default", 0) for n in req_names},
  )
  if "DeviceType" in req_names:
    try:
      from openpilot.system.hardware import HARDWARE
      values["DeviceType"] = HARDWARE.get_device_type()
    except Exception:
      values["DeviceType"] = "unknown"
  for n in req_names:
    if n not in values:
      try:
        values[n] = by_name.get(n, {}).get("default", 0)
      except Exception:
        values[n] = 0

  # Picks up values the driving code changed behind our back, so the history
  # can explain them. Restricted to catalog settings, so synthetic keys like
  # GitPullTime and hardware values like DeviceType -- which change on every
  # read -- never appear in the change history.
  observe_param_values(values, allowed=set(by_name) - {fcw_warning_test.KEY})
  return web.json_response({"ok": True, "values": values})


async def api_param_set(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  name = body.get("name")
  value = body.get("value")
  source = body.get("source")

  if not name:
    return web.json_response({"ok": False, "error": "missing name"}, status=400)
  if name == fcw_warning_test.KEY:
    return await set_fcw_warning_test(request, value, source)
  if name in INTERNAL_SESSION_PARAMS:
    return web.json_response({"ok": False, "error": "driver monitoring is controlled by the vehicle CANCEL gesture"}, status=403)
  if name in READ_ONLY_PARAMS:
    return web.json_response({"ok": False, "error": "cluster brightness is controlled by the selected theme"}, status=403)

  if name == "CarrotNaviSource" and (type(value) not in (int, str) or str(value) not in ("0", "1")):
    return web.json_response({"ok": False, "error": "CarrotNaviSource must be 0 (network) or 1 (Navdy)"}, status=400)

  if name == "NavdyConnectionMode" and (type(value) not in (int, str) or str(value) not in ("0", "1", "2")):
    return web.json_response({"ok": False, "error": "NavdyConnectionMode must be 0 (existing configuration), 1 (USB), or 2 (Bluetooth)"}, status=400)

  try:
    value = validate_vehicle_spec(name, value)
  except ValueError as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=400)

  # clamp using settings if numeric
  p = None
  try:
    _, _, by_name, _ = get_settings_cached()
    p = by_name.get(name)
  except Exception:
    pass

  # Read the old value before writing so the history can show what it replaced.
  previous = None
  try:
    previous = get_param_values([name], {name: (p or {}).get("default")}).get(name)
  except Exception:
    pass

  # If value numeric -> clamp
  try:
    if p is not None and isinstance(p.get("min"), (int, float)) and isinstance(p.get("max"), (int, float)):
      fv = float(value)
      fv = clamp_numeric(fv, p)
      # keep int if setting looks int-ish
      if isinstance(p.get("min"), int) and isinstance(p.get("max"), int) and isinstance(p.get("default"), int):
        value = int(round(fv))
      else:
        value = fv
  except Exception:
    pass

  try:
    set_param_value(name, value, p)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)

  # Changing settings while driving stays allowed on purpose; the history just
  # records that it happened. append_param_change never raises, so a log
  # problem cannot turn a successful write into a reported failure.
  if previous != value:
    append_param_change(
      name,
      previous,
      value,
      source=source,
      engaged=is_drive_engaged(request),
    )

  return web.json_response({"ok": True, "name": name, "value": value, "has_params": HAS_PARAMS})


async def api_param_changes(request: web.Request) -> web.Response:
  try:
    limit = int(request.query.get("limit", "50"))
  except Exception:
    limit = 50
  name = str(request.query.get("name", "")).strip()
  source = str(request.query.get("source", "")).strip()
  changes = await asyncio.to_thread(read_param_changes, max(0, min(limit, 500)), name, source)
  return web.json_response({"ok": True, "changes": changes})


async def api_param_changes_verify(request: web.Request) -> web.Response:
  """Re-walk the hash chain. Hashing is pointless if nothing ever checks it."""
  return web.json_response(await asyncio.to_thread(verify_param_changes))


async def api_param_fingerprint(request: web.Request) -> web.Response:
  """One short digest of every setting, plus how it compares to the saved
  reference, so the user does not have to remember or write the code down."""
  def build() -> dict:
    _data, _groups, by_name, _groups_list = get_settings_cached()
    values = get_param_values(
      [name for name in by_name if name != fcw_warning_test.KEY],
      {name: meta.get("default", 0) for name, meta in by_name.items()},
    )
    result = param_fingerprint(values)

    baseline = read_fingerprint_baseline()
    if baseline is None:
      # First look: adopt the current state as the reference so future visits
      # can say "changed since / unchanged" without the user setting it up.
      baseline = write_fingerprint_baseline(result["fingerprint"])
    result["baseline"] = baseline
    result["changed"] = result["fingerprint"] != baseline.get("fingerprint")
    result["changed_count"] = (
      count_changes_since(int(baseline.get("ts") or 0), allowed=set(by_name))
      if result["changed"] else 0
    )
    return {"ok": True, **result}

  try:
    return web.json_response(await asyncio.to_thread(build))
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_param_fingerprint_baseline(request: web.Request) -> web.Response:
  """Set the current settings as the reference to compare against from now on."""
  def build() -> dict:
    _data, _groups, by_name, _groups_list = get_settings_cached()
    values = get_param_values(
      [name for name in by_name if name != fcw_warning_test.KEY],
      {name: meta.get("default", 0) for name, meta in by_name.items()},
    )
    fingerprint = param_fingerprint(values)["fingerprint"]
    return {"ok": True, "baseline": write_fingerprint_baseline(fingerprint)}

  try:
    return web.json_response(await asyncio.to_thread(build))
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def handle_download_params_backup(request: web.Request) -> web.Response:
  path = PARAMS_BACKUP_PATH
  if not os.path.exists(path):
    return web.json_response({"ok": False, "error": "file not found"}, status=404)

  try:
    values = await asyncio.to_thread(read_param_backup_values, path)
    return web.json_response(
      values,
      headers={
        "Content-Disposition": "attachment; filename=params_backup.json",
        "Cache-Control": "no-store",
      },
    )
  except FileNotFoundError:
    return web.json_response({"ok": False, "error": "file not found"}, status=404)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_params_restore(request: web.Request) -> web.Response:
  if not HAS_PARAMS or ParamKeyType is None:
    return web.json_response({"ok": False, "error": "Params/ParamKeyType not available"}, status=500)

  try:
    reader = await request.multipart()
    part = await reader.next()
    if part is None or part.name != "file":
      return web.json_response({"ok": False, "error": "missing file field"}, status=400)

    data = await part.read(decode=False)
    text = data.decode("utf-8", errors="replace")
    j = json.loads(text)

    if not isinstance(j, dict):
      return web.json_response({"ok": False, "error": "bad json format (must be object)"}, status=400)

    values = j
    res = restore_param_values_from_backup(values)
    return web.json_response({"ok": True, "result": res})

  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_params_qr_backup(request: web.Request) -> web.Response:
  if not HAS_PARAMS or ParamKeyType is None:
    return web.json_response({"ok": False, "error": "Params/ParamKeyType not available"}, status=500)

  try:
    payload = build_params_qr_payload()
    return web.json_response({"ok": True, **payload}, headers={"Cache-Control": "no-store"})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_params_qr_dependency(request: web.Request) -> web.Response:
  try:
    return web.json_response(get_qr_dependency_status())
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_params_qr_dependency_ensure(request: web.Request) -> web.Response:
  try:
    result = await asyncio.to_thread(ensure_qr_dependency)
    status = 200 if result.get("ok") else 500
    return web.json_response(result, status=status)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_params_restore_preview(request: web.Request) -> web.Response:
  if not HAS_PARAMS or ParamKeyType is None:
    return web.json_response({"ok": False, "error": "Params/ParamKeyType not available"}, status=500)

  try:
    body = await request.json()
    payload = body.get("payload")
    values = body.get("values")
    selected_keys = body.get("keys")
    restore_values = parse_params_qr_payload(values if isinstance(values, dict) else payload)
    preview = preview_param_restore_values(
      restore_values,
      selected_keys if isinstance(selected_keys, list) else None,
    )
    return web.json_response({"ok": True, "preview": preview})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=400)


async def api_params_restore_json(request: web.Request) -> web.Response:
  if not HAS_PARAMS or ParamKeyType is None:
    return web.json_response({"ok": False, "error": "Params/ParamKeyType not available"}, status=500)

  try:
    body = await request.json()
    payload = body.get("payload")
    values = body.get("values")
    selected_keys = body.get("keys")
    restore_values = parse_params_qr_payload(values if isinstance(values, dict) else payload)
    restored = restore_param_values_validated(
      restore_values,
      selected_keys if isinstance(selected_keys, list) else None,
    )
    return web.json_response({"ok": True, **restored})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=400)


def register(app: web.Application) -> None:
  app[FCW_TEST_LOCK] = asyncio.Lock()
  app.router.add_get("/api/params_bulk", api_params_bulk)
  app.router.add_post("/api/param_set", api_param_set)
  app.router.add_get("/api/param_changes", api_param_changes)
  app.router.add_get("/api/param_changes/verify", api_param_changes_verify)
  app.router.add_get("/api/param_fingerprint", api_param_fingerprint)
  app.router.add_post("/api/param_fingerprint/baseline", api_param_fingerprint_baseline)
  app.router.add_post("/api/params_restore", api_params_restore)
  app.router.add_get("/api/params_qr_dependency", api_params_qr_dependency)
  app.router.add_post("/api/params_qr_dependency/ensure", api_params_qr_dependency_ensure)
  app.router.add_get("/api/params_qr_backup", api_params_qr_backup)
  app.router.add_post("/api/params_restore_preview", api_params_restore_preview)
  app.router.add_post("/api/params_restore_json", api_params_restore_json)
  app.router.add_get("/download/params_backup.json", handle_download_params_backup)
