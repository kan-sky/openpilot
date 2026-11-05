import math


EXTERNAL_NAVI_SOURCES = frozenset((
  "cam", "section", "bump", "police", "waze", "road", "atc", "atc2", "route",
))
VEHICLE_NAVI_SOURCES = frozenset(("hda", "hda_section", "hda_bump", "school"))

DECELERATION_SOURCE_LABELS = {
  "cam": "cam",
  "section": "section",
  "bump": "bump",
  "police": "police",
  "waze": "waze",
  "road": "road",
  "atc": "turn",
  "atc2": "turn",
  "route": "route",
  "hda": "cam",
  "hda_section": "section",
  "hda_bump": "bump",
  "school": "school",
  "gas": "gas",
  "vturn": "vturn",
  "model": "model",
  "turn": "turn",
}


def deceleration_candidates(speed_n_sources, targets, distances=None):
  """Expose only enabled, valid candidates without changing their control caps.

  A distant event may have a 250 kph approach envelope and a real endpoint.
  Conversely, sentinel road/straight-curve caps are not detected events.
  """
  candidates = []
  for speed, source in speed_n_sources:
    if source not in targets or source not in DECELERATION_SOURCE_LABELS or source == 'gas':
      continue
    target = targets[source]
    if not math.isfinite(speed) or not 0 < speed <= 250 or not math.isfinite(target):
      continue
    known_target = 0 < target < 200
    if not known_target and not 0 < speed < 200:
      continue
    distance = (distances or {}).get(source, -1.0)
    candidates.append({'source': source, 'speedKph': float(speed), 'targetKph': float(target) if known_target else 0.0,
                       'distanceM': float(distance) if math.isfinite(distance) and distance >= 0 else -1.0})
  return candidates


def is_vehicle_navigation_source(source: str | None) -> bool:
  normalized = str(source or "").strip().lower()
  return normalized in VEHICLE_NAVI_SOURCES or normalized in ("cam:v", "bump:v", "school:v")


def deceleration_source_presentation(source: str | None) -> tuple[str, int]:
  """Return the actual deceleration reason and its source color mode.

  Color mode 2 is the normal deceleration orange, mode 3 is vehicle CAN
  navigation lavender, and mode 4 is external-navigation orange.
  """
  normalized = str(source or "").strip().lower()
  if not normalized:
    return "apply", 2
  if is_vehicle_navigation_source(normalized):
    return DECELERATION_SOURCE_LABELS.get(normalized, normalized[:8]), 3
  if normalized in EXTERNAL_NAVI_SOURCES or normalized.endswith(":n"):
    base = normalized.removesuffix(":n")
    return DECELERATION_SOURCE_LABELS.get(base, base[:8]), 4
  if normalized.endswith(":v"):
    base = normalized.removesuffix(":v")
    mode = 3 if base in ("cam", "section", "bump", "school") else 2
    return DECELERATION_SOURCE_LABELS.get(base, base[:8]), mode
  if normalized.endswith(":c"):
    base = normalized.removesuffix(":c")
    return DECELERATION_SOURCE_LABELS.get(base, base[:8]), 2
  return DECELERATION_SOURCE_LABELS.get(normalized, normalized[:8]), 2


def navigation_status_presentation(vehicle_available: bool, external_active: bool) -> tuple[str, int] | None:
  """Return the navigation availability badge, independent of speed control."""
  if external_active:
    return "NAVI", 4
  if vehicle_available:
    return "vNAVI", 3
  return None


def external_navigation_connected(
  legacy_remote: str | None,
  carrot_navi_connected: bool,
  *,
  external_navigation_source: str = "",
  external_navigation_active: bool = False,
) -> bool:
  """Return navigation availability from fresh inputs, not speed-control state.

  Callers must gate remote/active by the publishing service's alive/valid state.
  Keep the selected source even when stale so Navdy cannot fall back to network
  inputs. Its active flag represents verified navigation, not a Bluetooth link.
  """
  if external_navigation_source == "navdy":
    return bool(external_navigation_active)
  return bool(external_navigation_active) or bool(str(legacy_remote or "").strip()) or bool(carrot_navi_connected)
