"""Compact optional Navdy badge code, not navigation/control input."""
from openpilot.selfdrive.carrot.deceleration_display import deceleration_state

# Stable wire IDs shared with Navdy v173. Never reorder or recycle them.
SOURCE_IDS = {
  'cam': 1, 'section': 2, 'bump': 3, 'police': 4, 'waze': 5, 'road': 6,
  'atc': 7, 'atc2': 8, 'hda': 9, 'hda_section': 10, 'hda_bump': 11,
  'school': 12, 'vturn': 13, 'route': 14,
}
PHASE_IDS = {'waiting': 1, 'decelerating': 2, 'complete': 3}


def badge_code(car, selfdrive, control, plan, carrot):
  state = deceleration_state(car, selfdrive, control, plan, carrot)
  if state is None or state.source not in SOURCE_IDS:
    return 0
  return 4 * SOURCE_IDS[state.source] + PHASE_IDS[state.phase]
