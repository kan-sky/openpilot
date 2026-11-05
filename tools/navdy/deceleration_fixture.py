"""Export synthetic production-encoder cases for the Navdy v173 host checks."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

from openpilot.selfdrive.carrot.deceleration_source import DECELERATION_SOURCE_LABELS
from openpilot.selfdrive.navdy.deceleration import PHASE_IDS, SOURCE_IDS
from openpilot.selfdrive.navdy.display import payload_from_messages


def cases():
  result = []
  for source, source_id in SOURCE_IDS.items():
    for phase, cap in [('waiting', 250), ('decelerating', 100), ('complete', 80)]:
      payload = payload_from_messages(
        onroad=True, car_state=NS(canValid=True, canTimeout=False, vEgo=20., vCruise=120.,
                                 vCruiseCluster=120., gearShifter='drive', gasPressed=False, brakePressed=False),
        selfdrive_state=NS(enabled=True, active=True), car_control=NS(longActive=True),
        longitudinal_plan=NS(cruiseTarget=cap),
        carrot_man=NS(desiredSource=source, desiredSpeed=cap,
                      decelerationCandidates=[NS(source=source, speedKph=cap, targetKph=80., distanceM=100.)]),
      )
      code = 4 * source_id + PHASE_IDS[phase]
      assert payload['decel'] == code
      result.append({'name': f'{source}-{phase}', 'payload': payload, 'code': code,
                     'source': source_id, 'phase': PHASE_IDS[phase],
                     'label': DECELERATION_SOURCE_LABELS[source].upper()})
  return result


if __name__ == '__main__':
  Path(sys.argv[1]).write_text(json.dumps({'schema': 'navdy.deceleration-tests.v1', 'cases': cases()}, indent=2) + '\n')
