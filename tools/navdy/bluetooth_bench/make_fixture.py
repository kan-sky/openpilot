#!/usr/bin/env python3
"""Print synthetic, production-serializer-shaped JSON. Does not read a car."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from openpilot.selfdrive.navdy.display import payload_from_messages


def fixture():
  xs = [80 * i / 32 for i in range(33)]
  def line(offset):
    return NS(x=xs, y=[offset + 0.0002 * x * x for x in xs])
  cs = NS(canValid=True, canTimeout=False, vEgo=20.0, vEgoCluster=21.0, vCruise=90.0,
          vCruiseCluster=91.0, gearShifter='drive', doorOpen=False, standstill=False,
          leftBlinker=False, rightBlinker=False, leftBlindspot=False, rightBlindspot=False,
          softHoldActive=0, brakeHoldActive=False, cruiseState=NS(speed=25, speedCluster=26))
  sd = NS(state='enabled', enabled=True, active=True, engageable=True,
          alertText1='\uc2dc\ud5d8\uc6a9 \ud654\uba74 \uc0c1\ud0dc', alertText2='',
          alertType='', alertStatus='normal', alertSize='none')
  model = NS(position=line(0), laneLines=[line(-5.4), line(-1.8), line(1.8), line(5.4)],
             laneLineProbs=[.8, .9, .95, .7], roadEdges=[line(-7), line(7)], roadEdgeStds=[.1, .1])
  def lead(track, y):
    return NS(status=True, dRel=40, yRel=y, vRel=-2, radar=True, radarTrackId=track, modelProb=.9)
  return payload_from_messages(seq=0, onroad=True, car_state=cs, selfdrive_state=sd,
                               model=model, radar_state=NS(leadOne=lead(42, 0), leadTwo=lead(43, 3.6)))


if __name__ == '__main__':
  print(json.dumps(fixture(), ensure_ascii=False, allow_nan=False, separators=(',', ':')))
