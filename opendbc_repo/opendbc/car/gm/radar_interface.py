#!/usr/bin/env python3
import math
from openpilot.cereal import car
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm.values import DBC, CanBus
from opendbc.car.interfaces import RadarInterfaceBase

RADAR_HEADER_MSG = 1120  # F_LRR_Obj_Header
CAMERA_DATA_HEADER_MSG = 1056  # F_Vision_Obj_Header
SLOT_1_MSG = RADAR_HEADER_MSG + 1
NUM_SLOTS = 20

# Actually it's 0x47f, but can parser only reports
# messages that are present in DBC
LAST_RADAR_MSG = RADAR_HEADER_MSG + NUM_SLOTS


def create_radar_can_parser(car_fingerprint):
  # C1A-ARS3-A by Continental
  radar_targets = list(range(SLOT_1_MSG, SLOT_1_MSG + NUM_SLOTS))
  signals = list(zip(['FLRRNumValidTargets',
                      'FLRRSnsrBlckd', 'FLRRYawRtPlsblityFlt',
                      'FLRRHWFltPrsntInt', 'FLRRAntTngFltPrsnt',
                      'FLRRAlgnFltPrsnt', 'FLRRSnstvFltPrsntInt'] +
                     ['TrkRange'] * NUM_SLOTS + ['TrkRangeRate'] * NUM_SLOTS +
                     ['TrkRangeAccel'] * NUM_SLOTS + ['TrkAzimuth'] * NUM_SLOTS +
                     ['TrkWidth'] * NUM_SLOTS + ['TrkObjectID'] * NUM_SLOTS,
                     [RADAR_HEADER_MSG] * 7 + radar_targets * 6, strict=True))

  messages = list({(s[1], 14) for s in signals})

  return CANParser(DBC[car_fingerprint][Bus.radar], messages, CanBus.OBSTACLE)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    # CP.radarUnavailable == True 인 차량은 레이더 완전 비사용 (비전-only 모드)
    self.rcp = None if CP.radarUnavailable else create_radar_can_parser(CP.carFingerprint)

    # 한 프레임이 완성되었다고 보는 트리거 메시지
    self.trigger_msg = RADAR_HEADER_MSG
    self.updated_messages = set()

    # Kans
    self.track_id = 0  # RadarPoint.trackId 생성용 내부 카운터

    # 부팅초기 레이더안정화 구간설정
    self.radar_warmup_done = False
    self.radar_valid_cnt = 0
    self.radar_valid_threshold = 10  # 연속 10회 정상수신시 warmup 종료

  def update(self, can_strings):
    # 레이더가 완전히 비활성(CP.radarUnavailable=True)인 경우
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    # 아직 헤더를 못 받았으면 프레임 미완성
    if self.trigger_msg not in self.updated_messages:
      return None

    ret = structs.RadarData()
    header = self.rcp.vl[RADAR_HEADER_MSG]

    fault = header['FLRRSnsrBlckd'] or header['FLRRSnstvFltPrsntInt'] or \
      header['FLRRYawRtPlsblityFlt'] or header['FLRRHWFltPrsntInt'] or \
      header['FLRRAntTngFltPrsnt'] or header['FLRRAlgnFltPrsnt']

    # 레이더 warmup 완료 판정
    if not self.radar_warmup_done:
      if self.rcp.can_valid:
        self.radar_valid_cnt += 1
      else:
        self.radar_valid_cnt = 0

      if self.radar_valid_cnt >= self.radar_valid_threshold:
        self.radar_warmup_done = True
        print("GM_RADAR warmup done")

    # debug
    if not self.rcp.can_valid or fault:
      print(
        f"GM_RADAR v={int(self.rcp.can_valid)} warm={int(self.radar_warmup_done)} "
        f"cnt={self.radar_valid_cnt} f={int(fault)} "
        f"blk={header['FLRRSnsrBlckd']} sns={header['FLRRSnstvFltPrsntInt']} "
        f"yaw={header['FLRRYawRtPlsblityFlt']} hw={header['FLRRHWFltPrsntInt']} "
        f"ant={header['FLRRAntTngFltPrsnt']} algn={header['FLRRAlgnFltPrsnt']} "
        f"nTgts={header['FLRRNumValidTargets']}"
      )

    # warmup 전에는 초기 false fault 무시
    if self.radar_warmup_done:
      if not self.rcp.can_valid:
        ret.errors.canError = True
      if fault:
        ret.errors.radarFault = True

    currentTargets = set()
    num_targets = header['FLRRNumValidTargets']

    # Not all radar messages describe targets,
    # no need to monitor all of the self.rcp.msgs_upd
    for ii in self.updated_messages:
      if ii == RADAR_HEADER_MSG:
        continue

      if num_targets == 0:
        break

      cpt = self.rcp.vl[ii]
      # Zero distance means it's an empty target slot
      if cpt['TrkRange'] > 0.0:
        targetId = cpt['TrkObjectID']
        currentTargets.add(targetId)

        # 새로운 타겟이면 RadarPoint 생성 + 내부 track_id 부여
        if targetId not in self.pts:
          self.pts[targetId] = structs.RadarData.RadarPoint()
          self.pts[targetId].trackId = self.track_id
          self.track_id += 1

        distance = cpt['TrkRange']

        # 값 업데이트
        self.pts[targetId].dRel = distance  # from front of car
        # From driver's pov, left is positive
        self.pts[targetId].yRel = math.sin(cpt['TrkAzimuth'] * CV.DEG_TO_RAD) * distance
        self.pts[targetId].vRel = cpt['TrkRangeRate']
        self.pts[targetId].vLead = self.pts[targetId].vRel + self.v_ego
        self.pts[targetId].aRel = float('nan')
        self.pts[targetId].yvRel = 0.0 # float('nan')
        self.pts[targetId].measured = True

    # 이전 프레임에서 사라진 타겟 제거
    for oldTarget in list(self.pts.keys()):
      if oldTarget not in currentTargets:
        del self.pts[oldTarget]

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret
