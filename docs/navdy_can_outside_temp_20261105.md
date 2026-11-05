# 나브디 외기온도를 CAN 값으로 (2026-11-05, Claude)

## 무엇을 바꾸나

- 나브디 HUD는 지금 외기온도를 자체 OBD 연결의 PID `0x46`에서 5초마다 읽는다
  (`OpenpilotStateReceiver.updateOutsideTemp`, v18 이후).
- 콤마 쪽: 트래버스에서 ECM `0x3FC`(bus0) byte4를 읽어 `carState.outsideAirTemp`로 내보내고,
  나브디 페이로드에 `outsideTempC`(0.5 °C 단위)를 붙인다. 값이 없거나 오래되면 키를 빼서
  HUD가 기존 OBD 값을 그대로 쓰게 한다.
- HUD 쪽(별도 APK 버전 필요): 신선한 `outsideTempC`가 있으면 그 값을, 없으면 OBD `0x46`을 표시.

## 해석 근거

- `0x3FC` byte4 = 계기판 표시용 필터 외기온, `raw * 0.5 - 40 °C`. 언제든 내려갈 수 있지만 오르는 것은 주행 중뿐.
- byte7은 byte4와 34,381/34,381 프레임에서 같다(무결성 확인에 사용).
- 레이더 버스 `0x303` byte1(외기온, CT6 object DBC)과 8,576/8,596 프레임 일치(나머지는 전환 시점 +1).
- 시동 직후 첫 프레임부터 값이 있다(1d3--0: 15.0 °C, 1dc--0: 14.0 °C).
- byte3은 정차 중 열이 차오르는 원시 센서 값이라 쓰지 않는다.

## 구현

- `openpilot/selfdrive/car/outside_air_temp.py`: 디코더(길이 8, byte4==byte7, -40~85 °C), 2초 신선도.
- `card.py`: 트래버스에서만 `can_list`를 한 번 더 훑어 `CS.outsideAirTemp`(valid, celsius) 기록. CAN 무효·타임아웃이면 valid=false.
- `opendbc_repo/opendbc/car/car.capnp`: `CarState.outsideAirTemp @106`.
- `navdy/display.py`: valid일 때만 `outsideTempC` 추가. 다른 페이로드 필드는 그대로.

## 검증

- 새 테스트 26개 통과(디코더·신선도·순서·capnp 왕복·페이로드). 차선 표시 테스트 11개 통과.
- 나브디 전체 테스트: 59 failed / 1424 passed. HEAD의 display.py로 돌려도 같은 59개가 실패(기존 Mac 환경·순서 문제).
- 실제 rlog 2구간(각 60초, 초당 약 5,100프레임): 디코더 총 9 ms/분(Mac 코어 0.015%), 값 14.0 °C·18.0 °C로 일정.
- 실차·HUD 화면은 아직 확인하지 않음. HUD APK 수정 전에는 화면 표시가 바뀌지 않음.
