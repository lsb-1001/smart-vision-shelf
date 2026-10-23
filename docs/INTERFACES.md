# 선반 인식 모듈 인터페이스 v0.1

아래는 독립 모듈용 제안이며 팀 통합 시 조정합니다.

## 역할 경계

- 이 모듈: 마커 ID 식별, 접근 좌표 조회, 주행 도착 후 ID 검증
- 위치 초기화 담당: 기준 마커 지도 자세 + 카메라/로봇 TF → AMCL 초기화
- 주행 담당: A*/Nav2, 경로 추종, 장애물 회피, 주행 성공/실패
- 통합 담당: 임무 상태 전환, 작업자 적재·하역 완료 입력

## 등록 파일

`locations.json`: schema_version, map_id, dictionary, verification, locations.
각 지점은 key/name/marker_id/item/approach_pose를 갖습니다.
실물 접근 좌표는 기본 null이며 좌표를 등록하려면 map_id와 frame_id=map이 필요합니다.
x/y는 m, yaw는 rad. 좌표는 마커 위치가 아닌 로봇 정차 접근 위치입니다.
지도 식별자가 다르면 조회 결과는 map_mismatch이며 사용할 좌표를 반환하지 않습니다.

## 요청과 응답

입력: `/shelf_vision/request`, `std_msgs/msg/String` 안 JSON.

```json
{"op":"lookup","location":"B","map_id":"warehouse-map-v1","request_id":"lookup-001"}
```

조회는 ready / coordinates_unregistered / map_mismatch로 응답합니다.
등록 지점이 없거나 JSON이 잘못되면 reply의 ok=false와 error를 확인합니다.

주행 담당이 도착 성공을 알린 뒤:

```json
{"op":"verify","location":"B","request_id":"verify-001","navigation_arrived":true}
```

navigation_arrived는 호출자가 제공하는 사건입니다. 선반 모듈이 실제 주행 위치를
독립적으로 검증했다는 의미가 아닙니다. 주행 허용 오차는 주행 모듈이 담당합니다.

취소:

```json
{"op":"cancel","request_id":"cancel-001"}
```

요청 ID는 매번 새로 발급합니다. 최근 100개 중복 ID는 거절합니다.
진행 중 목표를 교체하려면 취소한 뒤 새 verify 요청을 보냅니다.
reply는 명령 수락/거절, result는 검증 진행/종료를 전달합니다.

## 결과 상태

| state | 의미 |
|---|---|
| idle | 검증 요청 없음 |
| waiting_fresh_frame | 요청 이후의 최신 프레임 대기 |
| confirming_target | 목표 ID 연속 확인 중 |
| confirming_wrong_location | 다른 등록 지점 연속 확인 중 |
| verified | 목표 ID 일치 확인 완료 |
| wrong_location | 다른 등록 지점 확인으로 종료 |
| marker_not_visible | 프레임은 있으나 마커 없음 |
| unknown_marker | 미등록 ID 관측 |
| ambiguous_markers | 여러 ID 관측, 확정하지 않음 |
| camera_timeout | 최신 영상 수신 조건 충족 못 함, 종료 |
| camera_error | 영상 처리 오류, 종료 |
| verification_timeout | 제한 시간 내 확정 못 함, 종료 |
| cancelled | 취소 |

검증 결과는 요청별로 유지됩니다. 최신 observations와 종료된 요청의 result를
혼동하지 말고 request_id로 매칭합니다. 종료 뒤 재검증하려면 새 요청이 필요합니다.
unknown/multiple/not-visible은 기본 제한 시간 동안 재관측하며 자동 성공하지 않습니다.

기본 확인 조건: 0.6초, 최소 3프레임, 프레임 간격 최대 0.4초.
카메라 타임아웃 1.5초, 요청 제한 시간 10초. 설정은 registry에서 관리합니다.

## 영상 시각

실물: 노드와 카메라의 시계를 동기화합니다.
Gazebo: `use_sim_time=true`, /clock 필요.
영상 timestamp 0, 요청 이전 프레임, 중복/역행 프레임, 오래된 프레임은 거부합니다.
영상과 /clock 수신 순서 차이 때문에 0.1초 이내의 앞선 영상 시각을 허용합니다.
observations에 source_age_seconds/source_freshness_reason을 출력합니다.
서로 다른 시간 기준을 허용하는 기능이 아니며 큰 시각 차이는 여전히 거부됩니다.

## 작업자 확인

verified → Task Manager가 작업자 적재/하역 입력을 기다립니다.
이 모듈은 cargo/detected에 쓰지 않으며 물품 종류를 카메라로 분류하지 않습니다.
