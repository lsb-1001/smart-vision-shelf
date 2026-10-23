# Gazebo 선반 시뮬레이션

Ubuntu에서 Gazebo 카메라 검출과 Nav2 이동 후 마커 검증을 시험했다. 확인 범위와 남은 문제는 [검증 기록](docs/VALIDATION.md)을 참고한다.

## 범위

1. 고정 월드에 ID 1~4 표지판과 선반/배송 지점을 배치
2. TurtleBot3 Waffle Pi 실제 시뮬레이션 카메라에서 마커 검출
3. 원하면 Nav2 이동 성공 이후 같은 ROS 선반 모듈에 도착 검증 요청

색 적재 인식이나 자동 적재/하역은 포함하지 않는다. 마커 검증 결과가 verified여도
작업자 적재/하역 완료와 임무 완료는 별도다.

## 배치

| 지점 | 마커 (world x,y) | 로봇 접근 (map x,y,yaw rad) |
|---|---|---|
| A / ID 1 | (3,2) | (2.2,2,0) |
| B / ID 2 | (3,0) | (2.2,0,0) |
| C / ID 3 | (3,-2) | (2.2,-2,0) |
| DELIVERY / ID 4 | (-3,0) | (-2.2,0,pi) |

로봇은 B 접근 위치에서 시작한다. 마커 검은 부분 한 변 0.24m, 흰 판 0.30m,
중심 높이 0.15m이다. 이전 Gazebo 카메라 실습에서 보인 크기·높이를 활용했다.
마커는 텍스처 로딩 없이 흰 판과 검은 작은 상자로 만들어 네 변이 보이도록 한다.
표지판과 선반에는 충돌 형상이 있다. 시뮬레이션 좌표를 실물 좌표로 사용하지 않는다.

map은 같은 월드의 정적 기하에서 생성한 시험용 지도다. SLAM으로 측정한 지도나
실물 지도가 아니다. 원본 locations.json은 수정하지 않고 sim/locations.sim.json을 만든다.

## 준비

이전에 실행한 Gazebo/ROS 실습은 해당 실행 터미널에서 Ctrl+C로 종료한다.
다른 팀원의 노드를 임의로 종료하거나 시스템 전체를 종료하지 않는다.
로컬 도메인 87과 ROS_LOCALHOST_ONLY=1을 사용해 실물(44)·다른 팀 시뮬레이션과 분리한다.

저장소를 내려받은 뒤 Ubuntu에서 프로젝트 폴더로 이동한다.

```bash
cd ~/smart-vision-shelf
source sim_env.sh
python3 build_sim.py
```
필요 패키지(이전 실습에서 설치한 항목): gazebo_ros, turtlebot3_gazebo,
turtlebot3_navigation2, nav2_bringup, OpenCV ArUco, cv_bridge.

## 단계 1 — 정지한 로봇 카메라로 ID 2 확인

칸 1:

```bash
cd ~/smart-vision-shelf
source sim_env.sh
ros2 launch ./shelf_sim.launch.py
```

빈 방, 선반 4개, 로봇이 나타나고 spawn 성공을 확인한 뒤 칸 2:

```bash
cd ~/smart-vision-shelf
source sim_env.sh
python3 shelf_ros.py --registry sim/locations.sim.json --show --ros-args -p use_sim_time:=true
```

카메라 창에서 ID 2 검출을 확인한다. 아직 주행하지 않았으므로 자동 검증은 시작하지 않는다.
칸 3에서 현재 주차 위치를 도착 상태로 가정해 검증 연결만 시험하려면:

```bash
cd ~/smart-vision-shelf
source sim_env.sh
python3 sim_request.py verify B --assume-arrived
```

이 단계는 Gazebo 카메라 검증이지만 Nav2 주행 도착 시험은 아니다.

## 단계 2 — Nav2로 A/C/배송 지점 이동 후 검증

카메라 확인이 된 후에만 진행한다. 칸 4:

```bash
cd ~/smart-vision-shelf
source sim_env.sh
ros2 launch ./shelf_nav.launch.py
```

RViz는 실행하지 않는다. 칸 3에서 **새 월드 시작 후 로봇이 아직 B 출발 위치에 있을 때만**:

```bash
python3 sim_mission.py A --initialize
```

--initialize는 AMCL에 정해진 출발 위치(2.2,0,0)를 넣는 **시뮬레이션 편의 기능**이다.
ArUco 자동 위치 초기화 구현이라고 해석하지 않는다. 실물에 사용하지 않는다.
이미 로봇을 움직였다면 --initialize를 쓰지 않는다.

Nav2가 성공해야 마커 확인을 요청한다. 검증까지 성공한 뒤 다음 지점을 시험:

```bash
python3 sim_mission.py C
python3 sim_mission.py DELIVERY
```

각 명령이 끝난 뒤 다음을 실행한다. 자동 적재·하역이나 전체 물류 FSM은 실행하지 않는다.
Nav2 실패/시간 초과면 마커 검증을 시작하지 않으며, 진행 중 시간 초과/중단 시 취소를 요청한다.
마커가 보이지 않으면 목표 yaw, 카메라 시야, 실측 도착 오차를 확인한다.

## 종료와 한계

각 실행 칸에서 Ctrl+C. 도메인은 그 셸에만 적용되며 실물 연결 시 sim_env.sh를 사용하지 않는다.
소스 영상 시각과 ROS 시각을 맞추기 위해 shelf_ros에는 use_sim_time=true가 필수다.
Gazebo 서비스 준비는 로봇 생성 시 최대 120초 기다린다.
실물 연결 시 지도·접근 좌표·카메라·마커 사전·시간/네트워크 설정을 다시 맞춰야 한다.
