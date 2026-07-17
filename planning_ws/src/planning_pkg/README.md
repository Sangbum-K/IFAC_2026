# planning_pkg

F1TENTH 플래닝 패키지. 팀 인터페이스 규약:

```
/perception/objects  ──▶  planning_node  ──▶  /planning/plan
   (perception_pkg)          (여기)            (control_pkg)
```

ROS 2 Humble / ament_cmake. `launch/planning.launch.py`(리포 최상위)로 실행.

## 구성 개요

| 위치 | 내용 | 상태 |
|---|---|---|
| `src/planning_node.cpp` | 팀 인터페이스 골격 노드 | 골격 (아래 "포팅 TODO") |
| `src/global_trajectory_node.cpp` | raceline CSV → `/global_waypoints` (2 Hz) | **동작 코드** (planner_pkg 이식) |
| `src/frenet_frame_node.cpp` | odom → Frenet (s,d) → `/car_state/frenet/odom` | **동작 코드** (〃) |
| `src/local_trajectory_node.cpp` | 80 wpt 로컬 윈도우 → `/local_waypoints` (회피 궤적 splice) | **동작 코드** (〃 + 회피 병합) |
| `src/spliner_node.cpp` | 장애물 회피 스플라인 플래너 → `/planner/avoidance/otwpnts` | **동작 코드** (ForzaETH spliner C++ 포팅, 시나리오 검증됨) |
| `src/planner_visualizer_node.cpp` | waypoints/centerline RViz 마커 | **동작 코드** (〃) |
| `include/planning_pkg/*.hpp` | 위 4개 노드 헤더 | 이식 시 신규 작성 (원본에 누락) |
| `launch/planner_stack.launch.py` | 4개 노드 일괄 실행 | 신규 (구 launch 버그 제거) |
| `config/params.yaml`, `config/environments/*` | ROS 파라미터 (env 스위치) | 4개 노드 섹션 추가됨 |
| `config/global_trajectory/*.csv` | IQP raceline (런타임 입력) | 생성됨 |
| `config/centerline/*.csv` | centerline + wall boundary | 생성됨 |
| `maps/` | 개발용 트랙 맵 (대회 맵은 교체) | Spielberg |
| `scripts/global_traj_sim/` | **오프라인 개발 도구** (.venv Python) | 완성·검증됨 |
| `scripts/sim/` | 시뮬 전용 노드 3개 (tracker/obstacle/opponent) | 완성·검증됨 |
| `scripts/tuner/`, `scripts/lib/` | 레이스라인 튜너, pitwall 텔레메트리 | 완성 |
| `../pitwall_panel/` | RViz Pitwall 패널 (Qt 플러그인) | 완성 |
| `../ae_hyu_msgs/` | 공용 메시지 (Wpnt/Obstacle/BehaviorState) | 이식됨 — perception/control도 이 타입 사용 권장 |

> **이식 노트**: 원본 `planning_ws/planner_pkg`(리포 최상위)는 헤더 4개,
> CMakeLists.txt, planner_config.yaml, visualizer 콜백 구현 2개가 누락된
> 미완성 패키지였다. 이식하면서 전부 복원/신규 작성했고, share 디렉토리
> 참조를 `planning_pkg`로, 기본 CSV를 Spielberg IQP raceline으로 맞췄다.
> **첫 colcon build는 ROS2 Humble 머신에서 해봐야 한다** (이 Mac엔 ROS가
> 없어 정적 검토까지만 완료).

### 빌드 & 실행 (ROS2 Humble 머신)

```bash
cd main/planning_ws
colcon build            # ae_hyu_msgs -> planning_pkg 순서로 자동 빌드
source install/setup.bash
ros2 launch planning_pkg planner_stack.launch.py                 # sim (기본)
ros2 launch planning_pkg planner_stack.launch.py odom_mode:=real # 실차
```

## RViz 시뮬레이션 (sim_race.launch.py)

unicorn 튜토리얼(basic-simulator-usage)의 `race.launch sim:=true`에 대응하는
데모. 그 튜토리얼의 스택은 ROS1이라 이 머신(Humble)에서 직접 실행이 안 되므로,
`~/sim_ws`의 f1tenth_gym_ros(ROS2) 시뮬레이터에 이 planning 스택을 연결했다.

```bash
source /opt/ros/humble/setup.bash
source ~/sim_ws/install/setup.bash                  # f1tenth_gym_ros (필수)
source ~/IFAC_ws/planning_ws/install/setup.bash
ros2 launch planning_pkg sim_race.launch.py         # rviz:=false로 headless
```

구성: gym 시뮬(Spielberg 맵, **2-agent** — ego + 물리 상대 차량) + planner
스택 + pure pursuit 트래커 + Pitwall 시뮬 노드들 + RViz. 상대 차량은 시작
시 맵 구석 '주차장'(레이스라인에서 ~49 m, ego LiDAR 사거리 밖)에 있다.

RViz에서 (unicorn pitwall-panel-usage 튜토리얼과 동일):

- **Publish Point**로 트랙 위 클릭 → 정적 장애물 생성 → spliner가 회피
  (빨간 실린더 = 장애물, 보라 실린더 = 회피 스플라인). 근처(0.5 m) 재클릭
  → 제거, 패널의 **Clear static obstacles** → 전체 삭제
- **2D Goal Pose** → 상대 차량을 그 위치로 텔레포트 = **스폰**.
  패널의 **Remove opponent** → 주차장으로 복귀 = 제거
- **2D Pose Estimate** → ego 위치 재배치 (frenet/local 노드에 pose-jump
  전체 재탐색 폴백이 있어 어디로 옮겨도 즉시 재추종한다)
- **PitwallPanel** (Panels 메뉴에서 추가 가능, 기본 표시): 아래 Pitwall 절

시뮬 전용 노드 (`scripts/sim/`, 실차에선 팀 노드로 대체):

| 노드 | 역할 |
|---|---|
| `waypoint_tracker_node.py` | `/local_waypoints` pure pursuit → `/drive`. **speed_scale 0.5 기본** (7 m/s 전속은 스핀아웃, 0.5에서 횡편차 ±6 cm 검증). **벽 충돌 복구 내장** (스턱 감지 → 후진 → 재개). **ego 모드 먹스**: `/sim/ego_mode`("autonomous"/"manual") 또는 `/joy` LB=manual·RB=autonomous. manual이면 정지 1회 발행 후 침묵 → gym kb_teleop(`/cmd_vel`)이 조종. control 팀 컨트롤러로 대체 예정. |
| `virtual_obstacle_node.py` | `/clicked_point` → Frenet 변환 → `/perception/obstacles` 10 Hz 상시 발행. `/sim/clear_obstacles`로 전체 삭제. perception 주입 모드 게이팅(아래). perception 팀 노드로 대체 예정. |
| `opponent_driver_node.py` | gym 2번째 차량 조종. 모드 `/sim/opp_mode`: **path**(레이스라인 pure pursuit, 기본) / **ftg**(`/opp_scan` follow-the-gap) / **manual**(`/opp_cmd_vel` Twist — `teleop_twist_keyboard --ros-args -r cmd_vel:=/opp_cmd_vel`). `/sim/opp_speed_delta`(±0.5 m/s 상한 조절), `/sim/remove_opponent`(주차), `/sim/opp_lidar_enable`(false면 FTG 정지). ground-truth 관측을 `/perception/opponent`로 발행 (tracking_merge 모드일 때). |
Perception Injection (`/sim/perception_mode`, 패널 콤보박스 —
virtual_obstacle_node에 내장):
- **tracking_merge** (기본): ground-truth 장애물/상대를 `/perception/obstacles`로
  직접 주입 — 플래너 단독 검증 (지금까지의 동작).
- **lidar_overlay**: GT 주입 중단, 가상 장애물의 합성 반사점을 `/scan`에 삽입한
  `/scan_overlay`만 발행 — perception 팀 파이프라인이 생기면 탐지→추적→플래너
  전체 체인을 시뮬에서 검증. (상대 차량은 실제 gym 에이전트라 `/scan`에 이미
  물리적으로 찍힌다.)

검증(headless 26/26 PASS): 2-agent에서 ego 3.5 m/s 주행, 장애물 회피,
상대 스폰 → GP 예측(4 s) → 추월(10 s, 최소 이격 0.65 m·접촉 없음), FTG 주행,
LiDAR 토글, Remove/Clear, Manual/Autonomous 전환, 텔레메트리 수신.
좁은 구간(좌우 여유 < 1 m) 장애물은 TRAILING 폴백으로 3 m 앞 정지 확인.

> `spliner/evasion_dist`는 0.65가 이 raceline(safe-dist 0.5)의 실질 최대다.
> 0.7 이상이면 한쪽 벽 여유(~1.05 m)를 넘어 트랙경계 abort로 회피가 무산된다.
> 회피가 불가능한 구간에서는 spliner가 **TRAILING 폴백**을 발행한다: raceline을
> 유지한 채 장애물 뒤 `trailing_gap_m`(1.5 m)에서 장애물 속도(정적이면 0)까지
> `trailing_decel`(3 m/s²)로 감속 — 정면 돌진/관통 방지 (ForzaETH 상태머신의
> TRAILING 대응, 실차에서도 동일하게 동작).

## Pitwall (텔레메트리 + RViz 패널)

unicorn의 pitwall-telemetry / pitwall-panel-usage 대응.

**텔레메트리** — 토픽 정의 없이 내부 변수를 한 줄로 노출:

```python
import pitwall                       # scripts/lib/pitwall.py (노드와 같은 폴더에 설치됨)
pitwall.init(self)                   # __init__ 1회
pitwall.log("cmd_speed", v)          # -> /pitwall/cmd_speed (Float32)
pitwall.event("벽 충돌 감지")          # -> /pitwall/events (String)
```

```cpp
#include "planning_pkg/pitwall.hpp"  // header-only, control_ws에서도 사용 가능
pitwall::Init(this); pitwall::Log("lat_err", d); pitwall::Event("...");
```

init을 안 하면 전부 조용히 no-op — 계측이 주행 코드를 죽이지 않는다.
현재 계측: tracker(cmd_speed/cmd_steer/lookahead, 스턱·복구 이벤트),
frenet(lat_err, pose-jump), local(spliced_wpnts), spliner(evasion_d_apex,
회피 포기·사이드 스위치 거부), predictor(opp_gap/pred_catch_s, GP·예측 이벤트).

```bash
ros2 run rqt_plot rqt_plot /pitwall/lat_err/data /pitwall/cmd_speed/data
ros2 topic echo /pitwall/events
ros2 bag record -s mcap -e '/pitwall/.*' -o pitwall_$(date +%m%d)
```

**패널** (`pitwall_panel` 패키지, rviz_common::Panel 플러그인):
STATE 배너(GB_TRACK 초록 / TRAILING 노랑 / OVERTAKE 보라 / RECOVERY 빨강 /
MANUAL 회색 — H2H 상태머신 전까지 otwpnts·예측·이벤트에서 유도),
이벤트 로그(200줄), ego Autonomous/Manual 버튼, 상대 모드 콤보(path/ftg/manual)·
속도 ±0.5·Remove, Clear static obstacles, ego/상대 LiDAR 체크박스,
Perception 주입 콤보. `sim_race.rviz`에 기본 포함.

패널 ↔ 노드 토픽 계약: `/sim/ego_mode`, `/sim/ego_status`, `/sim/opp_mode`,
`/sim/opp_status`, `/sim/opp_speed_delta`(Float32), `/sim/remove_opponent`(Empty),
`/sim/clear_obstacles`(Empty), `/sim/ego_lidar_enable`·`/sim/opp_lidar_enable`(Bool),
`/sim/perception_mode`(String).

## 두 개의 층으로 나뉜다

### 1. 오프라인 도구 (`scripts/global_traj_sim/`, .venv Python)

colcon 빌드 대상이 아니다. numpy/scipy/matplotlib가 필요하며 `.venv`로
실행한다. 맵에서 raceline을 생성하고(step1~4), 두 로컬 플래너 알고리즘을
시뮬레이션·비교한다(step5~12). 파일명이 `step1`~`step12`로 학습/실행
순서를 나타낸다. 자세한 내용은 `scripts/global_traj_sim/README.md`.

```bash
# raceline 생성. 결과 CSV가 config/ 아래로 저장된다.
#  - 메인 주행 라인(IQP 최소곡률) + 추월 라인(SP 최단거리) 두 CSV가 함께 생성됨
#    (UNICORN이 IQP/SP를 둘 다 저장해 lane-change에 SP를 쓰는 것과 같은 구조)
#  - --safe-dist: 벽과 확보할 거리. 0.5 권장 (0.3은 코너 클리핑 위험)
python scripts/global_traj_sim/step4_main.py --map Spielberg_map --raceline iqp --safe-dist 0.5
# 대회 맵을 쓸 때: --maps-dir <조직위 맵 폴더> --map <맵이름>
# 생성 후 colcon build로 install에 복사해야 런타임 노드가 새 CSV를 읽는다
```

### 2. 런타임 노드 (ROS 2, colcon 빌드)

- `planning_node` — 현재는 std_msgs/String 골격. 실제 로직은 아래 포팅 TODO.
- 나머지 raceline 추종/회피 노드는 위 표 참고. 시뮬 실행은 위
  "RViz 시뮬레이션" 절, raceline 시각화·편집은 아래 "레이스라인 튜너" 절.

## 장애물 회피 (spliner_node)

ForzaETH `unicorn-racing-stack-main/planner/spliner`(ROS1 Python)의 C++ ROS2
포팅. 아키텍처는 원본과 동일하되, 원본에서 state_machine이 하던 splice
(회피 궤적을 로컬 경로에 병합)는 `local_trajectory_node`가 담당한다:

```
/perception/obstacles ──▶ spliner_node ──▶ /planner/avoidance/otwpnts
/car_state/frenet/odom ──▶     │                       │
/global_waypoints      ──▶     └─▶ /planner/avoidance/markers (RViz)
                                                       ▼
/global_waypoints ────────────────────▶ local_trajectory_node ──▶ /local_waypoints
                                        (겹치는 s 구간을 회피 궤적으로 치환)
```

동작 방식 (원본 알고리즘 그대로):
1. raceline 근처(`obs_traj_tresh`) + 전방 `lookahead_m` 이내 장애물만 필터,
   동적 장애물은 `fixed_pred_time`만큼 등속 전파.
2. 최근접 장애물 중심을 apex로, 좌/우 빈 공간 비교(`_more_space`)로 회피
   방향과 `d_apex`(장애물 가장자리 + `evasion_dist`) 결정.
3. apex 전후 7개 절점을 자연 3차 스플라인으로 연결해 d(s) 생성, 0.1 m 샘플.
4. 트랙 경계까지 `spline_bound_mindist` 미만이면 회피 포기(빈 배열 발행),
   raceline에서 벗어난 상태(|d| > `side_switch_d_tresh`)에선 사이드 변경 금지.
5. 코너 안쪽 통과 시 속도 `inside_speed_scale`로 감속, 바깥 추월 시 스플라인
   `outside_spline_scale`배 완만화.

원본 대비 추가된 안전장치: perception 두절 시 유령 장애물을 계속 피하지
않도록 `obs_timeout_sec`(spliner) / `avoidance_timeout_sec`(local) 타임아웃.

### Predictive Spliner (동적 상대 추월, arXiv:2410.04868)

`opponent_predictor_node`(scripts/planner/, rclpy)가 동적 상대의 관측
(s, d, vs)을 수집해 **GP 회귀로 상대의 주행 라인 d(s)와 속도 프로파일
vs(s)를 학습**하고, 자차/상대 속도 프로파일을 전방 적분해 "따라잡는
시점의 충돌 구간"을 계산한다. 그 구간을 가상 장애물로
`/opponent_prediction/obstacles`에 발행하면, spliner가
(`spliner/use_prediction: true`) 같은 id의 원시 관측 대신 **예측 구간
주위로 회피 스플라인을 생성** — 등속 전파보다 빠른 상대도 추월 가능.

시뮬 검증: 30% 속도 상대를 12 m 앞에 스폰 → 4 s 만에 GP 학습·예측 발동
(6.5 m 밖에서 18 m 앞 충돌구간 예측) → 9.5 s에 추월 완료, 최소 이격 0.52 m.

시뮬에서 상대 스폰: RViz **2D Goal Pose** 클릭 (재클릭 시 제거,
`virtual_opponent_node`, 속도는 `opponent/speed_scale`). GP는 numpy만
사용(RBF+Cholesky, 학습점 150 제한)이라 Jetson에서도 ms급.

검증: colcon 빌드 + 가짜 odom/장애물 시나리오 (장애물 등장 → apex d=0.8 m
회피 스플라인 + local 67/80 wpt 치환, 소멸 → 0.5 s 내 raceline 복귀 확인).

## RViz 인터랙티브 레이스라인 튜너

HMCL unicorn 튜토리얼의 raceline_tuner를 ROS2로 구현한 것. 코너별로
라인과 속도를 RViz에서 드래그로 조절한다. RViz 설정은 `sim_race.rviz`
하나로 통합돼 있다 (튜너 핸들/미리보기 디스플레이 포함):

```bash
# 방법 1: 시뮬과 함께 (같은 RViz에서 주행 보면서 편집)
ros2 launch planning_pkg sim_race.launch.py tuner:=true

# 방법 2: 튜너만 단독 실행 (gym 불필요, 맵 + 튜너 + RViz)
ros2 launch planning_pkg raceline_tuner.launch.py            # 기본 IQP 라인 편집
ros2 launch planning_pkg raceline_tuner.launch.py csv:=<파일명>.csv
```

- **보라 구체** 드래그 → 주변 ±8 m가 코사인 falloff로 함께 이동
  (횡방향 성분만 적용, 벽에서 0.35 m 이내로는 못 들어가게 클램프)
- **초록 수직 핸들** 드래그 → 그 구간 속도 조절 (높이 = 속도, 텍스트 표시)
- 아무 핸들 **우클릭 메뉴**: 속도 재계산(마찰한계, 라인 수정 후 권장) /
  실행취소(5회) / 전체 리셋 / CSV 저장
- 미리보기: 파랑(느림)→빨강(빠름) 라인이 실시간 갱신

저장은 **소스 트리의 CSV를 직접 수정**하고 원본은 `.bak`으로 백업한다.
저장 후 `colcon build --packages-select planning_pkg` 해야 런타임에 반영.

## 포팅 TODO (통합 시 남은 작업)

1. **인터페이스 확정** — spliner_node는 `/perception/obstacles`
   (ae_hyu_msgs/ObstacleArray, Frenet s/d 필드 사용)를 구독한다. perception
   팀이 이 타입·토픽으로 발행하는지 합의 필요 (d>0 = 진행방향 왼쪽,
   장애물이 없어도 빈 배열을 주기 발행할 것 — 타임아웃 기반 안전장치와 연동).
2. ~~planning_node 정리~~ — **해결**: control_ws의 `control_node`(C++ pure
   pursuit + 스턱 복구 + E-stop)가 `/local_waypoints`를 직접 구독해
   실차(`/ackermann_cmd`)/시뮬(`/drive`)로 명령한다 (environments 스위치).
   `planning_node`(String 골격)는 이제 불필요 — 삭제 후보.
3. **대안 플래너 비교** — `scripts/global_traj_sim`의 step7(path_selection) /
   step8(frenet) 플래너와 spliner 방식을 시뮬레이션으로 비교. unicorn 스택의
   sqp_planner / lane_change_planner도 참고 후보.
4. **H2H 상태머신** — BehaviorState 메시지는 정의만 있고 발행 노드가 없다.
   unicorn `state_machine` 참고해 추월 허용 구간/쿨다운 로직 추가.
   추월 차선용 SP 라인 CSV(`{맵}_sp_*.csv`)는 이미 생성되므로, unicorn
   `lane_change_planner`처럼 추월 시 SP 라인으로 갈아타는 방식으로 확장 가능.

원본 개발 이력은 리포 최상위 `planning_ws/global_traj_sim/`에 남아 있다
(이 패키지의 `scripts/`가 그 이식본).
