# planning_pkg

F1TENTH 플래닝 패키지. 팀 인터페이스 규약:

```
/perception/obstacles ──▶ spliner ─┐
                                   ├─▶ /local_waypoints ──▶ control_node
/global_waypoints ──▶ local_traj ──┘        (여기)           (control_ws)
```

ROS 2 Humble / ament_cmake. `launch/planning.launch.py`(리포 최상위)로 실행.

## 구성 개요

| 위치 | 내용 | 상태 |
|---|---|---|
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
| `maps/` | 개발용 트랙 맵 (대회 맵은 교체) | Spielberg, Budapest |
| `scripts/global_traj_sim/` | **오프라인 개발 도구** (.venv Python) | 완성·검증됨 |
| `scripts/sim/` | 시뮬 전용 노드 3개 (tracker/obstacle/opponent) | 완성·검증됨 |
| `scripts/tuner/`, `scripts/lib/` | 레이스라인 튜너, pitwall 텔레메트리 | 완성 |
| `../pitwall_panel/` | RViz Pitwall 패널 (Qt 플러그인) | 완성 |
| `../f1tenth_planning_msgs/` | 공용 메시지 (Wpnt/Obstacle/BehaviorState) | 이식됨 — perception/control도 이 타입 사용 권장 |

> **이식 노트**: 원본 `planning_ws/planner_pkg`(리포 최상위)는 헤더 4개,
> CMakeLists.txt, planner_config.yaml, visualizer 콜백 구현 2개가 누락된
> 미완성 패키지였다. 이식하면서 전부 복원/신규 작성했고, share 디렉토리
> 참조를 `planning_pkg`로, 기본 CSV를 Spielberg IQP raceline으로 맞췄다.
> **첫 colcon build는 ROS2 Humble 머신에서 해봐야 한다** (이 Mac엔 ROS가
> 없어 정적 검토까지만 완료).

### 빌드 & 실행 (ROS2 Humble 머신)

```bash
cd main/planning_ws
colcon build            # f1tenth_planning_msgs -> planning_pkg 순서로 자동 빌드
source install/setup.bash
ros2 launch planning_pkg planner_stack.launch.py                 # sim (기본)
ros2 launch planning_pkg planner_stack.launch.py odom_mode:=real # 실차
```

## RViz 시뮬레이션 (sim_race.launch.py)

unicorn 튜토리얼(basic-simulator-usage)의 `race.launch sim:=true`에 대응하는
데모. 그 튜토리얼의 스택은 ROS1이라 이 머신(Humble)에서 직접 실행이 안 되므로,
`~/sim_ws`의 f1tenth_gym_ros(ROS2) 시뮬레이터에 이 planning 스택을 연결했다.

```bash

source ~/sim_ws/install/setup.bash                  # f1tenth_gym_ros (필수)
source ~/IFAC_ws/planning_ws/install/setup.bash
ros2 launch planning_pkg sim_race.launch.py                    # my_track0807 (기본)
ros2 launch planning_pkg sim_race.launch.py map:=Budapest_map  # 헝가로링 (코너 다수)
```

구성: gym 시뮬(**2-agent** — ego + 물리 상대 차량) + planner
스택 + pure pursuit 트래커 + Pitwall 시뮬 노드들 + RViz. 상대 차량은 시작
시 맵 구석 '주차장'(레이스라인에서 30 m+, ego LiDAR 사거리 밖)에 있다.

**맵 전환/추가** (`map:=` 인자): 맵별 raceline CSV·centerline·시작 포즈(CSV
첫 행에서 자동)·주차 위치를 launch가 알아서 맞춘다. 새 맵 추가는
`maps/<맵>_map.{png,yaml}` 넣고 → `build_map.py <맵>_map` 실행(속도·마찰·
여유는 파일 상단 설정 블록) → launch의 `PARK_POSES`에 주차 좌표 한 줄 →
colcon build. 생성기는 기하·차체 footprint·조향 속도·폐루프 랩 검증을 모두
통과해야 CSV를 원자적으로 저장한다. f1tenth_racetracks의
맵을 그대로 쓸 수 있다 (Budapest가 그 예).

**속도 프로파일**: CSV 파일명의 `max...ms`는 생성 상한이고 실제 명령은
마찰원·가감속 제한으로 만든 CSV 속도에 environment의 `velocity_scale`과
control의 하드 속도 상한을 다시 적용한다. 스케일을 1보다 크게 올려 생성
검증을 우회할 수 없으며, 더 높은 속도가 필요하면 해당 마찰계수와 차량
모델로 CSV를 다시 생성하고 폐루프 검증을 통과시켜야 한다.

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
| `waypoint_tracker_node.py` | `/local_waypoints` pure pursuit → `/drive`. 기본 `speed_scale=0.30`; waypoint·odom 원본 timestamp와 recovery state/target paired lease가 신선할 때만 주행한다. 후진은 기본 금지이고 스턱 시 STOP-only다. **ego 모드 먹스**: `/sim/ego_mode`("autonomous"/"manual") 또는 `/joy` LB=manual·RB=autonomous. manual 진입도 연속 0 명령을 발행한다. |
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

검증(headless): Spielberg 7/7 · Budapest 5/5 PASS — 섹터 마커 발행(13개/5개),
속도 변화(실속 1.9~7.5 m/s), 비섹터 장애물 TRAILING 정지(3.0 m 앞, 관통 없음),
제거 후 재가속, 상대 스폰 → 섹터 추월(최소 이격 0.51 m / 0.45 m).
테스트: `test/sim/run_sector_test.sh [맵]` (시뮬을 직접 띄우고 검사 —
주의: 러너 호출 명령줄에 "planning_pkg" 문자열이 있으면 cleanup pkill이
자기 셸을 죽인다. 경로로만 실행할 것).

### 추월 섹터 (sector gating)

"대부분의 상황에서 추월 못 하고 뒤만 따라가는" 문제의 해결책. spliner가
raceline CSV의 좌우 벽 여유에서 **추월 가능 섹터를 자동 산출**하고
(`clearance >= ot_min_clearance`, `ot_sector_min_len_m` 이상 연속 구간),

- **섹터 안**: 경계·차체 footprint·조향·마찰원·가감속·jerk 검사를 모두
  통과한 회피만 발행한다. 기본 `ot_bound_mindist=0.20`,
  `ot_speed_scale=1.0`으로 경계 여유를 줄이거나 검증된 속도 프로파일을
  런타임에서 부스트하지 않는다.
- **섹터 밖**: 회피 자체를 시도하지 않고 **TRAILING**으로 따라붙어
  다음 섹터에서 승부

RViz의 초록 반투명 라인(`/planner/ot_sectors/markers`)이 섹터다.
`spliner/sector_gating: false`로 끄면 이전 동작(전 구간 시도),
`spliner/ot_sectors: [s0,s1, ...]`로 수동 지정도 가능 (빈 리스트는 금지 —
rclcpp 타입 오류로 노드가 죽는다). 섹터 진입/산출은 pitwall 이벤트로 찍힌다.

> `spliner/evasion_dist`는 0.65가 이 raceline(safe-dist 0.5)의 실질 최대다.
> 0.7 이상이면 한쪽 벽 여유(~1.05 m)를 넘어 트랙경계 abort로 회피가 무산된다.
> TRAILING 폴백: raceline을 유지한 채 장애물 뒤 `trailing_gap_m`(1.5 m)에서
> 장애물 속도(정적이면 0)까지 `trailing_decel`(2.5 m/s²)로 감속 — 정면
> 돌진/관통 방지 (ForzaETH 상태머신의 TRAILING 대응, 실차에서도 동일).
>
> **주의 (시뮬↔실차 스케일)**: `spliner/cmd_speed_scale`은 컨트롤러의 wpnt
> 속도 스케일(`tracker/speed_scale`)과 반드시 일치해야 한다 (sim 0.30, 실차
> 1.0). 불일치하면 동적 상대 trailing에서 상대보다 느리게 명령돼 격차가
> 벌어진다 (시뮬 실측: 8 m 표류). `spliner/lookahead_m`도 접근 속도에 맞출
> 것 — gym 제동은 속도비례라 실속 7.5 m/s에선 10 m 감지로는 못 멈춘다 (15 m로 상향).

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
#  - 속도/마찰/벽 여유는 build_map.py 상단 설정 블록에서 지정
python scripts/global_traj_sim/build_map.py Spielberg_map
# 대회 맵은 <workspace>/maps 또는 planning_pkg/maps에 yaml+image를 둔다
# 생성 후 colcon build로 install에 복사해야 런타임 노드가 새 CSV를 읽는다
```

### 2. 런타임 노드 (ROS 2, colcon 빌드)

- raceline 추종/회피 노드는 위 표 참고. 시뮬 실행은 위
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

### 이중장애물 재계획 + 안전 정지 (mina Task 2 병합)

회피 후보 경로를 **lookahead 내 모든 장애물**(raceline 근접 여부 무관)과
(s,d) 박스 충돌검사(`path_check_margin` 횡 여유)한다. 폴백 사다리:

1. 1차 후보(MoreSpace 사이드)가 다른 장애물에 막히면 → **반대 사이드 재계획**
   (`DoSpline(forced_side)`; 이때 사이드 스위치 금지 예외 + 차량 현재
   (cur_s, cur_d)를 첫 절점으로 쓰는 ego-anchor로 path jump 방지)
2. 좌·우 모두 막히면 → TRAILING (장애물 뒤 감속 추종)
3. trailing 경로(d=0)마저 **다른** 장애물에 막히면 → 차단 지점 앞
   `stop_margin_m`에서 vx=0 **안전 정지 경로** (`BuildStopWpnts`,
   감속은 `trailing_decel` 물리 한계와 `stop_ramp_dist_m` 선형 램프 중 낮은 쪽)

충돌검사에서 회피/추종 대상 장애물 자신은 id로 제외한다 (스플라인은 그
옆을, trailing은 그 뒤를 의도적으로 지나므로).

### 클러스터링 + 적응형 회피 (회피포기 남발 해결)

스크린샷처럼 대각선 이중장애물 앞에서 "왼쪽으로 크게 돌지 못하고
회피포기"하던 문제의 해결 구조:

1. **클러스터링**: 사이를 통과할 수 없는 배치(s 간격 < `cluster_gap_s_m`,
   corridor < `cluster_gap_d_m`)를 하나의 (s,d) 박스로 병합 — 개별로 보면
   최근접 기준 고정 폭 회피만 시도하다 포기한다. unwrap 기준은 ego 뒤
   15m (ego가 장애물 사이에 진입해도 클러스터가 쪼개지지 않게).
2. **적응형 apex**: 목표 간격은 `evasion_dist`(0.65)지만 벽이 좁으면
   corridor 여유를 장애물/벽에 균등 배분한 지점까지 줄여서 시도, 하한
   `evasion_dist_min`(0.35 = 차 반폭+여유). 벽 마진(`ot_bound_mindist`
   0.25)은 차 반폭 포함 — 0.1로 깎으면 기하 판정은 통과인데 차 모서리가
   벽을 스친다 (실측).
3. **squeeze 감속**: apex가 축소된 회피는 `squeeze_speed_scale`(0.35)로
   감속 + 진입/복귀 절점 2배 연장 — 추적오차는 속도 비례라 전속 squeeze가
   벽 접촉 원인이었다. apex는 점이 아니라 클러스터 s 구간 전체(±0.5m)에서
   유지 (진입/이탈 대각선 커팅 방지).
4. **경로 커밋**: 정적 장애물 회피는 시작하면 경로 끝까지 유지 — 매 사이클
   재계획 + '장애물 지남' 판정으로 회피가 도중에 사라지면 복귀 중인 차가
   raceline 전속 경로로 순간 전환돼 반대편 벽을 친다 (실측). 새 장애물이
   커밋 경로를 막으면 즉시 폐기·재계획.
5. **사이드 유지**: 회피 중(|d| 큼) MoreSpace가 반대 사이드를 제안하면
   거부(→trailing 붕괴) 대신 현재 사이드 유지로 재계산.
6. **예측+원시 공존**: Predictive Spliner의 충돌구간이 원시 관측을
   대체하지 않고 함께 유지(별도 id) — 클러스터링이 상대의 현재 위치~
   따라잡는 지점을 한 박스로 합쳐 전체를 회피한다. 대체하면 복귀 지점에서
   상대와 접촉 (실측 0.01m → 0.67m로 개선).

그래도 안 되는 배치(기하 불성립)는 TRAILING/안전정지가 맞는 동작이다.

### 추월 지점 선택 (코너 제외 + SP 레인체인지)

- **섹터 = 직선만**: 섹터 자동 산출에 `ot_max_kappa`(0.1 = 곡률반경 10m
  이상) 조건 추가. 코너 아웃라인 추월은 경로가 길어져 페이스 우위가
  상쇄되므로 — 코너에선 TRAILING으로 붙고 직선에서 속도 차로 제친다.
  단, 동적 상대를 이미 추월 중(|d| > side_switch_d_tresh)이면 섹터
  이탈로 공격을 끊지 않는다 (끊기면 trailing 복귀 램프가 차를 상대
  위치로 끌어당겨 접촉 — 실측).
- **SP 레인체인지** (`sp_lane_change`): 동적 상대에 한해 IQP 오프셋 대신
  SP(최단거리) 라인으로 갈아타는 경로를 우선 시도 — 섹터 여부와 무관
  (코너 인사이드 찌르기용). SP·IQP 라인이 그 구간에서 충분히 벌어질
  때만 검증을 통과하므로 좁은 트랙(Spielberg: 최대 벌어짐 1.13m)에선
  드물게 발동하고, 넓은 트랙에서 효과가 커진다.
- **trailing 접근 물리** (gym 1s 시정수 실측 보정): 감속 개시는
  `max(trailing_engage_dist, 실속×1.6+3)`m 앞, gap은 2.5m (1.5면 램프
  추종오차 ~1.3m로 범퍼 추돌), gap 안쪽 침투 시 목표속도 0.85배로
  간격 재확보.

### 시뮬 속도 (실제 F1TENTH 대회 기준)

ForzaETH(대회 다수 우승, IROS 2024) 기준 레이스 평균 ~5 m/s (35m 트랙
7.14s 랩), 직선 최고 7~10 m/s. ego 실속 1.9~7.5(평균 ~4)는 대회 페이스와
일치. opp 기본값은 ego의 ~85%(`opponent/speed_scale` 0.42, cap 8.0) —
현실적 경쟁 구도로, 추월엔 수십 초~수 분 걸린다 (테스트는 결정성을 위해
opp cap을 1.2로 낮춰 실행).

### 복구 감독 (recovery_supervisor_node)

벽 충돌/진행불능(stuck) 판정 상태머신. Planning은 판정+요청만 하고 실행은
control_node 책임: `/planner/recovery/state`(String),
`/planner/recovery/target_speed`(Float64; DRIVING=NaN, 정지=0.0, 후진=음수).
STOP은 래치 — `/planner/recovery/reset`(Bool true)로만 해제.
E-stop(`/safety/emergency_stop`)과 odom/perception/local_waypoints 두절은
항상 최우선 STOP이다.

`planner_stack.launch.py`는 기본 `recovery:=true`로 감독기를 기동한다.
실차·race·sim control 환경과 sim tracker는 state/target 두 토픽을 하나의
짧은 paired lease로 소비한다. 한쪽 누락, timestamp stale/skew, 알 수 없는
상태, state와 target 불일치 중 하나라도 있으면 즉시 0 명령이다.

- 부팅 중에는 모든 입력의 receipt와 원본 timestamp가 신선하고 차량이 실제로
  정지한 상태가 dwell 동안 유지될 때까지 `WAITING_FOR_INPUTS`에서 0을 발행한다.
- 주행 준비 후 입력이 끊기거나 비정상이면 `STOP`으로 래치하며 자동 재개하지
  않는다.
- 현재 차체 직사각 footprint와 속도 기반 반응거리+제동거리 전체의 경계를
  검사한다. local horizon이 짧아 제동거리까지 증명할 수 없어도 정지한다.
- 후진은 Planning·Control 양쪽 opt-in과 명시적인 후방 sensor coverage가 모두
  필요하다. 기본 설정은 `allow_reverse=false`, `rear_coverage_confirmed=false`
  이므로 stuck/collision은 STOP-only다.
- Spliner 경계 요구값과 recovery 요구값은 모두 차량 반폭과 추적 여유를 포함해
  계산한다. 어느 한쪽의 margin을 줄여 다른 쪽을 우회하면 안 된다.

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
ros2 launch planning_pkg sim_race.launch.py map:=my_track0807 tuner:=true

# 방법 2: 튜너만 단독 실행 (gym 불필요, 맵 + 튜너 + RViz)
#   기본값이 my_track0807 기준 (stride 10 / falloff 3.13 / v_max 7.0 / 계획 mu 0.68)
ros2 launch planning_pkg raceline_tuner.launch.py

#   다른 맵은 map:= 하나만 — CSV는 <맵>_iqp_*.csv 로 자동 탐색
ros2 launch planning_pkg raceline_tuner.launch.py \
    map:=Spielberg_map stride:=25 falloff:=8.0 v_max:=15.0 mu:=0.85
```

- **보라 구체** 드래그 → 주변이 코사인 falloff(`falloff:=`, 기본 ±8 m)로 함께 이동
  (횡방향 성분만 적용, 벽에서 0.35 m 이내로는 못 들어가게 클램프)
- **초록 수직 핸들** 드래그 → 그 구간 속도 조절 (높이 = 속도, 텍스트 표시)
- 아무 핸들 **우클릭 메뉴**: 속도 재계산(마찰한계, 라인 수정 후 권장) /
  실행취소(5회) / 전체 리셋 / CSV 저장
- 미리보기: 파랑(느림)→빨강(빠름) 라인이 실시간 갱신

저장은 **소스 트리의 CSV를 직접 수정**하고 원본은 `.bak`으로 백업한다.
저장 후 `colcon build --packages-select planning_pkg` 해야 런타임에 반영.

> `sim_race.launch.py tuner:=true` 는 CSV 경로만 넘기므로 stride/falloff/v_max/mu 가
> **노드 기본값(25 / 8.0 / 12.0 / 0.85) 고정**이다 — my_track0807에선 드래그 범위가
> 트랙 절반이 되고 "속도 재계산"이 12 m/s까지 올린다. 편집이 목적이면 방법 2를 쓸 것.
> 자세한 건 [`scripts/tuner/README.md`](scripts/tuner/README.md).

## 포팅 TODO (통합 시 남은 작업)

1. **인터페이스 확정** — spliner_node는 `/perception/obstacles`
   (f1tenth_planning_msgs/ObstacleArray, Frenet s/d 필드 사용)를 구독한다. perception
   팀이 이 타입·토픽으로 발행하는지 합의 필요 (d>0 = 진행방향 왼쪽,
   장애물이 없어도 빈 배열을 주기 발행할 것 — 타임아웃 기반 안전장치와 연동).
2. ~~planning_node 정리~~ — **완료 (2026-07-27 삭제)**: control_ws의
   `control_node`(C++ pure pursuit + E-stop)가 `/local_waypoints`를 직접
   구독해 실차(`/autonomy/ackermann_cmd`)로 명령하므로 String 골격
   `planning_node`는 불필요 — 소스·CMake 타깃·params 블록 모두 제거함.
3. **대안 플래너 비교** — `scripts/global_traj_sim`의 step7(path_selection) /
   step8(frenet) 플래너와 spliner 방식을 시뮬레이션으로 비교. unicorn 스택의
   sqp_planner / lane_change_planner도 참고 후보.
4. **H2H 상태머신** — BehaviorState 메시지는 정의만 있고 발행 노드가 없다.
   Spliner의 섹터·trailing·SP 전환은 구현돼 있지만, 독립적인 경기 규칙
   상태머신과 쿨다운은 추가 통합 항목이다. SP는 자동 활성화하지 않으며 메인과
   동일한 map/suffix로 생성·검증된 파일을 명시적으로 지정하고
   `sp_lane_change=true`로 opt-in해야 한다.

원본 개발 이력은 리포 최상위 `planning_ws/global_traj_sim/`에 남아 있다
(이 패키지의 `scripts/`가 그 이식본).
