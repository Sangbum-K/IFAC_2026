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
| `scripts/global_traj_sim/ros2/raceline_viz_node.py` | RViz 시각화 노드 (rclpy) | 완성 |
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

구성: gym 시뮬(Spielberg 맵, raceline 시작점 스폰) + planner 스택 + pure
pursuit 트래커 + 가상 장애물 노드 + RViz. RViz에서:

- **Publish Point**로 트랙 위 클릭 → 정적 장애물 생성 → spliner가 회피
  (빨간 실린더 = 장애물, 보라 실린더 = 회피 스플라인)
- 장애물 근처(0.5 m)를 **다시 클릭 → 제거**
- **2D Pose Estimate** → 차량 위치 재배치 (frenet/local 노드에 pose-jump
  전체 재탐색 폴백이 있어 어디로 옮겨도 즉시 재추종한다)

시뮬 전용 노드 (`scripts/sim/`, 실차에선 팀 노드로 대체):

| 노드 | 역할 |
|---|---|
| `waypoint_tracker_node.py` | `/local_waypoints` pure pursuit → `/drive`. **speed_scale 0.5 기본** — 순정 pure pursuit는 raceline 최고속(7 m/s)에서 스핀아웃하므로 절반 속도로 데모 (0.5에서 횡편차 ±6 cm 검증). **벽 충돌 복구 내장**: gym은 충돌 시 차를 정지시키므로, 전진 명령 중 0.6 s 이상 정지하면 스턱으로 판정하고 경로 방향으로 기수가 돌아가게 후진(1.2 m/s, 최대 3 s) 후 감속 재개한다. control 팀 컨트롤러로 대체 예정. |
| `virtual_obstacle_node.py` | `/clicked_point` → Frenet 변환 → `/perception/obstacles` 10 Hz 상시 발행 (튜토리얼의 virtual_perception 역할). perception 팀 노드로 대체 예정. |

검증(headless): 3.5 m/s로 d ±6 cm 추종, 전방 8 m 클릭 장애물에 대해
|d|=0.67 m로 회피 통과 후 raceline 복귀, 재클릭 제거 확인.

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
- `raceline_viz_node.py` — config의 raceline/centerline/walls CSV를 읽어
  RViz MarkerArray로 발행 (주황 raceline + 속도 막대 + 3D 벽). rqt_reconfigure
  실시간 튜닝 + CSV 자동 리로드 지원.

```bash
ros2 run planning_pkg raceline_viz_node.py --ros-args \
  -p raceline_csv:=$(ros2 pkg prefix planning_pkg)/share/planning_pkg/config/global_trajectory/Spielberg_map_iqp_max7.00ms_margin0.15m_mu0.85.csv \
  -p centerline_csv:=$(ros2 pkg prefix planning_pkg)/share/planning_pkg/config/centerline/Spielberg_map_centerline.csv \
  -p walls_csv:=$(ros2 pkg prefix planning_pkg)/share/planning_pkg/config/centerline/Spielberg_map_walls.csv
```

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

## RViz 인터랙티브 레이스라인 튜너 (raceline_tuner.launch.py)

HMCL unicorn 튜토리얼의 raceline_tuner를 ROS2로 구현한 것. 코너별로
라인과 속도를 RViz에서 드래그로 조절한다:

```bash
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
