# planning_ws — F1TENTH 시뮬레이션 & 플래닝 스택

ROS 2 **Humble** 기반. `f1tenth_gym_ros`(2-agent 물리 시뮬) 위에서
전역 raceline · Frenet 변환 · 회피 스플라인 플래너 · pure pursuit 컨트롤러 ·
Pitwall 패널을 한 번에 올려 RViz로 추월/회피를 검증한다.

핵심 패키지:
- `planning_pkg` — 플래닝 스택 전체 (C++ 노드 + 시뮬/튜닝용 Python 노드)
- `f1tenth_planning_msgs` — Obstacle/Wpnt 등 메시지 정의
- `f1tenth_perception_msgs` — perception 팀 메시지 (팀 zip에서 벤더링, 브릿지 빌드용)
- `pitwall_panel` — RViz 제어 패널 (RViz 플러그인)

## 실차 통합 (2026-07-20)

팀원 워크스페이스(control/hardware/perception/localization) 인터페이스에 맞춰
planning_ws만 수정해 통합했다. **실차 실행:**

```bash
ros2 launch planning_pkg planner_stack.launch.py env:=real_car
```

`env:=real_car`가 추가로 하는 일 (sim 기본값은 무변경):

| 간극 | 해결 |
|---|---|
| perception 팀은 `/perception/obstacles`를 **f1tenth_perception_msgs** 타입, **laser frame 카르테시안**으로 발행 — spliner는 f1tenth_planning_msgs frenet 타입 기대 | `perception_obstacle_bridge_node`가 TF(laser→map)+frenet 변환해 `/perception/obstacles_frenet` 발행. spliner/recovery는 `spliner/obstacle_topic`·`recovery/obstacle_topic`으로 그 토픽 구독 |
| control·frenet(real)·recovery가 기대하는 `/pf/pose/odom`을 아무도 발행 안 함 (localization은 slam_toolbox/amcl → TF만) | `pose_odom_bridge_node`가 TF map→base_link + `/odom`(vesc) twist를 합성해 발행. TF 0.5s 두절 시 발행 중단 → control odom_timeout 정지 |
| `environments/real_car.yaml`이 launch에서 로드되지 않았음 | `planner_stack.launch.py env:=<이름>` 인자 추가 (params.yaml 위에 override) |
| real_car.yaml의 `spliner/stop_margin_m` 등 존재하지 않는 파라미터 | 제거 (정지 거동은 trailing_* 파라미터가 담당) |

주의: perception의 `velocity`/`is_dynamic`은 laser frame 기준이라 ego 이동이
섞여 있음 — 브릿지가 map-frame 위치 시간차분으로 vs/vd를 다시 추정하고,
최종 정적/동적 판정은 spliner 재분류(`cls_*`)가 담당한다.

> `unicorn-racing-stack-jazzy/`는 **다른 팀 참고용 소스**로, 이 스택 실행과 무관하다.

---

## 1. 워크스페이스 구조

시뮬 실행에 필요한 워크스페이스는 세 개다. **각각 자기 폴더 안에서** 빌드한다.

| 워크스페이스 | 경로 | 역할 |
|---|---|---|
| `sim_ws` | `~/sim_ws` | `f1tenth_gym_ros` 물리 시뮬레이터 (외부, 최초 1회만 빌드) |
| `planning_ws` | `~/IFAC_ws/planning_ws` | 플래닝 스택 (이 워크스페이스) |
| `control_ws` | `~/IFAC_ws/control_ws` | C++ 컨트롤러 (선택 — `tracker:=false`일 때만 필요) |

> ⚠️ **`~/IFAC_ws` 최상위에서 `colcon build`를 실행하지 말 것.**
> 상위에서 빌드하면 `~/IFAC_ws/build`, `~/IFAC_ws/install`이 새로 생겨
> 각 워크스페이스의 install을 가리게 되고, source가 꼬여 노드가
> `Waiting for global waypoints...`에서 멈춘다. 빌드는 항상
> `planning_ws/` 안으로 `cd` 한 뒤 실행한다.

---

## 2. 빌드 (최초 1회 / 소스 수정 후)

```bash
# 시뮬레이터 (한 번만)
cd ~/sim_ws
colcon build

# 플래닝 스택
cd ~/IFAC_ws/planning_ws
colcon build

# (선택) C++ 컨트롤러 — tracker:=false 로 쓸 때만
cd ~/IFAC_ws/control_ws
colcon build
```

> ⚠️ **`--symlink-install`을 쓰지 말 것.** 이 패키지의 Python 노드들은
> 같은 폴더의 `pitwall.py`를 `import pitwall`로 불러오는데, symlink 빌드는
> 노드를 `scripts/sim/` 원본을 가리키는 심링크로 만든다. 그러면 파이썬이
> `sys.path`를 그 원본 폴더로 잡아 `pitwall.py`(→ `scripts/lib/`에 있음)를
> 못 찾고 **모든 파이썬 노드가 죽는다** — 컨트롤러(waypoint_tracker)가
> 죽으면 차가 안 움직이고, map_server activate도 실패해 맵이 검게 뜬다.
> 일반 `colcon build`는 `pitwall.py`를 노드 옆에 복사하므로 import가 된다.
> **Python 노드/config를 수정하면 다시 `colcon build`** 해야 반영된다.

---

## 3. 실행

### source 순서 (중요)

반드시 아래 순서로 source 한다. 순서가 틀리거나 하나라도 빠지면
`package not found` 또는 `Waiting...` 멈춤이 발생한다.

```bash
source /opt/ros/humble/setup.bash
source ~/sim_ws/install/setup.bash
source ~/IFAC_ws/planning_ws/install/setup.bash
source ~/IFAC_ws/control_ws/install/setup.bash   # 선택
```

편의를 위해 저장소에 **`sim_env.sh`** 헬퍼를 넣어두었다. 매번 위 줄을
치는 대신:

```bash
source ~/IFAC_ws/planning_ws/sim_env.sh
```

### 런치

```bash
# Spielberg (기본)
ros2 launch planning_pkg sim_race.launch.py

# Budapest (코너 다수 — S자/헤어핀 테스트)
ros2 launch planning_pkg sim_race.launch.py map:=Budapest_map
```

### launch 인자

| 인자 | 기본 | 설명 |
|---|---|---|
| `map` | `Spielberg_map` | 맵 이름. 현재 `Spielberg_map`, `Budapest_map` 지원 |
| `rviz` | `true` | RViz 실행 여부 |
| `tracker` | `true` | Python pure pursuit 트래커. `false`면 control_ws의 C++ `control_node` 사용 |
| `tuner` | `false` | 레이스라인 튜너 동시 실행 (같은 RViz에서 라인/속도 드래그 편집) |

---

## 4. RViz / Pitwall 패널 사용법

- **Publish Point** — 트랙 위 클릭 → 정적 장애물 생성 → spliner가 회피.
  같은 장애물 근처를 다시 클릭하면 제거.
- **2D Goal Pose** — 상대 차량(gym 2번째 에이전트)을 그 위치로 텔레포트 = **스폰**.
  시작 시 상대는 맵 구석 '주차장'(ego LiDAR 사거리 밖)에 있다.
- **2D Pose Estimate** — ego 위치 재배치.
- **초록 반투명 라인** — 자동 산출된 추월 섹터 (`/planner/ot_sectors/markers`).
- **Pitwall 패널** (Panels ▸ PitwallPanel): STATE 배너 · 이벤트 로그 ·
  ego Autonomous/Manual · 상대 모드·속도±·제거 · 장애물 Clear ·
  LiDAR 토글 · Perception 주입 모드(tracking_merge / lidar_overlay).

---

## 5. 맵 추가

1. `maps/`에 `<맵>_map.png`, `<맵>_map.yaml`을 넣는다.
2. raceline을 생성한다:
   ```bash
   cd ~/IFAC_ws/planning_ws/src/planning_pkg/scripts/global_traj_sim
   python3 step4_main.py --map <맵>_map --raceline iqp \
       --safe-dist 0.5 --v-max 15
   ```
   → `config/global_trajectory/`에 `_iqp_` / `_sp_` CSV가 생성된다.
3. `launch/sim_race.launch.py`의 `PARK_POSES`에 상대 주차 좌표
   (레이스라인에서 30m+ 떨어진 맵 구석)를 추가한다.
4. `colcon build` 후 `map:=<맵>_map`으로 실행.

CSV 파일명 접미사(`max15.00ms_margin0.15m_mu0.85`)는 launch의
`RACELINE_SUFFIX`와 일치해야 한다.

---

## 6. 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `package 'planning_pkg' not found` | planning_ws를 source 안 함 | §3 source 순서대로 다시 source |
| `package 'f1tenth_gym_ros' not found` | sim_ws를 source 안 함 | `source ~/sim_ws/install/setup.bash` |
| `ModuleNotFoundError: No module named 'pitwall'` → 파이썬 노드 다 죽고 **차 안 움직임 + 검은 맵** | `--symlink-install`로 빌드해 노드가 심링크가 됨 (§2 경고) | `cd ~/IFAC_ws/planning_ws && rm -rf build install log && colcon build` (symlink 없이) |
| `executable 'xxx_node.py' not found on libexec` | Python 노드가 실행권한(+x)을 잃음 | `chmod +x src/planning_pkg/scripts/{sim,planner,lib}/*.py` |
| 노드가 `Waiting for global waypoints...`에서 멈춤 | ① 루트(`~/IFAC_ws`)에서 빌드해 install이 꼬임 ② source 순서 오류 | `~/IFAC_ws/{build,install,log}` 삭제 → 각 워크스페이스에서 재빌드 → §3 순서로 source |
| libexec 경로가 `~/IFAC_ws/install/...`로 뜸 | 상위 폴더에 실수로 만든 install이 sourced됨 | `rm -rf ~/IFAC_ws/build ~/IFAC_ws/install ~/IFAC_ws/log` 후 새 터미널에서 §3대로 source |

### 깨끗한 재시작 (꼬였을 때)

```bash
# 1. 실수로 만든 루트 산출물 제거
rm -rf ~/IFAC_ws/build ~/IFAC_ws/install ~/IFAC_ws/log

# 2. 각 워크스페이스에서 재빌드
cd ~/IFAC_ws/planning_ws && colcon build

# 3. 새 터미널을 열고 순서대로 source
source ~/IFAC_ws/planning_ws/sim_env.sh

# 4. 실행
ros2 launch planning_pkg sim_race.launch.py map:=Budapest_map
```
