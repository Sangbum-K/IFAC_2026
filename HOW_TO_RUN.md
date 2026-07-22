# IFAC_ws 실행 가이드 (처음부터 끝까지)

F1TENTH 자율주행 전체 스택(perception → planning → control)을 **시뮬레이터**와
**실차**에서 실행하는 방법입니다. ROS/터미널이 처음이어도 따라 할 수 있게
한 줄씩 설명합니다.

---

## 0. 이 워크스페이스가 하는 일

차 한 대가 트랙을 스스로 도는 데 필요한 4가지가 각각 별도 워크스페이스에
들어 있습니다.

| 워크스페이스 | 역할 | 핵심 노드 |
|---|---|---|
| `perception_ws` | LiDAR로 **장애물 찾기** | `obstacle_detector_node` |
| `planning_ws` | 레이스라인 추종 + **회피 경로 만들기** | `spliner_node` 등 |
| `control_ws` | 경로를 **핸들/속도 명령**으로 | `control_node` |
| `integration_ws` | 위 셋을 **한 번에 켜고 상태 감시** | `stack.launch.py` |

데이터 흐름 (실차와 시뮬 동일):

```
LiDAR /scan
  → [perception] 장애물 탐지 → /perception/obstacles(_detected)
  → [planning] frenet 변환 → 회피 경로 → /local_waypoints
  → [control] pure pursuit → /ackermann_cmd (실차) 또는 /drive (시뮬)
  → 차량
```

---

## 1. 최초 1회: 빌드

빌드는 **딱 이 순서**로 해야 합니다. 뒤 패키지가 앞 패키지의 메시지
(`f1tenth_planning_msgs`, `f1tenth_perception_msgs`)를 참조하기 때문입니다.

시뮬만 돌릴지, 실차까지 돌릴지에 따라 빌드할 워크스페이스가 다릅니다.

| | 시뮬 | 실차 |
|---|---|---|
| `planning_ws` (메시지 포함, 항상 먼저) | ✅ | ✅ |
| `perception_ws` | ✅ | ✅ |
| `control_ws` | ✅ | ✅ |
| `integration_ws` | ✅ | ✅ |
| `hardware_ws` (LiDAR·VESC·odom·TF·estop·mux) | — | ✅ |
| `localization_ws` (particle_filter) | — | ✅ |
| `~/sim_ws` (f1tenth_gym_ros) | ✅ 별도 | — |

### 1-A. 공통 (시뮬·실차 모두)

```bash
# ROS 2 Humble 환경 켜기 (모든 터미널에서 맨 먼저)
# 현재 터미널이 zsh면 .zsh, bash면 .bash 를 쓰세요.
source /opt/ros/humble/setup.zsh

# planning 먼저 — 여기에 메시지 패키지가 있음. 나머지는 이 순서대로.
# --base-paths src 를 반드시 붙이세요 (이유는 아래 주의사항 참고).
for ws in planning perception control integration; do
  cd ~/IFAC_ws/${ws}_ws && colcon build --base-paths src && source install/setup.zsh
done
```

### 1-B. 실차에서만 추가로

```bash
# 하드웨어 + 위치추정 (시뮬에선 gym이 대신하므로 불필요)
for ws in hardware localization; do
  cd ~/IFAC_ws/${ws}_ws && colcon build --base-paths src && source install/setup.zsh
done
```

> ⚠️ **주의**
> - **`perception_ws`에서는 `--base-paths src` 없이 `colcon build`를 하면
>   `perception_pkg`가 빌드되지 않습니다.** 워크스페이스 **루트**에 남아 있는
>   `setup.py`(+ `obstacle_detector_node.py` 등 압축 해제 잔재) 때문에 colcon이
>   루트를 `f1tenth_perception`이라는 패키지로 오인하고 `src/`로 내려가지
>   않습니다. 에러 없이 "0 packages" 로 끝나서 **고친 코드가 반영 안 된 채로
>   주행하게 되는** 함정입니다. 확인: `colcon list` 결과가
>   `perception_pkg src/perception_pkg` 여야 정상.
> - `--symlink-install` 쓰지 마세요. 기존 빌드 캐시와 충돌해서 이상한
>   에러(`ae_hyu_msgs` / `ament_cmake_python` 심링크 에러)가 납니다.
> - `zsh` 터미널에서 `setup.bash`를 source 하면
>   `.../local_setup.bash not found` 같은 오류가 날 수 있습니다.
>   `zsh`에서는 반드시 `setup.zsh`를 쓰세요.
> - 빌드가 꼬이면 그 워크스페이스의 `build/ install/ log/`를 **통째로 지우고**
>   다시 빌드하세요: `rm -rf build install log && colcon build`
> - 다른 PC에서 복사해 온 워크스페이스라서
>   `CMakeCache.txt directory is different` 에러가 나면 먼저
>   `colcon build --cmake-clean-cache`를 한 번 실행하세요.
> - 시뮬을 돌리려면 별도로 `~/sim_ws`(f1tenth_gym_ros)가 빌드돼 있어야 합니다.

---

## 2. 매번 실행 전: source (환경 켜기)

새 터미널을 열 때마다 아래를 **순서대로** 복사해서 붙여넣으세요.
(source = "이 워크스페이스를 쓸 수 있게 등록"하는 명령. 빌드가 아니라 매번 필요.)

**시뮬용** (gym 포함):

```bash
source /opt/ros/humble/setup.zsh
source ~/sim_ws/install/setup.zsh
source ~/IFAC_ws/planning_ws/install/setup.zsh
source ~/IFAC_ws/perception_ws/install/setup.zsh
source ~/IFAC_ws/control_ws/install/setup.zsh
source ~/IFAC_ws/integration_ws/install/setup.zsh
```

**실차용** (`sim_ws` 대신 `hardware_ws` + `localization_ws`):

```bash
source /opt/ros/humble/setup.zsh
source ~/IFAC_ws/planning_ws/install/setup.zsh
source ~/IFAC_ws/perception_ws/install/setup.zsh
source ~/IFAC_ws/control_ws/install/setup.zsh
source ~/IFAC_ws/integration_ws/install/setup.zsh
source ~/IFAC_ws/hardware_ws/install/setup.zsh
source ~/IFAC_ws/localization_ws/install/setup.zsh
```

> 💡 매번 치기 귀찮으면 해당 블록을 `~/.bashrc` 맨 아래에 넣어두면 터미널 열 때
> 자동으로 됩니다. (실차 PC엔 실차용, 시뮬 PC엔 시뮬용 블록.)
> 💡 실차는 터미널 A/B/C가 각각 다른 launch를 쓰지만, 위 블록 하나를 모든
> 터미널에서 source 해두면 안전합니다 (필요한 것만 골라 쓰게 됨).

---

## 3. 시뮬레이터로 실행하기 (가장 많이 쓰는 방법)

### 3-1. 한 줄로 전체 켜기

터미널 하나에서 (위 2번 source 먼저 하고):

```bash
ros2 launch integration_pkg stack.launch.py
```

이 한 줄이 아래를 전부 띄웁니다:
- f1tenth_gym_ros 시뮬레이터 (차 2대: 내 차 + 상대 차)
- 실제 perception 파이프라인 (LiDAR → 장애물 탐지)
- planning 스택 (레이스라인 추종 + 회피)
- control_ws의 C++ 컨트롤러
- RViz (화면)
- integration 헬스 모니터

차가 곧바로 레이스라인을 따라 달리기 시작합니다.

### 3-2. 다른 맵 / RViz 끄기

```bash
# 헝가로링(코너 많은 맵)
ros2 launch integration_pkg stack.launch.py map:=Budapest_map

# 화면 없이 (테스트/서버용)
ros2 launch integration_pkg stack.launch.py rviz:=false
```

사용 가능한 맵: `Spielberg_map`(기본), `Budapest_map`, `my_map`(실차 트랙)

### 3-3. RViz에서 조작하기

RViz 창 위쪽 버튼을 씁니다.

| 하고 싶은 것 | 방법 |
|---|---|
| **정적 장애물 놓기** | `Publish Point` 클릭 → 트랙 위 클릭 (차가 알아서 회피) |
| **장애물 제거** | 그 장애물 근처를 다시 `Publish Point`로 클릭 |
| **상대 차 스폰** | `2D Goal Pose` → 트랙 위 원하는 위치 클릭 |
| **내 차 위치 옮기기** | `2D Pose Estimate` → 위치 클릭 |
| **각종 제어 버튼** | 왼쪽 `Panels` → `PitwallPanel` (자율주행 ON/OFF, 상대 속도 등) |

초록 반투명 선 = 추월 섹터(직선 구간에서만 공격적으로 추월).

### 3-4. 상태 확인

새 터미널(source 후)에서:

```bash
ros2 topic echo /integration/status
```

이런 한 줄이 1초마다 나옵니다:

```
scan:OK odom:OK perception:OK planning:OK control:OK | obs=2 wpnts=80 v=4.00 cmd=4.00
```

- `OK` = 정상, `STALE` = 끊김, `WAIT` = 아직 신호 안 옴
- `obs` = 인지된 장애물 수, `wpnts` = 경로 점 수, `v` = 현재 속도, `cmd` = 명령 속도

**하나라도 STALE/WAIT면 그 단계가 문제**입니다 (아래 6번 참고).

---

## 4. 실차로 실행하기

> ⚠️ **먼저 읽을 것 — 2026-07-21 hardware_ws 재구성(950b3b5)으로 끊겼던 자율
> 명령 배선은 2026-07-23에 복구됐지만, 비상정지는 아직 빠져 있습니다.**
> - ✅ 자율 명령: `ackermann_mux`의 `navigation` 입력이
>   `/autonomy/ackermann_cmd`(control_pkg 발행)를 구독하도록 `mux.yaml`에서
>   복구. mux 출력은 launch의 `ackermann_drive_out:=ackermann_cmd` remap으로
>   `ackermann_to_vesc_node`에 연결됩니다.
> - ✅ 자율/수동 토글: `joy_toggle_teleop`이 버튼 5로 래치 전환. 수동일 땐
>   `/teleop`(priority 100)이 자율(priority 10)을 가리고, 자율로 넘기면 0속도
>   버스트 뒤 침묵해서 mux가 자율 명령으로 폴스루합니다.
> - ❌ **비상정지는 아직 배선 안 됨.** `estop_node.cpp`는 남아 있지만
>   `bringup_launch.py`가 띄우지 않고 `mux.yaml`에 `locks` 항목도 없습니다.
>   `/ackermann_cmd_mux/mode`도 없습니다. **아래 "실차 안전장치" 절의 버튼 4(LB)
>   설명은 현재 유효하지 않으니, 정지는 버튼 5로 수동 전환하거나 차단기를
>   쓰세요.**

실차는 **A(하드웨어) → B(위치추정) → C(자율스택)** 순서로 띄웁니다.

### 4-A. 한 줄로 실행 (권장)

터미널 하나에서:

```bash
ros2 launch integration_pkg real_car.launch.py
```

이 한 줄이 A→B→C를 **자동 순서로** 처리합니다:

1. 하드웨어 bringup (LiDAR/VESC/joy/mux/TF)
2. 5초 뒤 particle_filter + **RViz(pf.rviz) 자동 실행**
3. `wait_for_localization` 게이트가 파티클 수렴을 감시하며 1초마다 상태 로그
4. **수렴이 확인되면 자율 스택(perception+planning+control)을 자동 시작**

사람이 할 일은 **RViz에서 `2D Pose Estimate` 한 번 찍는 것뿐**입니다
(방법은 아래 4-1의 "초기 위치 찍는 순서" 참고).

게이트 로그로 어디서 막혔는지 바로 알 수 있습니다:

```
대기 중: /scan 없음 — 하드웨어(LiDAR)가 안 떴습니다
대기 중: map -> base_link TF 없음 — PF가 아직 TF를 안 냅니다
대기 중: 파티클이 아직 흩어져 있음 (spread=3.42m > 0.35m) — RViz에서 2D Pose Estimate로 초기 위치를 찍으세요
수렴 확인 중 (spread=0.12m, 6/10)
수렴 완료 (파티클 spread=0.11m) — 자율 스택을 시작합니다.
```

옵션:

```bash
# 이미 하드웨어가 떠 있을 때 (B부터)
ros2 launch integration_pkg real_car.launch.py hardware:=false

# 화면 없이 / 자율 스택은 수동으로
ros2 launch integration_pkg real_car.launch.py rviz:=false autostart:=false

# 2분 안에 수렴 못 하면 자율 스택을 아예 띄우지 않음
ros2 launch integration_pkg real_car.launch.py wait_timeout:=120.0
```

### 4-B. 터미널 3개로 따로 실행 (디버깅용)

**순서가 매우 중요합니다.** C(자율스택)는 B의 파티클 필터가 실제로
`/pf/pose/odom`과 `map` TF를 내보내기 시작한 **뒤에** 띄워야 합니다.
안 그러면 C의 노드들이 아래처럼 무한 대기에 빠집니다(이게 가장 흔한 실수):

```
[frenet_frame_node]   Waiting for odom (real)...          ← /pf/pose/odom 이 없음
[local_trajectory_node] RUN: has_global=true, has_frenet=false
[obstacle_detector_node] TF lookup laser -> map failed: "map" ... does not exist
```

> 위 로그가 계속 반복되면 **B가 안 떴거나, 초기 포즈를 안 줘서 파티클 필터가
> 아직 map TF/odom을 안 내보내는 상태**입니다. C를 껐다 B부터 다시 확인하세요.

### 4-0. 터미널 A — 하드웨어 bringup

```bash
# LiDAR, VESC, odom, TF, mux, 조이스틱
# (2026-07-21 재구성으로 파일명이 bringup_launch.py로 바뀌었고 env/sensor_interface
#  인자는 없어졌습니다. LiDAR IP 등은 f1tenth_stack/config/sensors.yaml에서 설정.)
ros2 launch f1tenth_stack bringup_launch.py
```

`/scan`, `/odom`이 나오는지 확인:

```bash
ros2 topic hz /scan     # LiDAR 스캔이 나와야 함
ros2 topic hz /odom     # VESC odom 이 나와야 함
```

### 4-1. 터미널 B — localization (파티클 필터)

```bash
# 파티클 필터 (map→odom TF + /pf/pose/odom 발행)
ros2 launch localization_bringup localization.launch.py backend:=particle_filter
#   ⚠️ backend:=particle_filter 를 반드시 명시! (기본값은 slam_toolbox)
#   ⚠️ 현재 my_map 기준으로 설정됨 (localize.yaml map: my_map,
#      particle_filter/maps/my_map.{yaml,png}). 다른 맵이면 함께 교체.
```

#### 초기 위치 잡기 (필수) — 파티클 필터 초기화

파티클 필터는 켜지는 순간 파티클(차 위치 후보 3000개)을 **맵 전체에 흩뿌린
상태**로 시작합니다(로그 `GLOBAL INITIALIZATION`). 즉 차가 어디 있는지 아직
모릅니다. 이 상태에서 C를 띄우면 엉터리 위치로 차가 튑니다. **내가 실제
위치를 찍어줘야** 파티클이 모이고 LiDAR 스캔 매칭으로 수렴합니다.

> ⚠️ 초기 위치를 찍으려면 **터미널 A(하드웨어)가 먼저 떠 있어야** 합니다.
> laser→base_link TF가 없으면 클릭이 무시되고
> `Cannot set pose yet, waiting for laser transform` 로그만 뜹니다.

**RViz는 localization launch가 자동으로 띄웁니다** (`rviz:=false`로 끌 수 있음).
따로 띄우려면 — 반드시 **설치된** 설정 파일을 쓰세요:

```bash
rviz2 -d $(ros2 pkg prefix --share particle_filter)/rviz/pf.rviz
```

**초기 위치 찍는 순서:**

1. RViz에 맵 + 흩어진 파티클 구름(빨간 화살표들)이 보임
2. **실제 차를 맵상 알기 쉬운 지점**(보통 출발선 = 레이스라인 시작점)에
   물리적으로 갖다 놓기
3. 상단 툴바 **`2D Pose Estimate`** 클릭
4. 맵에서 **차가 서 있는 그 위치**를 클릭 → **차 앞 방향으로 드래그** 후 놓기
   (화살표 = 위치 + 헤딩)
5. 파티클이 그 지점으로 확 모임 → 차를 손으로 살짝 밀거나 천천히 전진시키면
   한 점으로 수렴 (`/pf/viz/particles` 구름이 작게 뭉치고
   `/pf/viz/inferred_pose`가 차 위에 얹힘)

| RViz 도구 | 용도 |
|---|---|
| **2D Pose Estimate** (`/initialpose`) | 초기 위치 지정 (정상 방법) |
| **Publish Point** (`/clicked_point`) | 위치를 완전히 잃었을 때 전체 재분산(리셋) |

> RViz 없이 명령줄로도 가능:
> ```bash
> ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
>   '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0},
>   orientation: {z: 0.0, w: 1.0}}}}'
> ```
> (x·y = 맵 좌표, orientation = 헤딩 쿼터니언)

**C를 띄우기 전에 반드시** `/pf/pose/odom`과 `map` TF가 수렴된 값으로 나오는지 확인:

```bash
ros2 topic hz /pf/pose/odom               # 값이 나와야 함
ros2 run tf2_ros tf2_echo map base_link   # map→base_link TF가 실제 차 위치와 맞아야 함
```

이 둘이 나오지 않으면(또는 값이 엉뚱하면) C를 띄워봐야 위의 무한 대기 로그만
반복됩니다. 위치를 잃으면 `2D Pose Estimate`로 다시 찍으면 됩니다.

### 4-2. 터미널 C — 우리 스택 (perception + planning + control + 모니터)

`/pf/pose/odom`·map TF 확인이 끝난 뒤에만 실행:

```bash
#   ※ 실차는 sim_ws source 하지 말 것
#   ※ 실차 planner는 map:= 인자를 무시하고 planning_pkg/config/params.yaml의
#     trajectory/overtaking/centerline CSV 기본값을 읽는다 (현재 my_map 기준).
#     다른 맵이면 params.yaml의 세 CSV 파일명을 교체 후 colcon build.
ros2 launch integration_pkg stack.launch.py env:=real_car
```

정상이면 `frenet_frame_node`가 더는 "Waiting for odom"을 찍지 않고
`[Frenet] XY(...) => s:.. d:..` 로그로 바뀝니다. 최종 확인은
`ros2 topic echo /integration/status` (3-4 참고).

### 4-1. 조이스틱으로 조종하기

하드웨어 bringup(터미널 A)이 뜨면 조이스틱이 바로 물립니다. 아래 매핑은
`hardware_ws/src/f1tenth_stack/config/params.yaml` 기준이며, **버튼/축 번호는
Xbox 계열 기준**입니다(패드가 다르면 번호가 다를 수 있음 —
`ros2 topic echo /joy`로 확인).

| 조작 | 입력 | 동작 |
|---|---|---|
| **전진/후진 속도** | 축 `1` (왼쪽 스틱 상하) | 앞으로 밀면 전진, 최고 1.5 m/s (후진 버튼과 함께면 후진 0.8 m/s) |
| **조향** | 축 `3` (오른쪽 스틱 좌우) | 좌/우 조향 (±0.4189 rad) |
| **후진** | 버튼 `0` (A) | 누른 채로 있으면 속도축이 후진으로 |
| **자율 ↔ 수동 토글** | 버튼 `5` (RB) | 누를 때마다 전환. 시작은 수동 |
| **비상 정지 토글** | 버튼 `4` (LB) | 누르면 정지 래치, 다시 누르면 해제 |

- **수동 명령은 `/manual_ackermann_cmd`** 로 나가고, `ackermann_cmd_mux`가
  자율 명령(`/autonomy/ackermann_cmd`)과 섞어 최종 `/ackermann_cmd`를 냅니다.
- **부드러운 가속**: teleop이 `max_accel`(기본 3.0 m/s²)로 속도를 램핑해서 VESC
  오픈루프에서 급출발을 막습니다. 스틱을 확 밀어도 서서히 붙습니다.
- **현재 모드 확인**: `ros2 topic echo /ackermann_cmd_mux/mode`
  → `manual` / `autonomous` / `estop` / `idle` 중 하나가 나옵니다.
- 스틱을 놓거나 조이스틱이 0.5초(`joy_timeout`) 끊기면 자동으로 속도 0.

### 실차 안전장치

- **자율/수동 전환은 조이스틱 버튼 5(RB)** 입니다.
  - control_pkg는 `autonomous_on_start: true`로 자율 명령(`/autonomy/ackermann_cmd`)을
    계속 내보내지만, **mux는 수동 모드로 시작**하므로 처음엔 바퀴로 안 나갑니다.
  - **버튼 5를 누르는 순간마다 수동 ↔ 자율 토글**(엣지 트리거)됩니다.
  - 각 명령이 0.5초 넘게 끊기면(`command_timeout`) 그 소스는 무시하고 정지합니다.
- **비상 정지는 버튼 4(LB) 토글**입니다 (`latch_estop: false`). 누르면
  `estop_node`가 `/safety/emergency_stop`을 true로 래치 → mux가 어느 모드든
  **무조건 정지 명령으로 덮어씀**(최우선). **다시 누르면 해제**됩니다.
  (이전의 "누르고 있는 동안만 정지" 방식에서 토글로 바뀌었습니다.)
- 실차 속도 상한은 안전상 제한돼 있습니다
  (`control_pkg/config/environments/real_car.yaml`의 `vehicle/max_speed_mps`)

---

## 5. 부분만 실행하기 (디버깅용)

각 단계를 따로 켜서 문제를 좁힐 수 있습니다. `env`는 `sim`/`real_car`/`race_map`
중 하나.

```bash
# perception만
ros2 launch perception_pkg perception.launch.py env:=sim

# planning만 (gym 없이 planner 노드들만)
ros2 launch planning_pkg planner_stack.launch.py            # sim 기본
ros2 launch planning_pkg planner_stack.launch.py env:=real_car

# control만
ros2 launch control_pkg control.launch.py env:=sim
```

> 시뮬 전체(gym 포함)를 planning 쪽에서 직접 켜고 싶으면:
> `ros2 launch planning_pkg sim_race.launch.py` (integration 없이 planning 단독 데모)

---

## 6. 문제가 생기면 (자주 겪는 것들)

**`Package 'xxx_pkg' not found`**
→ 그 워크스페이스를 source 안 했거나 빌드 안 됨. 2번 source 순서 다시 확인.

**`/integration/status`에서 `perception:STALE` 또는 장애물이 안 잡힘**
→ TF 문제일 수 있음. 실차면 localization(map→odom TF)이 떠 있는지 확인.
→ 시뮬이면 gym이 완전히 뜰 때까지 10초쯤 기다렸는지 확인.

**차가 벽을 따라 후진하거나 이상하게 감**
→ frenet 좌표가 깨진 것. `2D Pose Estimate`로 차를 레이스라인 위에 다시 놓기.

**RViz를 띄웠는데 맵/파티클이 하나도 안 보임**
→ 예전 `pf.rviz`는 **ROS1 rviz 포맷**이라 RViz2에서 디스플레이가 전부 사라졌습니다.
  현재는 RViz2 포맷으로 교체돼 있으니, 소스가 아니라 **설치된** 설정을 쓰세요:
  `rviz2 -d $(ros2 pkg prefix --share particle_filter)/rviz/pf.rviz`
  (`localization.launch.py`가 자동으로 띄워주므로 보통 직접 실행할 일은 없습니다.)

**코드를 고쳤는데 실행하면 예전 동작 그대로**
→ `perception_ws`에서 `--base-paths src` 없이 빌드했을 가능성이 큽니다 (1-A 주의사항).
  `colcon list`로 `perception_pkg`가 잡히는지 먼저 확인하세요.

**종료할 때 `obstacle_detector_node`에 `Exception in thread Thread-1`**
→ tf2 TransformListener의 내부 스레드가 남기는 로그로, 프로세스는 정상 종료(exit 0)합니다.
  `process has died [exit code 1]`이 함께 뜨면 그건 다른 문제입니다.

**빌드 에러 (`ae_hyu_msgs`, 심링크 관련)**
→ `--symlink-install` 썼거나 옛 캐시 남음. 해당 ws에서
`rm -rf build install log && colcon build`.

**시뮬 재실행 시 이전 노드가 안 죽어서 꼬임**
→ 프로세스 정리 후 재시작:
```bash
pkill -f "gym_bridge|sim_race|stack.launch"
```

---

## 7. 요약 (치트시트)

```bash
# ── 최초 1회 빌드 (순서 중요, planning 먼저) ──
source /opt/ros/humble/setup.zsh
# 공통(시뮬·실차): planning perception control integration
# 실차면 뒤에 hardware localization 도 추가
for ws in planning perception control integration; do   # 실차: ... integration hardware localization
  cd ~/IFAC_ws/${ws}_ws && colcon build --base-paths src && source install/setup.zsh
done
# ⚠️ --base-paths src 빼면 perception_pkg가 조용히 안 빌드됨 (1-A 주의사항)

# ── 매번 실행 ──
# 시뮬 (source 시뮬용 블록 후, 터미널 1개):
ros2 launch integration_pkg stack.launch.py

# 실차 (source 실차용 블록 후, 터미널 1개):
ros2 launch integration_pkg real_car.launch.py
#   → RViz가 자동으로 뜸. "2D Pose Estimate"로 실제 차 위치 클릭+드래그.
#   → 파티클이 수렴하면 자율 스택이 자동 시작됨.
# ⚠️ 자율 명령 배선이 950b3b5 이후 끊겨 있음 — 4번 맨 위 경고 확인

# 실차를 단계별로 나눠 띄우려면 (디버깅용, 터미널 3개 — A→B→C 순서 엄수):
ros2 launch f1tenth_stack bringup_launch.py                                             # A 하드웨어
ros2 launch localization_bringup localization.launch.py backend:=particle_filter        # B 위치추정(+RViz)
ros2 launch integration_pkg stack.launch.py env:=real_car                               # C 자율스택
# ⚠️ C에서 "Waiting for odom (real)" / "map ... does not exist" 반복 = B 미수렴 (4번 참고)
```
