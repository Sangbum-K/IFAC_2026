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
for ws in planning perception control integration; do
  cd ~/IFAC_ws/${ws}_ws && colcon build && source install/setup.zsh
done
```

### 1-B. 실차에서만 추가로

```bash
# 하드웨어 + 위치추정 (시뮬에선 gym이 대신하므로 불필요)
for ws in hardware localization; do
  cd ~/IFAC_ws/${ws}_ws && colcon build && source install/setup.zsh
done
```

> ⚠️ **주의 3가지**
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
source ~/sim_ws/install/setup.zsh          # 시뮬 전용
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

사용 가능한 맵: `Spielberg_map`(기본), `Budapest_map`

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

실차는 **hardware / localization을 각 팀 launch로 먼저 띄운 뒤**, 이 스택을
`env:=real_car`로 실행합니다.

```bash
# (터미널 A) 하드웨어 bringup — LiDAR, VESC, odom, TF, estop
ros2 launch f1tenth_stack bringup.launch.py env:=real_car sensor_interface:=ethernet

# (터미널 B) localization — 파티클 필터 (map→odom TF + /pf/pose/odom 발행)
ros2 launch localization_bringup localization.launch.py backend:=particle_filter
#   ⚠️ backend:=particle_filter 를 반드시 명시! (기본값은 slam_toolbox)
#   ⚠️ 당일 맵으로 particle_filter/config/localize.yaml 의 map 경로도 교체

# (터미널 C) 우리 스택 — perception + planning + control + 모니터
#   ※ 실차는 sim_ws source 하지 말 것
ros2 launch integration_pkg stack.launch.py env:=real_car
```

### 실차 안전장치

- **자율/수동 전환은 조이스틱 버튼 5** 입니다. 하드웨어 bringup의
  `ackermann_cmd_mux` 노드가 명령을 섞어 최종 `/ackermann_cmd`를 냅니다.
  - control_pkg는 `autonomous_on_start: true`로 자율 명령(`/autonomy/ackermann_cmd`)을
    계속 내보내지만, **mux는 수동 모드로 시작**하므로 처음엔 바퀴로 안 나갑니다.
  - **버튼 5를 누르는 순간마다 수동 ↔ 자율 토글**(엣지 트리거)됩니다.
  - 각 명령이 0.5초 넘게 끊기면(`command_timeout`) 그 소스는 무시하고 정지합니다.
- **비상 정지**: `/safety/emergency_stop` (latched Bool) 이 true면 mux가 어느 모드든
  **무조건 정지 명령으로 덮어씀** (최우선).
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
  cd ~/IFAC_ws/${ws}_ws && colcon build && source install/setup.zsh
done

# ── 매번 실행 ──
# 시뮬 (source 시뮬용 블록 후, 터미널 1개):
ros2 launch integration_pkg stack.launch.py

# 실차 (source 실차용 블록 후, 터미널 3개):
ros2 launch f1tenth_stack bringup.launch.py env:=real_car sensor_interface:=ethernet   # A 하드웨어
ros2 launch localization_bringup localization.launch.py backend:=particle_filter        # B 위치추정
ros2 launch integration_pkg stack.launch.py env:=real_car                               # C 자율스택
# 주행: 조이스틱 버튼 5 = 자율 ON/OFF 토글, estop 버튼 = 즉시 정지
```
