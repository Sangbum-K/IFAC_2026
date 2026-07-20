# IFAC_ws

ROS 2 **Humble** 기반 F1TENTH 자율주행 스택입니다. LiDAR로 장애물을 찾아
레이스라인을 추종하며 회피 경로를 만들고(perception → planning), 이를 조향/속도
명령으로 바꿔(control) 시뮬레이터와 실차에서 동일한 토폴로지로 주행합니다.

> 실행 절차를 처음부터 따라 하려면 **[HOW_TO_RUN.md](HOW_TO_RUN.md)** 를 보세요.
> 이 README는 다른 PC에서 이 레포를 참조할 때 필요한 **구조·빌드·실차 배선**을
> 한눈에 보는 문서입니다.

---

## 워크스페이스 구성

이 레포는 역할별로 분리된 여러 colcon 워크스페이스로 이뤄져 있습니다.

| 워크스페이스 | 역할 | 핵심 패키지 |
|---|---|---|
| `hardware_ws` | 실차 하드웨어 bringup (joy, VESC, LiDAR, odom, TF, estop, cmd mux) | `f1tenth_stack`, `vehicle_bringup`, `vehicle_teleop`, `vesc`, `urg_node` |
| `localization_ws` | 위치 추정 (particle_filter / slam_toolbox / AMCL) | `localization_bringup`, `particle_filter` |
| `perception_ws` | LiDAR 장애물 탐지 | `perception_pkg` |
| `planning_ws` | 레이스라인 추종 + 회피 경로 + 모니터/RViz 패널 + **메시지 정의** | `planning_pkg`, `monitor_pkg`, `pitwall_panel`, `f1tenth_planning_msgs`, `f1tenth_perception_msgs` |
| `control_ws` | pure pursuit 제어 (C++) | `control_pkg` |
| `integration_ws` | 전체 스택을 한 번에 기동 + 헬스 모니터 | `integration_pkg` |

> `unicorn-racing-stack-jazzy/` 는 별도 참조용(ROS 2 Jazzy + conda) 스택으로, 위
> Humble 스택과 독립입니다. 이 README/HOW_TO_RUN의 절차와 무관하니 혼동하지 마세요.

메시지 패키지(`f1tenth_planning_msgs`, `f1tenth_perception_msgs`)가
`planning_ws`에 있어서, **planning_ws를 항상 가장 먼저 빌드**해야 나머지가
그 메시지를 참조할 수 있습니다.

---

## 데이터 흐름 (실차·시뮬 공통)

```
LiDAR /scan
  → [perception] 장애물 탐지        → /perception/obstacles
  → [planning]  frenet 변환·회피 경로 → /local_waypoints
  → [control]   pure pursuit         → /autonomy/ackermann_cmd (실차) / /drive (시뮬)
```

실차는 여기에 **명령 mux**가 붙습니다 (`vehicle_teleop/ackermann_cmd_mux`):

```
조이스틱 수동 → /manual_ackermann_cmd ┐
자율주행     → /autonomy/ackermann_cmd ┼→ [mux] → /ackermann_cmd → VESC → 차량
e-stop      → /safety/emergency_stop  ┘   (버튼5 토글, estop 최우선)
```

- **버튼 5**를 누르는 순간마다 수동 ↔ 자율 **토글**(엣지 트리거)
- `/safety/emergency_stop`(latched Bool)이 true면 mux가 무조건 정지 명령으로 덮어씀
- 각 소스가 `command_timeout`(0.5s) 넘게 끊기면 그 소스는 무시(idle 정지)

위치 추정은 `localization_ws`의 particle_filter가 `map→odom` TF와
`/pf/pose/odom`을 발행해 스택에 좌표계를 제공합니다.

---

## 빌드 (최초 1회 · 순서 중요)

경로는 각자 홈에 맞게 바꾸세요. 아래는 `~/IFAC_ws` 기준입니다.
`zsh`면 `.zsh`, `bash`면 `.bash`를 쓰세요.

```bash
source /opt/ros/humble/setup.zsh

# 1) planning 먼저 — 메시지 패키지가 여기 있음
cd ~/IFAC_ws/planning_ws && colcon build && source install/setup.zsh

# 2) 나머지 (실차면 hardware_ws / localization_ws 포함)
for ws in perception control integration hardware localization; do
  cd ~/IFAC_ws/${ws}_ws && colcon build && source install/setup.zsh
done
```

> ⚠️ 주의
> - `--symlink-install` 쓰지 마세요 (심링크/캐시 충돌 에러 유발).
> - 빌드가 꼬이면 해당 ws에서 `rm -rf build install log && colcon build`.
> - 다른 PC에서 복사해 와 `CMakeCache.txt directory is different` 에러가 나면
>   `colcon build --cmake-clean-cache`를 한 번 실행하세요.
> - 시뮬을 돌리려면 별도로 `~/sim_ws`(f1tenth_gym_ros)가 빌드돼 있어야 합니다.

---

## 실행 요약

자세한 단계·RViz 조작·트러블슈팅은 **[HOW_TO_RUN.md](HOW_TO_RUN.md)** 참고.

### 시뮬 (gym 통합)
```bash
ros2 launch integration_pkg stack.launch.py           # env:=sim 기본
```

### 실차 (터미널 3개, 각 터미널에서 필요한 ws를 source 후)
```bash
# A) 하드웨어 — LiDAR·VESC·odom·TF·estop·cmd mux
ros2 launch f1tenth_stack bringup.launch.py env:=real_car sensor_interface:=ethernet

# B) 위치추정 — 반드시 backend:=particle_filter 명시 (기본값은 slam_toolbox)
ros2 launch localization_bringup localization.launch.py backend:=particle_filter

# C) 자율주행 스택 — perception + planning + control + 모니터
ros2 launch integration_pkg stack.launch.py env:=real_car
```

### 매핑 (SLAM Toolbox)
```bash
ros2 launch f1tenth_stack mapping.launch.py env:=mapping sensor_interface:=ethernet
```

---

## 실차 안전장치

- **자율주행 토글은 조이스틱 버튼 5** (mux 엣지 트리거). 시작 시 control_pkg는
  `autonomous_on_start: true`로 자율 명령을 내보내지만, mux가 수동 모드로 시작하므로
  **버튼 5를 눌러야 실제로 자율 명령이 바퀴로 나갑니다.**
- **비상 정지**: `/safety/emergency_stop`(latched Bool)이 true면 즉시 정지, 최우선.
- 실차 속도 상한은 안전상 제한돼 있습니다
  (`control_ws/src/control_pkg/config/environments/real_car.yaml`의
  `vehicle/max_speed_mps`).

---

## 주요 파라미터 위치

| 대상 | 파일 |
|---|---|
| 하드웨어 공통 | `hardware_ws/src/vehicle_bringup/config/params.yaml` |
| 하드웨어 환경별 | `hardware_ws/src/vehicle_bringup/config/environments/{sim,real_car,race_map,mapping}.yaml` |
| control 환경별 | `control_ws/src/control_pkg/config/environments/*.yaml` |
| integration 환경별 | `integration_ws/src/integration_pkg/config/environments/*.yaml` |
| localization | `localization_ws/src/localization_bringup/config/localization/*.yaml` |
| particle_filter 맵 경로 | `localization_ws/src/particle_filter/config/localize.yaml` (당일 맵으로 교체) |

`env` 값은 `sim` / `real_car` / `race_map` / `mapping` 중 하나이며, 각 워크스페이스가
같은 이름의 환경 yaml을 읽어 토픽·속도 등을 바꿉니다.

---

## Notes

- ROS 2 Humble 기준. `zsh` 터미널에서는 `setup.zsh`를 source 하세요
  (`setup.bash`를 쓰면 `local_setup.bash not found` 에러가 날 수 있음).
- `f1tenth_stack`은 `f1tenth/f1tenth_system` 구조를 참고해 이 레포에 맞게 재구성한 것입니다.
- 확장 체크리스트: [F1TENTH_CHECKLIST.md](F1TENTH_CHECKLIST.md),
  진행 항목: [PITWALL_TODO.md](PITWALL_TODO.md).
- `control_ws/build,install,log`가 과거에 커밋돼 git에 추적 중입니다.
  `.gitignore`엔 이미 제외돼 있으니, 정리하려면
  `git rm -r --cached control_ws/build control_ws/install control_ws/log` 후 커밋하세요.
