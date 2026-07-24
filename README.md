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
| `hardware_ws` | 실차 하드웨어 bringup (joy, VESC, LiDAR, odom, TF, cmd mux) | `f1tenth_stack`, `ackermann_mux`, `vesc`, `urg_node` |
| `localization_ws` | 위치 추정 (particle_filter / slam_toolbox / AMCL) | `localization_bringup`, `particle_filter` |
| `perception_ws` | LiDAR 장애물 탐지 | `perception_pkg` |
| `planning_ws` | 레이스라인 추종 + 회피 경로 + 모니터/RViz 패널 + **메시지 정의** | `planning_pkg`, `monitor_pkg`, `pitwall_panel`, `f1tenth_planning_msgs`, `f1tenth_perception_msgs` |
| `control_ws` | pure pursuit 제어 (C++) | `control_pkg` |
| `integration_ws` | 전체 스택을 한 번에 기동 + 헬스 모니터 + **매핑** | `integration_pkg` |

워크스페이스는 아니지만 `maps/`는 **매핑으로 만든 맵이 저장되는 공용 폴더**입니다
(`mapping.launch.py`의 기본 저장 위치이자 `map:=`가 가장 먼저 찾는 곳).

가장 단순한 방법은 레포 루트에서 한 번에 빌드하는 것입니다.

```bash
source /opt/ros/humble/setup.zsh

cd ~/IFAC_ws
# 최초 1회: 시스템 의존성 설치 (특히 hardware_ws — 없으면 urg_node/vesc_driver 빌드 실패)
rosdep install --from-paths */src --ignore-src -r -y

colcon build
```

워크스페이스를 나눠서 빌드할 때는 `f1tenth_stack`가 `localization_bringup`를 참조하므로 아래 순서를 권장합니다.

```bash
source /opt/ros/humble/setup.zsh

cd ~/IFAC_ws/localization_ws
colcon build

source ~/IFAC_ws/localization_ws/install/setup.zsh

cd ~/IFAC_ws/hardware_ws
colcon build
```

실차는 여기에 **명령 mux**가 붙습니다 (`ackermann_mux` + `f1tenth_stack`의 `joy_toggle_teleop`).

루트에서 빌드했다면 아래 한 줄이면 충분합니다.

```bash
source /opt/ros/humble/setup.zsh
source ~/IFAC_ws/install/setup.zsh
```

워크스페이스를 나눠서 빌드했다면 실행 전에는 아래 순서로 source 합니다.

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
> - **`git pull`/소스 수정 후에는 반드시 다시 `colcon build`** 하세요. 실행은
>   `install/`을 쓰므로, 빌드 안 하면 옛 맵·옛 파라미터로 주행합니다. (실제로
>   맵을 `my_track`으로 바꾼 뒤 재빌드를 안 해서 `Spielberg_map_centerline.csv`
>   로드 실패로 이어진 적이 있음.)
> - `hardware_ws`는 시스템 의존성 `ros-humble-laser-proc`(urg_node)·
>   `ros-humble-serial-driver`(vesc_driver)가 없으면 빌드가 실패합니다. 위
>   `rosdep install`로 먼저 설치하세요.
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
# A) 하드웨어 — LiDAR·VESC·odom·TF·joy·cmd mux (인자 없음. LiDAR IP 등은 config/sensors.yaml)
ros2 launch f1tenth_stack bringup_launch.py

# B) 위치추정 — 반드시 backend:=particle_filter 명시 (기본값은 slam_toolbox). 맵은 my_track
ros2 launch localization_bringup localization.launch.py backend:=particle_filter

# C) 자율주행 스택 — perception + planning + control + 모니터
ros2 launch integration_pkg stack.launch.py env:=real_car
```

> 위 3개를 자동 순서로 묶은 원커맨드도 있습니다: `ros2 launch integration_pkg real_car.launch.py`
> (하드웨어→PF+RViz→파티클 수렴 게이트→자율 스택. 사람은 RViz에서 `2D Pose Estimate`만 찍으면 됨.)

### 매핑 (SLAM Toolbox)

한 줄로 **하드웨어 bringup + SLAM + RViz + 맵 저장**까지 띄웁니다.

```bash
ros2 launch integration_pkg mapping.launch.py map_name:=my_track2
```

조이스틱으로 트랙을 천천히 한 바퀴 돌고 **`Ctrl+C`** 하면 그 시점의 맵이
`~/IFAC_ws/maps/my_track2.{pgm,yaml}`로 저장됩니다.
주행 중 중간 저장은 `ros2 service call /save_map std_srvs/srv/Trigger`.

| 인자 | 기본값 | |
|---|---|---|
| `map_name` | `my_map` | 저장할 맵 이름 |
| `map_dir` | `~/IFAC_ws/maps` | 저장 디렉터리 |
| `hardware` | `true` | 이미 bringup이 떠 있으면 `false` |
| `rviz` | `true` | 맵이 채워지는 걸 보며 주행 |
| `save_on_exit` | `true` | `false`면 `/save_map` 서비스로만 저장 |

저장 형식은 nav2 `map_server` 표준 **PGM(P5) + YAML**이며, 같은 `/map`에 대해
`nav2 map_saver_cli`와 픽셀 바이트까지 동일합니다. yaml 끝에는 참고용으로
`size_px` / `size_m` / `bounds_x_m` / `bounds_y_m`가 붙습니다
(`map_server`는 모르는 키를 무시하므로 로드에 영향 없음).

> 맵 저장은 `/map`을 계속 캐시해 두고 종료 시 **파일로 직접** 씁니다.
> 종료 시점에 slam_toolbox로 서비스를 호출하는 방식은 launch가 SIGINT를
> 프로세스 그룹 전체에 동시에 보내 slam_toolbox가 이미 죽는 중이라 자주 실패합니다.

터미널을 나눠 띄우려면:
```bash
ros2 launch f1tenth_stack bringup_launch.py
ros2 launch localization_bringup localization.launch.py backend:=slam_toolbox mode:=mapping
```

### 맵 바꾸기

`map:=<이름>` 하나로 **PF 맵과 planner CSV 3종이 함께** 바뀝니다.

```bash
ros2 launch integration_pkg real_car.launch.py map:=my_track2   # 실차 원커맨드
ros2 launch integration_pkg stack.launch.py env:=real_car map:=my_track2
ros2 launch localization_bringup localization.launch.py backend:=particle_filter map:=my_track2
```

- **PF 맵** — `~/IFAC_ws/maps/<이름>.yaml` → `particle_filter/maps/<이름>.yaml` 순으로
  찾습니다. 매핑으로 만든 맵은 **재빌드 없이** 바로 쓸 수 있습니다.
- **planner CSV** — `<이름>_iqp_*.csv`, `<이름>_sp_*.csv`, `<이름>_centerline.csv`를
  glob으로 찾습니다(파일명에 속도·마진·mu가 붙어 이름만으로 유도 불가). 이건
  `planning_pkg/config/` 안에 있어야 하므로 **재빌드가 필요**합니다.
- `map:=`를 **안 주면** 종전대로 `localize.yaml` / `params.yaml` 값을 씁니다.
  `env:=sim`은 gym 맵 이름이라 기본값 `Spielberg_map` 그대로입니다.
- 맵 파일이 없으면 **launch 단계에서** 찾아본 폴더를 전부 보여주며 멈춥니다.
  (전에는 `map_server`가 조용히 죽고 PF가 영영 수렴하지 않아 원인 찾기가 어려웠음)

개별 override는 `map:=`보다 우선합니다 — `map_yaml_path:=<전체경로>`,
`trajectory_csv_file:=`, `overtaking_csv_file:=`, `centerline_csv_file:=`.

새 맵을 만들어 주행하기까지의 전체 순서:

```bash
# 1) 매핑 → ~/IFAC_ws/maps/my_track2.{pgm,yaml}
ros2 launch integration_pkg mapping.launch.py map_name:=my_track2

# 2) 레이스라인 CSV 3종 생성
cd ~/IFAC_ws/planning_ws/src/planning_pkg/scripts/global_traj_sim
python3 step4_main.py --map my_track2 --maps-dir ~/IFAC_ws/maps \
    --v-max 6.0 --mu 0.75 --margin 0.08

# 3) CSV 반영을 위해 재빌드 (맵 자체는 재빌드 불필요)
cd ~/IFAC_ws && ./build_all.sh

# 4) 주행 — localize.yaml/params.yaml은 손대지 않아도 됨
ros2 launch integration_pkg real_car.launch.py map:=my_track2
```

---

## 실차 안전장치

- **자율주행 토글은 조이스틱 버튼 5** (`joy_toggle_teleop`, 엣지 트리거). 시작 시
  control_pkg는 `autonomous_on_start: true`로 자율 명령(`/autonomy/ackermann_cmd`)을
  내보내지만, teleop이 **수동 모드로 시작**해 mux(joystick 우선순위)를 점유하므로
  **버튼 5를 눌러야** teleop이 침묵하고 자율 명령이 바퀴로 나갑니다.
- ⚠️ **비상 정지(E-stop)는 현재 배선돼 있지 않습니다.** control_node는
  `/safety/emergency_stop`(latched Bool)을 구독하지만 이 토픽을 **발행하는 노드가
  없고**(joy에 estop 버튼 없음, `estop_node`는 legacy `vehicle_bringup`에만 있어
  bringup에서 안 뜸), mux에도 lock이 없습니다. 즉 **소프트웨어 즉시정지는 없습니다.**
  정지 수단은 (1) 버튼 5로 수동 전환 후 스틱 중립, (2) 물리 kill 스위치/차단기,
  (3) `Ctrl-C`. 실차 주행 전 estop 배선 여부를 팀과 확정하세요.
- 실차 속도 상한은 안전상 **1.0 m/s**로 제한돼 있습니다
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
| particle_filter 맵 (기본값) | `localization_ws/src/particle_filter/config/localize.yaml` — `map:=`를 안 줬을 때만 쓰입니다. 당일 맵은 `map:=<이름>`으로 지정하는 편이 낫습니다 |
| planner raceline CSV (기본값) | `planning_ws/src/planning_pkg/config/params.yaml` — 마찬가지로 `map:=`가 없을 때만 |
| 매핑으로 만든 맵 | `maps/` (`map:=`가 가장 먼저 찾는 곳) |

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
