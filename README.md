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
| `localization_ws` | 위치 추정 (particle_filter / slam_toolbox / AMCL) | `localization_bringup`, `particle_filter_cpp` |
| `perception_ws` | LiDAR 장애물 탐지 + 마커 시각화 | `perception_pkg` |
| `planning_ws` | 레이스라인 추종 + 회피 경로 + 모니터/RViz 패널 + **메시지 정의** | `planning_pkg`, `monitor_pkg`, `pitwall_panel`, `f1tenth_planning_msgs`, `f1tenth_perception_msgs` |
| `control_ws` | pure pursuit 제어 (C++) | `control_pkg` |
| `integration_ws` | 전체 스택을 한 번에 기동 + 헬스 모니터 + **매핑** | `integration_pkg` |

워크스페이스는 아니지만 `maps/`는 **매핑으로 만든 맵이 저장되는 공용 폴더**입니다
(`mapping.launch.py`의 기본 저장 위치이자 `map:=`가 가장 먼저 찾는 곳).

실차는 여기에 **명령 mux**가 붙습니다 (`ackermann_mux` + `f1tenth_stack`의 `joy_toggle_teleop`).

---

## 빌드

루트의 스크립트 두 개가 전부입니다. **직접 `colcon build`를 치지 마세요** —
워크스페이스 순서와 `--base-paths src`를 스크립트가 챙깁니다.

```bash
cd ~/IFAC_ws
./setup_deps.sh          # ① 머신당 1회 — 시스템 의존성
./build_all.sh           # ② 코드 고칠 때마다 — 빌드
```

시뮬 전용 머신이면 두 스크립트 모두 `sim` 인자를 주세요 (`hardware_ws`·
`localization_ws`와 `range_libc`를 건너뜁니다).

```bash
./setup_deps.sh sim && ./build_all.sh sim
```

### `setup_deps.sh` — 머신당 1회

- `rosdep install`을 **모든 워크스페이스 `src`에 한 번에** 실행합니다.
  한 호출로 넘겨야 크로스-워크스페이스 소스 의존(`f1tenth_planning_msgs` 등)을
  "로컬 소스"로 인식해 헛되이 apt에서 찾지 않습니다.
  `laser_proc`(urg_node)·`serial_driver`(vesc_driver)·`tf_transformations`·
  `transforms3d` 등이 여기서 깔립니다.
- **`range_libc`** 는 rosdep 밖의 유일한 수동 의존입니다 (실차만). `~/range_libc`
  소스에서 빌드하므로 없으면 먼저 클론하세요:
  `git clone https://github.com/f1tenth/range_libc ~/range_libc`

### `build_all.sh` — 매번

- 빌드 순서: `planning`(메시지 패키지가 여기 있어 항상 먼저) → `perception` →
  `control` → `integration` → (실차) `hardware` → `localization`.
- 각 워크스페이스를 `colcon build --base-paths src`로 빌드하고 곧바로 source 합니다.
- 의존성이 없으면 빌드를 낭비하지 않고 멈춰 `./setup_deps.sh`를 안내합니다.
- 끝에 **실행용 source 블록**을 출력하므로 그대로 복사해 쓰면 됩니다.

> ⚠️ 주의
> - **두 스크립트는 zsh 전용입니다** (`#!/usr/bin/env zsh`). `bash setup_deps.sh`처럼
>   호출하지 말고 `./setup_deps.sh`로 실행하세요.
> - **`numpy`는 apt 1.x여야 합니다.** `~/.local`에 pip numpy 2.x가 깔리면
>   scipy/ROS Python 확장이 ABI로 깨집니다. 두 스크립트 모두 시작할 때 이걸
>   검사하고 오염됐으면 중단합니다
>   (고치기: `/usr/bin/python3 -m pip uninstall -y numpy`).
> - **`git pull`/소스 수정 후에는 반드시 다시 `./build_all.sh`.** 실행은 `install/`을
>   쓰므로, 빌드 안 하면 옛 맵·옛 파라미터로 주행합니다. (실제로 맵을 `my_track`으로
>   바꾼 뒤 재빌드를 안 해서 `Spielberg_map_centerline.csv` 로드 실패로 이어진 적이 있음.)
> - **`--base-paths src`가 빠지면 `perception_pkg`가 조용히 안 빌드됩니다.**
>   워크스페이스 루트에 남은 잔재를 colcon이 패키지로 오인해 `src/`로 내려가지
>   않습니다. `build_all.sh`는 항상 이 옵션을 씁니다. 수동 빌드했다면 `colcon list`로
>   `perception_pkg`가 잡히는지 확인하세요.
> - `--symlink-install` 쓰지 마세요 (심링크/캐시 충돌 에러 유발).
> - 빌드가 꼬이면 해당 ws에서 `rm -rf build install log` 후 다시 `./build_all.sh`.
> - 다른 PC에서 복사해 와 `CMakeCache.txt directory is different` 에러가 나면
>   `colcon build --cmake-clean-cache`를 한 번 실행하세요.
> - **시뮬(`sim_ws`)은 두 스크립트의 대상이 아닙니다** — 별도로 빌드해야 하고,
>   경로는 레포 안의 `~/IFAC_ws/sim_ws`입니다(`~/sim_ws`가 아님). 현재
>   `sim_ws/src/f1tenth_gym_ros`에 `.py` 소스가 없고 `install/` 산출물만 남아
>   있으니, `install/`을 지우기 전에 소스를 복구해 두세요 (`sim_ws`는 git 미추적).

---

## 실행 요약

> 단계별 절차·RViz 조작·트러블슈팅은 **[HOW_TO_RUN.md](HOW_TO_RUN.md)** 에 있습니다.
> 여기서는 **어떤 launch가 무엇을 켜는지**만 요약합니다. 실행 전 각 터미널에서
> 필요한 워크스페이스를 먼저 source 하세요 (HOW_TO_RUN §2).

| 목적 | 명령 |
|---|---|
| **시뮬 전체** (gym+perception+planning+control+RViz) | `ros2 launch integration_pkg stack.launch.py` |
| **실차 원커맨드** (하드웨어→PF+RViz→initialpose/quality 게이트→자율 스택) | `ros2 launch integration_pkg real_car.launch.py` |
| **매핑** (하드웨어+SLAM+RViz+저장) | `ros2 launch integration_pkg mapping.launch.py map_name:=<이름>` |

원커맨드로 실차를 띄우면 사람이 할 일은 **RViz에서 `2D Pose Estimate` 한 번**
찍는 것뿐입니다. PF가 `/initialpose` 적용과 scan-map quality 연속 OK를 확인하면
자율 스택이 자동으로 시작됩니다.

### 실차를 터미널 3개로 나눠 띄우기 (디버깅용)

**반드시 A → B → C 순서.** C는 B가 `/pf/pose/odom`과 map TF를 내보낸 뒤에 띄워야 합니다.

| 터미널 | 역할 | 명령 |
|---|---|---|
| **A** | 하드웨어 (LiDAR·VESC·odom·TF·joy·mux) | `ros2 launch f1tenth_stack bringup_launch.py` |
| **B** | 위치추정 (PF, map→odom TF + RViz) | `ros2 launch localization_bringup localization.launch.py backend:=particle_filter` |
| **C** | 자율 스택 (perception+planning+control+모니터) | `ros2 launch integration_pkg stack.launch.py env:=real_car` |

> B는 `backend:=particle_filter`를 **반드시** 명시하세요 (기본값은 slam_toolbox).

### 매핑

조이스틱으로 트랙을 한 바퀴 돌고 **`Ctrl+C`** 하면 그 시점의 맵이
`~/IFAC_ws/maps/<이름>.{pgm,yaml}` (nav2 `map_server` 표준 PGM+YAML)로 저장됩니다.
주행 중 중간 저장은 `ros2 service call /save_map std_srvs/srv/Trigger`.
주요 인자: `map_name`(저장 이름), `map_dir`(기본 `~/IFAC_ws/maps`),
`hardware:=false`(이미 bringup이 떠 있을 때), `rviz:=false`, `save_on_exit:=false`.

### 맵 바꾸기 — `map:=<이름>`

`map:=` 하나로 **PF 맵 + planner CSV 3종**이 함께 전환됩니다. 어느 실차 launch에든 붙입니다.

```bash
ros2 launch integration_pkg real_car.launch.py map:=my_track2          # 실차 원커맨드
ros2 launch integration_pkg stack.launch.py env:=real_car map:=my_track2
```

- **PF 맵** — `~/IFAC_ws/maps/<이름>.yaml` → `particle_filter_cpp/maps/<이름>.yaml` 순으로
  찾습니다. 매핑으로 만든 맵은 **재빌드 없이** 바로 쓸 수 있습니다.
- **planner CSV** — `<이름>_iqp_*.csv` 등을 glob으로 찾습니다(파일명에 속도·마진·mu가
  붙어 이름만으로 유도 불가). `planning_pkg/config/` 안에 있어야 하므로 새로 뽑았다면
  **재빌드가 필요**합니다.
- 맵 파일이 없으면 **launch 단계에서** 찾아본 폴더를 전부 보여주며 멈춥니다.
- 개별 override(`map_yaml_path:=`, `trajectory_csv_file:=`, `overtaking_csv_file:=`,
  `centerline_csv_file:=`)가 `map:=`보다 우선합니다.
- `map:=`를 **안 주면** `localize.yaml` / `params.yaml`의 기본값을 씁니다.

**새 맵을 만들어 주행하기까지의 전체 순서:**

```bash
# 1) 매핑 → ~/IFAC_ws/maps/my_track2.{pgm,yaml}
ros2 launch integration_pkg mapping.launch.py map_name:=my_track2

# 2) 레이스라인 CSV 생성 (.venv python, 주행 방향 명시 권장)
cd ~/IFAC_ws/planning_ws/src/planning_pkg/scripts/global_traj_sim
~/IFAC_ws/.venv/bin/python build_map.py my_track2 --dir ccw
#    맵은 ~/IFAC_ws/maps 를 자동으로 찾음 (복사 불필요).
#    속도·마찰·마진은 build_map.py 상단 '설정' 블록(V_MAX/MU/MARGIN/SAFE_DIST)에서 조정.

# 3) CSV 반영 재빌드 (맵 자체는 재빌드 불필요)
cd ~/IFAC_ws && ./build_all.sh

# 4) 주행
ros2 launch integration_pkg real_car.launch.py map:=my_track2
```

---

## 실차 안전장치

- **localization 출발 게이트**는 `/initialpose`가 실제 적용되고 scan-map quality가
  5회 연속 OK가 되기 전에는 자율 스택을 시작하지 않습니다. 실차 control도
  `/pf/localization_ready`가 false 또는 0.5초 stale이면 0 명령만 냅니다. 이 게이트는
  autonomy 입력에만 적용되므로 mux의 고우선순위 joystick/estop은 계속 동작합니다.
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
- 실차 설정의 현재 속도 상한은 **3.6 m/s**입니다
  (`control_ws/src/control_pkg/config/environments/real_car.yaml`의
  `vehicle/max_speed_mps`). 플래너는 6 m/s raceline에 `velocity_scale: 0.6`을
  적용해 같은 상한으로 맞춥니다.

---

## 주요 파라미터 위치

| 대상 | 파일 |
|---|---|
| LiDAR (Hokuyo IP·FOV) | `hardware_ws/src/f1tenth_stack/config/sensors.yaml` |
| VESC (조향/속도 캘리브레이션) | `hardware_ws/src/f1tenth_stack/config/vesc.yaml` — **hardware_ws 전체의 single source of truth. launch 인자로 덮어쓰지 말 것** |
| 명령 mux (우선순위·타임아웃) | `hardware_ws/src/f1tenth_stack/config/mux.yaml` |
| 조이스틱 (버튼·축·상한) | `hardware_ws/src/f1tenth_stack/config/joy_teleop.yaml` |
| control 공통 / 환경별 | `control_ws/src/control_pkg/config/params.yaml`, `config/environments/*.yaml` |
| planning 공통 / 환경별 | `planning_ws/src/planning_pkg/config/params.yaml`, `config/environments/*.yaml` |
| perception 공통 / 환경별 | `perception_ws/src/perception_pkg/config/params.yaml`, `config/environments/*.yaml` |
| integration 환경별 | `integration_ws/src/integration_pkg/config/environments/*.yaml` |
| localization | `localization_ws/src/localization_bringup/config/localization/*.yaml` |
| particle_filter_cpp 맵 (기본값) | `localization_ws/src/particle_filter_cpp/config/localize.yaml` — `map:=`를 안 줬을 때만 쓰입니다. 당일 맵은 `map:=<이름>`으로 지정하는 편이 낫습니다 |
| planner raceline CSV (기본값) | `planning_ws/src/planning_pkg/config/params.yaml` — 마찬가지로 `map:=`가 없을 때만 |
| 매핑으로 만든 맵 | `maps/` (`map:=`가 가장 먼저 찾는 곳) |

`env` 값은 `sim` / `real_car` / `race_map` / `mapping` 중 하나이며, 각 워크스페이스가
같은 이름의 환경 yaml을 읽어 토픽·속도 등을 바꿉니다.

---

## Notes

- ROS 2 Humble 기준. `zsh` 터미널에서는 `setup.zsh`를 source 하세요
  (`setup.bash`를 쓰면 `local_setup.bash not found` 에러가 날 수 있음).
- `f1tenth_stack`은 `f1tenth/f1tenth_system` 구조를 참고해 이 레포에 맞게 재구성한 것입니다.
- 조향/속도 캘리브레이션의 단일 기준 문서는 **[TUNING_STATUS.md](TUNING_STATUS.md)** 입니다.
  캘리브레이션 관련 작업 전에 먼저 읽으세요 (`hardware_ws/tools/`의 계측 스크립트 포함).
- 확장 체크리스트: [F1TENTH_CHECKLIST.md](F1TENTH_CHECKLIST.md),
  진행 항목: [PITWALL_TODO.md](PITWALL_TODO.md).
