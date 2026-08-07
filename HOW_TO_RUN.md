# IFAC_ws 실행 가이드 

F1TENTH 자율주행 전체 스택(perception → planning → control)을 **시뮬레이터**와
**실차**에서 실행하는 방법

## 주행명령어 alias버전입니다

> 🖥️ **어디서 치느냐 — 기준은 "자율주행 중이냐"입니다.**
>
> | 단계 | 어디서 | 이유 |
> |---|---|---|
> | 빌드 · `mapp` · `mkcsv` · `tune` | **NoMachine / 차량 모니터** (예전 그대로) | 조이스틱 저속 작업이라 부하 무관. `tune`은 마커를 드래그하는 GUI라 **화면이 필수** |
> | `rc` (주행) | **SSH + 노트북 RViz** | 주행 중 차량 렌더링 부하가 제어 주기를 흔든다 |
>
> 주행만 `ssh miru@10.1.1.250` → `sa` → `rc <맵> rviz:=false`, RViz는 노트북에서
> 따로 띄웁니다. 셋업은 **[4-4](#4-4-노트북에서-원격-rviz로-주행-2026-08-03-이후-권장-구성)**.
> alias·함수는 SSH로 접속해도 그대로 동작합니다(로그인 쉘이 zsh). 단 SSH 세션에는
> 화면이 없으므로 RViz를 여는 `mapp`·`rc`는 `rviz:=false`가 필요하고, `tune`은
> 끄는 인자 자체가 없어 SSH로는 못 씁니다.

빌드 안했으면
```bash
cd ~/IFAC_ws
./build_all.sh  
```
매핑하기
```bash
sa #전체 소스
mapp my_track0728   # 맵 이름 — 날짜 등으로 매번 새 이름을 쓸 것
```
> `maps/`는 다른 어떤 폴더보다 먼저 검색되므로, 기존 맵이나 패키지 맵
> (`particle_filter/maps`, `planning_pkg/maps`)과 같은 이름으로 저장하면 그 맵이
> 조용히 가려집니다. 그래서 이름이 겹치면 **주행 시작 전에** launch가 거부합니다.
> 정말 덮어쓸 때만 `mapp <이름> overwrite:=true` (기존 파일은 `.bak`으로 백업됨).
플래닝용 csv파일생성
```bash
mkcsv my_track0728 ccw   # 맵과 같은 이름 (기본 CCW, 시계 트랙만 cw)
cd ~/IFAC_ws && ./build_all.sh 
```
레이스라인 손보기 (선택)
```bash
sa
tune my_track0728   # mkcsv로 만든 맵 이름. v_max/mu/stride/falloff 전부 자동
# RViz에서 라인을 드래그 → 우클릭 "CSV 저장"
# 저장은 '소스 트리' CSV를 덮어쓰므로 반드시:
cd ~/IFAC_ws && ./build_all.sh
```
> `tune -h` 로 사용법 확인. 좁은 트랙에서 드래그가 안 먹으면 `tune my_track0728 min_wall:=0.25`,
> 추월(sp) 라인을 편집하려면 `tune my_track0728 csv:=my_track0728_sp_max6.00ms_margin0.08m_mu0.75.csv`.

만든 맵으로 주행
```bash
sa #전체 소스
rc my_track0728   # mapp/mkcsv와 같은 이름 — 생략하면 기본 맵(my_track0807)으로 돕니다
# 맵 이름은 원하는대로 바꿔서 설정할 수 있음
# 위 명령어들은 젯슨에 alias 설정함
#   → RViz(monitor.rviz)가 뜨면 "2D Pose Estimate"로 실제 차 위치 클릭+드래그
#   → 파티클 수렴하면 자율 스택 자동 시작 → 조이스틱 버튼 5로 자율주행 모드 실행
```

> 💻 **주행은 노트북 원격 RViz가 권장 구성입니다** (차량 부하 0).
> ```bash
> # 터미널1 (SSH)                      # 터미널2 (노트북)
> ssh miru@10.1.1.250
> sa && rc my_track0728 rviz:=false     rv
> ```
> → 2D Pose Estimate 로 초기 위치 → 수렴 확인 → **RViz 닫고** → 버튼 5 → 주행.
> 셋업과 주의사항은 **[4-4](#4-4-노트북에서-원격-rviz로-주행-2026-08-03-이후-권장-구성)** 참고.

주행 데이터 녹화 (별도 터미널)
```bash
sa
cd ~/IFAC_ws && rosbag       # run_MMDD_HHMMSS/ 로 저장, Ctrl+C 로 종료
```

녹화한 백 시각화 (ROS source 불필요)
```bash
bagviz                       # 서버 시작 → 브라우저에서 http://127.0.0.1:8998
# .db3 파일/백 폴더를 드래그앤드롭하거나, 화면의 로컬 백 목록 클릭
# 지도+궤적(속도 색)+LiDAR 재생, 속도/조향/횡오차 차트, 이벤트·경고 로그
# 다른 PC 브라우저에서 보려면: bagviz --host 0.0.0.0
```

### alias 목록 (`~/.zshrc`)

| alias | 하는 일 |
|---|---|
| `sa` | IFAC_ws 전체 워크스페이스 source (오버레이 순서 고정) |
| `rc [맵] [인자...]` | 실차 원커맨드 (`real_car.launch.py`) |
| `mapp <맵> [인자...]` | 매핑 (`mapping.launch.py`) |
| `mkcsv <맵> [cw\|ccw]` | 레이스라인/센터라인 CSV 생성 |
| `tune [맵] [인자...]` | 레이스라인 튜너 GUI (`raceline_tuner.launch.py`), 저장 후 `./build_all.sh` |
| `rosbag` | 전체 토픽 녹화 (marker류 제외) |
| `bagviz` | 로스백 시각화 웹앱 (`tools/bagviz.py`, 드래그앤드롭·재생·차트) |
| `hw` | 하드웨어 bringup만 (`rc hardware:=false`와 짝) |
| `cb` | `colcon build` |



**나눠 띄우기 (디버깅용) — 터미널 3개, A→B→C 순서 엄수:**

| 터미널 | 역할 | 명령 |
| **A** | 하드웨어 (LiDAR·VESC·odom·TF·joy·mux) | `ros2 launch f1tenth_stack bringup_launch.py` |
| **B** | 위치추정 (PF, map→odom TF + RViz) | `ros2 launch localization_bringup localization.launch.py backend:=particle_filter` |
| **C** | 자율 스택 (perception+planning+control+모니터) | `ros2 launch integration_pkg stack.launch.py env:=real_car` |

> - B는 `backend:=particle_filter`를 **반드시** 명시 (기본값은 slam_toolbox).

---

## 0. 이 워크스페이스가 하는 일

| 워크스페이스 | 역할 | 핵심 노드 |
|---|---|---|
| `perception_ws` | LiDAR로 **장애물 찾기** | `obstacle_detector_node` |
| `planning_ws` | 레이스라인 추종 + **회피 경로 만들기** | `spliner_node` 등 |
| `control_ws` | 경로를 **핸들/속도 명령**으로 | `control_node` |
| `integration_ws` | 위 셋을 **한 번에 켜고 상태 감시** | `stack.launch.py` |

데이터 흐름 (실차와 시뮬 동일):

```
LiDAR /scan
  → [perception] 장애물 탐지 → 실차 /perception/obstacles · 시뮬 /perception/obstacles_detected
  → [planning] frenet 변환 → 회피 경로 → /local_waypoints
  → [control] pure pursuit → 실차 /autonomy/ackermann_cmd · 시뮬 /drive
  → (실차만) ackermann_mux 가 조이스틱 teleop 과 중재 → /ackermann_cmd
  → ackermann_to_vesc → VESC
```
## 1. 최초 1회: 빌드
시뮬만 돌릴지, 실차까지 돌릴지에 따라 빌드 대상이 다릅니다.

| | 시뮬 | 실차 |
|---|---|---|
| `planning_ws` (메시지 포함, 항상 먼저) | ✅ | ✅ |
| `perception_ws` | ✅ | ✅ |
| `control_ws` | ✅ | ✅ |
| `integration_ws` | ✅ | ✅ |
| `hardware_ws` (LiDAR·VESC·odom·TF·mux·조이스틱) | — | ✅ |
| `localization_ws` (particle_filter) | — | ✅ |
| `sim_ws` (f1tenth_gym_ros) | ✅ 별도 | — |

빌드는 두 단계입니다 — 의존성은 **차량당 1회**, 코드 빌드는 **고칠 때마다**.

```bash
cd ~/IFAC_ws
./setup_deps.sh          # ① 차량당 1회: rosdep 의존성 + range_libc + numpy 검사
./build_all.sh           # ② 매번: 순서대로 colcon build (실차)
./build_all.sh sim       #         시뮬 (hardware/localization 제외)
```

- **`setup_deps.sh`** (차량당 1회) — `rosdep`으로 `tf_transformations`·`transforms3d`·
  `laser_proc`·`serial_driver` 등 설치(apt라 sudo 비밀번호를 물어봄) + `range_libc` 빌드.
  `range_libc` 소스가 없으면 clone 명령을 안내:
  `git clone https://github.com/f1tenth/range_libc ~/range_libc`.
- **`build_all.sh`** (매번) — 빌드 순서(planning 먼저 — 메시지 패키지가 여기 있음)와
  `--base-paths src`를 보장하며 **colcon build 만** 한다. 차량이 같으면 의존성은 이미
  있으므로 다시 설치하지 않는다. 시작 시 numpy·PF 의존성만 빠르게 확인하고 안 돼
  있으면 `./setup_deps.sh`를 실행하라며 멈춘다. 끝에 실행용 source 블록(아래 2장) 출력.

> ⚠️ **numpy는 apt 1.x 고정.** `pip install --user`로 numpy 2.x가 깔리면
> scipy·pythran·ROS C확장이 ABI로 깨지고 particle_filter가 조용히 죽습니다.
> 스크립트가 시작할 때 검사하며, 오염됐다면: `python3 -m pip uninstall -y numpy`.

> ⚠️ **시뮬(`sim_ws`)은 `build_all.sh` 대상이 아닙니다** — 별도로 빌드해야 합니다.
> 경로는 레포 안의 **`~/IFAC_ws/sim_ws`** 입니다(`~/sim_ws`가 아님).
> 다만 현재 `sim_ws/src/f1tenth_gym_ros`에 **`.py` 소스가 없고 `__pycache__`만
> 남아 있습니다.** 기존 `sim_ws/install/`에 빌드 산출물이 있어 지금 있는 건
> 돌지만, `install/`을 지우면 복구할 수 없습니다. 시뮬을 계속 쓸 거라면
> `install/.../f1tenth_gym_ros/gym_bridge.py`에서 소스를 되살리거나
> upstream(f1tenth_gym_ros)에서 다시 받아 두세요. (`sim_ws`는 git 미추적)

**빌드가 꼬이면** — 해당 ws에서 통째로 지우고 다시 빌드:
`rm -rf build install log && colcon build --base-paths src`.
`--symlink-install`은 쓰지 마세요(`ae_hyu_msgs`/심링크 에러). 다른 PC에서 복사해
`CMakeCache.txt directory is different`가 나면 `colcon build --cmake-clean-cache` 한 번.

---

## 2. 매번 실행 전: source (환경 켜기)

새 터미널을 열 때마다 아래를 **순서대로** 복사해서 붙여넣으세요.
(source = "이 워크스페이스를 쓸 수 있게 등록"하는 명령. 빌드가 아니라 매번 필요.)

**시뮬용** (gym 포함):

```bash
source /opt/ros/humble/setup.zsh
source ~/IFAC_ws/sim_ws/install/setup.zsh
source ~/IFAC_ws/planning_ws/install/setup.zsh
source ~/IFAC_ws/perception_ws/install/setup.zsh
source ~/IFAC_ws/control_ws/install/setup.zsh
source ~/IFAC_ws/integration_ws/install/setup.zsh
```

**실차용** (`sim_ws` 대신 `hardware_ws` + `localization_ws`)

```bash
source /opt/ros/humble/setup.zsh
source ~/IFAC_ws/planning_ws/install/setup.zsh
source ~/IFAC_ws/perception_ws/install/setup.zsh
source ~/IFAC_ws/control_ws/install/setup.zsh
source ~/IFAC_ws/integration_ws/install/setup.zsh
source ~/IFAC_ws/hardware_ws/install/setup.zsh
source ~/IFAC_ws/localization_ws/install/setup.zsh
```
실차용 소스 alias 해서 sa치면 위에 블록 전체 실행됩니다!

> `ROS_DOMAIN_ID=81`과 `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`는 `sa`가 아니라
> `~/.zshrc`(+`~/.bashrc`)에 들어 있어 새 터미널이면 자동으로 잡힙니다.
> **값을 바꿨다면 열려 있는 터미널을 전부 닫고 새로 열 것** — 반만 재시작하면
> 도메인이 갈려서 노드끼리 서로 안 보입니다. 노트북에서 RViz를 붙이려면
> 노트북에도 같은 두 줄이 있어야 합니다([4-4](#4-4-노트북에서-원격-rviz로-주행-2026-08-03-이후-권장-구성)).

## 3. 시뮬레이터로 실행하기 

### 3-1. 한 줄로 전체 켜기

터미널 하나에서 :

```bash
sa
ros2 launch integration_pkg stack.launch.py
```
- f1tenth_gym_ros 시뮬레이터 (차 2대: 내 차 + 상대 차)
- 실제 perception 파이프라인 (LiDAR → 장애물 탐지 → frenet 변환)
- planning 스택: 레이스라인(RACE) 추종 + **spliner 회피** +
  검증된 파일을 명시한 경우의 **SP 레인체인지**
  (`local_trajectory_node` + `spliner_node` + `recovery_supervisor_node`).
  recovery는 기본 기동되며 control이 state/target paired lease로 소비한다.
- control_ws의 C++ 컨트롤러 (ETH MAP)
- RViz (화면)
- integration 헬스 모니터

### 3-2. 다른 맵 / RViz 끄기

```bash
# 실차 트랙(my_track0807)을 시뮬에서 — 실차와 동일한 spliner+SP 라인으로 주행
ros2 launch integration_pkg stack.launch.py map:=my_track0807

# 헝가로링(코너 많은 맵)
ros2 launch integration_pkg stack.launch.py map:=Budapest_map

# 화면 없이 (테스트/서버용)
ros2 launch integration_pkg stack.launch.py rviz:=false
```

사용 가능한 시뮬 맵: **`my_track0807`(실차 트랙, 기본)**, `Spielberg_map`, `Budapest_map`, `my_map`.
(라인 CSV가 있는 맵만 뜹니다 — 없으면 launch가 가능한 맵 목록과 함께 즉시 에러)
`map:=<이름>`형태로 원하는 맵 연결

> ⚠️ **my_track0807은 16.1×8.1m로 작습니다.** 상대 차(gym 2번째 에이전트)를 LiDAR
> 사거리(≈30m) 밖에 '주차'시킬 공간이 없어, 스폰 안 해도 상대/벽이 유령 장애물로
> 몇 개 잡힙니다(`obs` 값이 큼). 주행 자체는 정상이며, **깨끗한 회피 데모는
> `Publish Point`로 정적 장애물을 직접 놓아** 확인하는 걸 권장합니다.

### 3-3. RViz에서 조작하기

RViz 창 위쪽 버튼을 씁니다.
**정적 장애물 놓기** 'Publish Point' 클릭 → 트랙 위 클릭 (차가 알아서 회피) 
**장애물 제거** 그 장애물 근처를 다시 `Publish Point`로 클릭 
**상대 차** `2D Goal Pose` → 트랙 위 원하는 위치 클릭 
**내 차 위치 옮기기** `2D Pose Estimate` → 위치 클릭 
**각종 제어 버튼** 왼쪽 `Panels` → `PitwallPanel` (자율주행 ON/OFF, 상대 속도 등) 
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

> ⚠️ **먼저 읽을 것 — 명령 배선 현황 (2026-07-23 확인, 헤드리스 기동 검증됨)**
> - ✅ **자율 명령**: `ackermann_mux`의 `navigation` 입력이
>   `/autonomy/ackermann_cmd`(control_pkg 발행)를 구독 → mux 출력이
>   `ackermann_drive_out:=ackermann_cmd` remap으로 `ackermann_to_vesc_node`에 연결.
> - ✅ **자율/수동 토글**: `joy_toggle_teleop`이 버튼 5로 래치 전환. 수동일 땐
>   `teleop`(우선순위 100)이 자율(우선순위 10)을 가리고, 자율로 넘기면 0속도
>   버스트 뒤 침묵해서 mux가 자율 명령으로 폴스루합니다.
> - ❌ **비상정지(E-stop)는 배선 안 됨.** `/safety/emergency_stop`을 발행하는
>   노드가 없습니다(joy에 estop 버튼 없음, `estop_node.cpp`는 launch 안 되는
>   legacy `vehicle_bringup`에만, `mux.yaml`에 `locks` 없음, `/ackermann_cmd_mux/mode`
>   없음). **정지는 버튼 5로 수동 전환하거나 차단기/`Ctrl-C`를 쓰세요.** 자세한
>   내용은 아래 "실차 안전장치" 절 참고.

> 🔑 **실차 PC에서도 오늘 `git pull` 했으면 실행 전에 반드시 다시 빌드하세요**
> (특히 `planning_ws`·`perception_ws`). 안 그러면 install에 옛 맵/파라미터가 남아
> **옛 맵으로 주행**하거나 `Cannot open CSV ...Spielberg_map_centerline.csv`로
> 죽습니다. 현재 맵은 `my_track0807`입니다 — `colcon list`/로그로 `my_track0807_iqp...`가
> 로드되는지 확인. (재빌드는 위 1장의 `./build_all.sh`.)

실차는 **A(하드웨어) → B(위치추정) → C(자율스택)** 순서로 띄웁니다.

> 🩺 **출발 전 perception 생존 확인 (run_0801_175634 사고 재발 방지)**
> 08-01 런에서 perception이 3분 내내 장애물 0건인 채로 주행한 사고가 있었습니다
> (RELIABLE `/tf`가 느린 구독자에 막혀 TF 신선도 게이트에 전멸 — 원격 RViz(WiFi) 의심).
> 스택이 뜬 뒤 **출발 전에** 반드시:
> 1. `env | grep RMW` → `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS` 확인
>    (`~/.zshrc`와 `real_car.launch.py` 양쪽에 있음 — 없으면 절대 출발 금지)
> 2. `ros2 topic echo /integration/status --once` → **`perception:OK`** 확인.
>    `perception:WAIT`가 계속되면 장애물 회피가 통째로 죽어 있는 것
> 3. 의심되면 `ros2 topic delay /tf` — 지연이 수백 ms를 넘으면 원격 RViz(WiFi)를
>    끄고 다시 비교
> 4. **주행 중에는 원격 RViz를 끌 것.** 2026-08-03부터 노트북 원격 RViz가 기본
>    구성이 됐으므로([4-4](#4-4-노트북에서-원격-rviz로-주행-2026-08-03-이후-권장-구성)),
>    **초기 위치를 찍고 수렴을 확인한 뒤 RViz 창을 닫고 출발**하는 것이 절차입니다.
>    주행 중 상태는 SSH에서 `ros2 topic echo /integration/status`로 봅니다.

### 4-A. 한 줄로 실행 (권장)

터미널 하나에서:

```bash
ros2 launch integration_pkg real_car.launch.py
```

이 한 줄이 A→B→C를 **자동 순서로** 처리합니다:

1. 하드웨어 bringup (LiDAR/VESC/joy/mux/TF)
2. 5초 뒤 particle_filter + **RViz(monitor.rviz) 자동 실행**
   — 초기 위치 지정과 주행 모니터를 겸하는 창 하나입니다. `pf.rviz`는 안 뜹니다
   (파티클/InferredPose 디스플레이를 `monitor.rviz`에 병합해 뒀습니다).
   같이 뜨는 `monitor_node`가 랩타임/CTE/속도 오버레이를 채웁니다
   (자율 스택이 뜬 뒤부터).
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
[frenet_frame_node]   Waiting for odom (real)...              ← /pf/pose/odom 이 없음
[local_trajectory_node] RUN: has_global=true, has_frenet=false
[control]             Local waypoints missing or stale -> stop  ← 궤적이 안 옴 → 정지
[perception_obstacle_bridge_node] TF base_link->map 없음        ← PF map TF가 없음
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
#   맵을 안 주면 localize.yaml 의 map: 을 따른다 (현재 my_track0807).

# 다른 맵을 쓰려면 map:=<이름> — ~/IFAC_ws/maps 를 먼저, 없으면
# particle_filter/maps 를 찾는다. 매핑으로 만든 맵은 재빌드 없이 바로 쓰인다.
ros2 launch localization_bringup localization.launch.py backend:=particle_filter map:=my_track2
```

#### 초기 위치 잡기 (필수) — 파티클 필터 초기화

파티클 필터는 켜지는 순간 파티클(차 위치 후보 3000개)을 **맵 전체에 흩뿌린
상태**로 시작합니다(로그 `GLOBAL INITIALIZATION`). 즉 차가 어디 있는지 아직
모릅니다. 이 상태에서 C를 띄우면 엉터리 위치로 차가 튑니다. **내가 실제
위치를 찍어줘야** 파티클이 모이고 LiDAR 스캔 매칭으로 수렴합니다.

> ⚠️ 초기 위치를 찍으려면 **터미널 A(하드웨어)가 먼저 떠 있어야** 합니다.
> laser→base_link TF가 없으면 클릭이 무시되고
> `Cannot set pose yet, waiting for laser transform` 로그만 뜹니다.

**RViz는 `real_car.launch.py`가 monitor_pkg 설정으로 자동으로 띄웁니다**
(`rviz:=false`로 끌 수 있음). 터미널 B를 직접 띄우는 경우(`localization.launch.py`
단독 실행)에는 여전히 `pf.rviz`가 뜹니다.

따로 띄우려면 — 반드시 **설치된** 설정 파일을 쓰세요:

```bash
# 초기 위치 + 주행 모니터 (권장, real_car.launch.py가 쓰는 것과 같은 설정)
rviz2 -d $(ros2 pkg prefix --share monitor_pkg)/rviz/monitor.rviz

# 초기 위치만 보는 최소 구성
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
#   ※ map:= 를 안 주면 planning_pkg/config/params.yaml의 trajectory/overtaking/
#     centerline CSV 기본값을 읽는다 (현재 my_track0807 기준).
ros2 launch integration_pkg stack.launch.py env:=real_car

#   다른 맵이면 map:=<이름> 하나로 CSV 3종이 함께 바뀐다 (params.yaml 수정 불필요).
#   단 CSV는 planning_pkg/config/ 안에 있어야 하므로 새로 뽑았다면 재빌드 필요.
ros2 launch integration_pkg stack.launch.py env:=real_car map:=my_track2
```

정상이면 `frenet_frame_node`가 더는 "Waiting for odom"을 찍지 않고
`[Frenet] XY(...) => s:.. d:..` 로그로 바뀝니다. 최종 확인은
`ros2 topic echo /integration/status` (3-4 참고).

### 4-3. 조이스틱으로 조종하기

하드웨어 bringup(터미널 A)이 뜨면 조이스틱이 바로 물립니다. 아래 매핑은
`joy_toggle_teleop` 노드(`hardware_ws/src/f1tenth_stack/config/joy_teleop.yaml`)
기준이며, **버튼/축 번호는 Xbox 계열 기준**입니다(패드가 다르면 번호가 다를 수
있음 — `ros2 topic echo /joy`로 확인).

| 조작 | 입력 | 동작 |
|---|---|---|
| **전진/후진 속도** | 축 `1` (왼쪽 스틱 상하) | 앞으로 밀면 전진, 뒤로 당기면 후진. 최고 ±1.5 m/s (`max_speed`) |
| **조향** | 축 `3` (오른쪽 스틱 좌우) | 좌/우 조향 (±0.34 rad, `max_steering_angle`) |
| **자율 ↔ 수동 토글** | 버튼 `5` (RB) | 누를 때마다 전환. **시작은 수동** |

> ℹ️ 이 teleop에는 **별도 후진 버튼도, 비상정지 버튼도 없습니다.** 후진은 스틱을
> 뒤로 당기면 됩니다(속도축이 음수). 정지는 스틱 중립 또는 아래 "안전장치" 참고.

- **수동 명령은 `teleop` 토픽**으로 나가고(`AckermannDriveStamped`), `ackermann_mux`가
  이를 자율 명령(`/autonomy/ackermann_cmd`)보다 **높은 우선순위**로 받아 최종
  `/ackermann_cmd`를 냅니다. 수동일 때는 teleop이 자율 명령을 가립니다.
- **현재 모드 확인**: `joy_toggle_teleop` 로그에 `--> MANUAL` / `--> AUTONOMOUS`가
  찍힙니다. (`/ackermann_cmd_mux/mode` 같은 모드 토픽은 **없습니다.**)
- 수동 모드에서 스틱을 놓거나 조이스틱이 0.5초(`joy_timeout`) 끊기면 속도 0을 냅니다.

### 4-4. 노트북에서 원격 RViz로 주행 (2026-08-03 이후 권장 구성)

예전에는 차량에 NoMachine으로 붙어 화면을 스트리밍하며 RViz를 봤습니다. 이 방식은
젯슨에 **삼중 부하**(RViz 렌더링 + H.264 인코딩 + 데스크톱 세션)를 겁니다. 차량 RAM이
7.4GB인데 데스크톱만으로 5GB를 쓰고 있어 여유가 2GB뿐이라, 주행 중 부하가 튀면 제어
주기가 흔들립니다.

**대신 스택은 SSH로 차량에서 띄우고, RViz만 노트북에서 직접 실행합니다.** 렌더링이
전부 노트북 몫이 되므로 차량 부하는 0입니다.

```
차량(Jetson)  ──  MIRU_5G  ──  노트북(Ubuntu 22.04)
  스택 (SSH로 실행)              rviz2 (렌더링은 여기서)
  토픽 발행만                    2D Pose Estimate → /initialpose 가 네트워크로 전달
```

#### 차량 네트워크 현황 (2026-08-03 설정 완료)

| 항목 | 값 |
|---|---|
| 접속 | `ssh miru@10.1.1.250` (또는 `ssh miru@IFAC.local`) |
| 호스트명 | `IFAC` |
| WiFi | `MIRU_5G` 고정 (`autoconnect-priority 100`) |
| IP | `10.1.1.250/24` static, GW `10.1.1.1`, DNS `166.104.100.100` |
| `ROS_DOMAIN_ID` | **81** (`~/.zshrc`·`~/.bashrc`) |
| Tailscale | 정지·비활성화 (학교망이 차단, health 경고만 냄) |
| ufw | `allow from 10.1.1.0/24` |

> ⚠️ **ufw 규칙을 지우지 마세요.** 22번만 열면 SSH는 되지만 DDS(UDP 27650번대)와
> 멀티캐스트 디스커버리가 막혀서 **RViz에 토픽이 하나도 안 뜹니다.**
> 도메인 81 → 포트 `7400 + 250×81 = 27650`. 도메인을 바꾸면 포트도 바뀝니다.

> ⚠️ **`ROS_DOMAIN_ID`는 차량·노트북이 반드시 같아야** 합니다. 다르면 같은 WiFi에
> 있어도 서로 완전히 안 보입니다. 값은 0–101 안에서 고를 것 — 그 위는 커널 임시
> 포트 대역(32768–60999)과 겹쳐 디스커버리가 간헐적으로 실패합니다.

#### 노트북 최초 1회

Ubuntu 22.04 + ROS 2 Humble 필요 (Humble은 22.04 전용, macOS는 RViz2 불가).

```bash
sudo apt install -y ros-humble-desktop \
  ros-humble-rviz-2d-overlay-plugins ros-humble-rviz-2d-overlay-msgs \
  python3-colcon-common-extensions qtbase5-dev avahi-daemon libnss-mdns

# 소스는 GitHub 말고 차량에서 직접 — 커스텀 메시지 정의가 한 글자라도 다르면
# DDS 타입 해시가 어긋나 그 토픽만 조용히 안 붙는다.
mkdir -p ~/IFAC_ws/planning_ws
rsync -av --exclude build --exclude install --exclude log \
  miru@10.1.1.250:~/IFAC_ws/planning_ws/src/ ~/IFAC_ws/planning_ws/src/

source /opt/ros/humble/setup.bash
cd ~/IFAC_ws/planning_ws
colcon build --base-paths src --packages-select \
  f1tenth_planning_msgs f1tenth_perception_msgs monitor_pkg pitwall_panel

cat >> ~/.bashrc <<'EOF'
source /opt/ros/humble/setup.bash
source ~/IFAC_ws/planning_ws/install/setup.bash
export ROS_DOMAIN_ID=81
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
alias rv='rviz2 -d $(ros2 pkg prefix --share monitor_pkg)/rviz/monitor.rviz'
EOF
```

빌드는 이 4개면 됩니다 — `planning_pkg`(플래너 본체)는 무거운 파이썬 의존성이
딸려오는데 RViz엔 필요 없습니다. `monitor_pkg`는 rviz 설정, `pitwall_panel`은 RViz
패널 플러그인 때문에 필요합니다. RobotModel은 `Description Source: Topic`이라
URDF가 네트워크로 넘어오므로 `hardware_ws`는 안 받아도 됩니다.

#### 실행 (터미널 3개)

```bash
# 터미널 1 — SSH (스택). alias·함수는 SSH로도 그대로 동작한다(로그인 쉘이 zsh).
ssh miru@10.1.1.250
sa
rc my_track0801 rviz:=false      # ← rviz:=false 필수. 빼면 차량에서도 RViz가 뜬다

# 터미널 2 — 노트북 로컬 (RViz)
rv

# 터미널 3 — SSH (녹화, 필요할 때)
ssh miru@10.1.1.250
sa && cd ~/IFAC_ws && rosbag
```

RViz에서 `2D Pose Estimate`로 초기 위치를 찍는 절차는 4-1과 동일합니다.

> ⚠️ **SSH 한 줄 명령으로는 alias가 안 먹습니다.** zsh는 `.zshrc`를 대화형 쉘일
> 때만 읽습니다. `ssh miru@10.1.1.250 'sa && rc ...'` → `command not found`.
> 자동화하려면 `ssh -t miru@10.1.1.250 'zsh -ic "sa && rc my_track0801"'`.

#### 🚨 수렴 확인 후 RViz를 닫고 출발할 것

원격 RViz는 **주행 중에 켜 두면 안 됩니다.** run_0801_175634 사고(perception이 3분간
장애물 0건)의 원인으로 지목된 것이 WiFi 원격 RViz입니다 — RELIABLE `/tf`가 느린
구독자에 막혀 TF 신선도 게이트에 전멸했습니다. 완화책
(`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`)은 **그 사고 이후에** 추가돼서 아직
실제 부하에서 검증되지 않았습니다.

**초기 위치 찍기 → 수렴 확인 → RViz 닫기 → 조이스틱 버튼 5 → 주행.**
이러면 차량 렌더링 부하와 TF 기아 위험을 둘 다 피합니다. 주행 중 상태는
SSH에서 `ros2 topic echo /integration/status`로 보세요.

#### 명령별 실행 위치 (SSH로 되는 것 / 화면이 필요한 것)

alias·함수는 **전부 차량에서(SSH로) 실행**합니다. 하드웨어가 차량에 물려 있고
결과물(맵·CSV)이 차량 소스 트리에 있어야 하기 때문입니다. 다만 **RViz 창을 여는
명령은 화면이 필요**해서 처리가 갈립니다.

| 명령 | SSH로 그대로? | 비고 |
|---|---|---|
| `./build_all.sh`, `cb` | ✅ | 화면 불필요 |
| `sa` | ✅ | |
| `mkcsv <맵> ccw` | ✅ | 순수 파이썬, 화면 불필요 |
| `rosbag` | ✅ | |
| `hw` | ✅ | |
| `mapp <맵>` | ⚠️ | 기본 `rviz:=true`. **`mapp <맵> rviz:=false`** 로 띄우고 RViz는 노트북에서 |
| `rc <맵>` | ⚠️ | 마찬가지로 **`rviz:=false`** + 노트북 RViz |
| `tune <맵>` | ❌ | **`rviz` 인자가 없어 RViz가 무조건 뜬다.** 아래 참고 |

**매핑을 원격으로** — 노트북에서 맵이 그려지는 걸 보며 조이스틱으로 한 바퀴 돕니다.
`mapp`이 쓰는 설정은 `particle_filter/rviz/pf.rviz`인데 노트북엔 그 패키지가 없으므로,
이미 빌드해 둔 `monitor.rviz`(`rv`)를 쓰면 됩니다 — Map(`/map`)·LaserScan(`/scan`)·TF가
들어 있어 매핑 확인에 충분합니다.

```bash
# 터미널1 (SSH)          # 터미널2 (노트북)
sa && mapp my_track0805 rviz:=false      rv
```

**`tune`은 차량 화면이 필요합니다.** [raceline_tuner.launch.py](planning_ws/src/planning_pkg/launch/raceline_tuner.launch.py)가
`rviz_node`를 조건 없이 띄우고 `rviz` 인자를 제공하지 않습니다. 게다가 인터랙티브
마커를 드래그해 편집하고 저장이 차량 소스 트리로 가는 도구입니다. **주행 중에 쓰는
게 아니므로 NoMachine(또는 차량 모니터)으로 하면 됩니다** — 부하가 문제되는 건 주행
중일 때뿐입니다. `ssh -X`도 되지만 인터랙티브 마커 드래그가 매우 느립니다.

`bagviz`는 차량에서 `bagviz --host 0.0.0.0`으로 띄우고 노트북 브라우저에서
`http://10.1.1.250:8998`로 봅니다.

---

### 실차 안전장치 (요약 — 배선 상세는 [README](README.md#실차-안전장치))

- **자율/수동 전환 = 조이스틱 버튼 5(RB).** 시작은 **수동**이라, 버튼 5를 눌러야
  자율 명령이 바퀴로 나갑니다(teleop이 침묵 → mux가 자율로 폴스루). 다시 누르면 수동.
- ⚠️ **비상정지(E-stop)는 배선 안 됨 — 소프트웨어 즉시정지가 없습니다.** 정지 수단은
  (1) 버튼 5로 수동 전환 후 스틱 중립, (2) 물리 kill 스위치/차단기, (3) `Ctrl-C`.
- 속도 상한 **3.6 m/s** (`control_pkg .../environments/real_car.yaml`의
  `vehicle/max_speed_mps`). 플래너는 6 m/s raceline에 `velocity_scale: 0.6`을
  곱해 같은 상한으로 맞춥니다.

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
  `rviz2 -d $(ros2 pkg prefix --share monitor_pkg)/rviz/monitor.rviz`
  (`real_car.launch.py`가 자동으로 띄워주므로 보통 직접 실행할 일은 없습니다.)
  `monitor.rviz`를 고쳤는데 반영이 안 되면 `planning_ws`를 다시 `colcon build`
  했는지 확인 — RViz는 `install/`에 복사된 설정을 읽습니다.

**코드/파라미터를 고쳤는데(또는 `git pull` 후) 실행하면 예전 동작 그대로**
→ 실행은 소스가 아니라 `install/`을 씁니다. **고친 뒤 반드시 다시 `colcon build`**
  하세요. 예: 맵을 `my_track0807`로 바꾼 뒤 재빌드를 안 하면 install엔 옛 맵이 남아
  `Cannot open CSV .../Spielberg_map_centerline.csv` 같은 로드 실패가 납니다.
→ `perception_ws`는 `--base-paths src` 없이 빌드하면 조용히 안 잡힙니다
  (`build_all.sh`는 항상 이걸 씀). 수동 빌드했다면 `colcon list`로 `perception_pkg` 확인.

**종료할 때 `obstacle_detector_node`에 `Exception in thread Thread-1`**
→ tf2 TransformListener의 내부 스레드가 남기는 로그로, 프로세스는 정상 종료(exit 0)합니다.
  `process has died [exit code 1]`이 함께 뜨면 그건 다른 문제입니다.

**노트북에서 `ros2 topic list`에 아무것도 안 보임 (원격 RViz)**
→ 순서대로 확인:
1. `echo $ROS_DOMAIN_ID` — 차량·노트북 **양쪽 다 81**이어야 함. 다르면 서로 완전히 안 보임
2. `env | grep RMW` — 노트북에 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`가 잡혀 있으면
   차량(기본 FastDDS)과 안 붙음. 양쪽 다 미설정이 정답
3. 차량 ufw — `sudo ufw status`에 `10.1.1.0/24` ALLOW가 있어야 함. 22번만 열려 있으면
   SSH는 되는데 DDS(UDP 27650번대)가 막혀 토픽이 안 보임
4. 노트북 방화벽(`sudo ufw status`), 같은 WiFi(MIRU_5G)인지
5. 공유기 무선 분리(AP isolation) — 켜져 있으면 기기 간 통신 자체가 불가

**커스텀 메시지 토픽만 안 보임 (`/perception/obstacles` 등)**
→ 노트북과 차량의 메시지 정의가 다르면 DDS 타입 해시가 어긋나 그 토픽만 조용히
  안 붙습니다. 차량에서 다시 동기화 후 재빌드:
  `rsync -av --exclude build --exclude install --exclude log miru@10.1.1.250:~/IFAC_ws/planning_ws/src/ ~/IFAC_ws/planning_ws/src/`

**SSH로 `sa`/`rc`가 `command not found`**
→ `ssh user@host '명령'` 형태는 비대화형이라 `.zshrc`를 안 읽습니다. 그냥 접속해서
  치거나, `ssh -t miru@10.1.1.250 'zsh -ic "sa && rc my_track0801"'`.

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
# ── 차량당 1회: 의존성 (rosdep + range_libc + numpy) ──
cd ~/IFAC_ws && ./setup_deps.sh       # 시뮬은: ./setup_deps.sh sim
# ── 코드 고칠 때마다: 빌드 ──
./build_all.sh                        # 실차 (시뮬은: ./build_all.sh sim). 끝에 source 블록 출력

# ── 매번 실행 ──
# 시뮬 (source 시뮬용 블록 후, 터미널 1개):
ros2 launch integration_pkg stack.launch.py

# 실차 (source 실차용 블록 후, 터미널 1개):
ros2 launch integration_pkg real_car.launch.py
#   → RViz가 자동으로 뜸. "2D Pose Estimate"로 실제 차 위치 클릭+드래그.
#   → 파티클이 수렴하면 자율 스택이 자동 시작됨. 버튼 5로 자율 명령 인가.
# ⚠️ 비상정지(E-stop)는 배선 안 돼 있음 — 4번 맨 위 경고 / "실차 안전장치" 참고

# ── 노트북 원격 RViz (권장 구성, 상세는 4-4) ──
ssh miru@10.1.1.250        # 차량: 호스트명 IFAC, ROS_DOMAIN_ID=81
sa && rc my_track0801 rviz:=false                                   # 터미널1 (SSH)
rv                                                                  # 터미널2 (노트북)
#   rv = rviz2 -d $(ros2 pkg prefix --share monitor_pkg)/rviz/monitor.rviz
#   → 2D Pose Estimate 로 초기 위치 → 수렴 확인 → 🚨 RViz 닫고 → 버튼 5 → 주행
#   노트북에도 ROS_DOMAIN_ID=81 + RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS 필수

# 실차를 단계별로 나눠 띄우려면 (디버깅용, 터미널 3개 — A→B→C 순서 엄수):
ros2 launch f1tenth_stack bringup_launch.py                                             # A 하드웨어
ros2 launch localization_bringup localization.launch.py backend:=particle_filter        # B 위치추정(+RViz)
ros2 launch integration_pkg stack.launch.py env:=real_car                               # C 자율스택
# ⚠️ C에서 "Waiting for odom (real)" / "map ... does not exist" 반복 = B 미수렴 (4번 참고)

# ── 새 트랙 매핑 (하드웨어+SLAM+RViz+저장 한 줄) ──
ros2 launch integration_pkg mapping.launch.py map_name:=my_track2
#   → 조이스틱으로 한 바퀴 → Ctrl+C → ~/IFAC_ws/maps/my_track2.{pgm,yaml}
#   → 레이스라인 CSV는 별도 (.venv python, 맵은 ~/IFAC_ws/maps 자동 탐색):
#        cd planning_ws/src/planning_pkg/scripts/global_traj_sim
#        ~/IFAC_ws/.venv/bin/python build_map.py my_track2 --dir ccw
#      뽑은 뒤 ./build_all.sh 로 CSV 반영
#   → 주행: ros2 launch integration_pkg real_car.launch.py map:=my_track2
```
