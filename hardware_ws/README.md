# hardware_ws

실차 하드웨어 bringup 워크스페이스입니다. `f1tenth_stack`은 공식
[`f1tenth/f1tenth_system`](https://github.com/f1tenth/f1tenth_system) 방식으로 구성되어
있으며, **하드웨어 드라이버 계층만** 담당합니다. 매핑(SLAM)과 자율주행은 그 위에 따로 올립니다.

## Packages

- `f1tenth_stack`: 메인 launch/config 패키지 (원본 f1tenth_system 구조)
- `ackermann_mux`: teleop/navigation Ackermann 명령 먹스
- `teleop_tools/joy_teleop`: 조이스틱 → Ackermann 명령 변환
- `vesc/vesc_driver`: VESC 시리얼 드라이버
- `vesc/vesc_ackermann`: `ackermann <-> vesc`, `vesc -> odom`
- `urg_node`: Hokuyo 라이다 드라이버
- `vehicle_bringup`: (legacy) URDF/TF·e-stop 유틸리티 — 현재 기본 bringup 경로에서는 사용하지 않음

## Build

```bash
source /opt/ros/humble/setup.bash

cd ~/IFAC_ws/hardware_ws
colcon build --symlink-install
```

`f1tenth_stack`은 이제 self-contained라 다른 워크스페이스 없이 단독 빌드됩니다.
(라이다 매핑에는 `slam_toolbox`가 필요: `sudo apt install ros-humble-slam-toolbox`)

## Run

각 명령은 **새 터미널**에서 실행하며, 매 터미널마다 아래 두 줄을 먼저 source 합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/IFAC_ws/hardware_ws/install/setup.bash
```

### Terminal 1 — 하드웨어 bringup (필수)

라이다 + VESC + 조이스틱 + mux + static TF를 한 번에 올립니다.

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

실행되는 노드:

- `joy` / `joy_toggle_teleop` — 조이스틱 입력 → `/teleop` + `/estop_cmd` + `/safety/emergency_stop`
  (upstream `joy_teleop`을 안 쓰고 자체 노드로 교체 — 데드맨 방식이 아니라 버튼 한 번으로
  전환되는 토글/e-stop 방식이 필요해서)
- `vesc_driver_node` — VESC 시리얼 통신
- `ackermann_to_vesc_node` — Ackermann 명령 → VESC
- `vesc_to_odom_node` — VESC 속도 + IMU 상대 yaw 기반 `/odom` + `odom→base_link` TF
- `urg_node` — 라이다 → `/scan`
- `ackermann_mux` — teleop/estop/navigation 먹스 → `ackermann_cmd`
- `static_baselink_to_laser` — `base_link→laser` static TF (0.27, 0, 0.11)

> `throttle_interpolator`는 원본과 동일하게 정의만 되어 있고 기본 비활성(주석)입니다.

#### 조이스틱 조작 (버튼은 전부 edge-triggered — 누르고 있을 필요 없음, 한 번씩만 누르면 됨)

| 버튼 | 기능 |
|---|---|
| **4 (LB)** | **E-STOP 토글.** MANUAL/AUTONOMOUS 어느 모드에서든 누르면 즉시 정지(최우선). 정지 중 다시 누르면 해제 — **항상 MANUAL로만 복귀**(AUTONOMOUS로 바로 안 돌아감) |
| **5 (RB)** | MANUAL ↔ AUTONOMOUS 토글. **E-STOP 중에는 완전히 무시됨** |
| Axis 1 / Axis 3 | 속도 / 조향 (MANUAL일 때만 반영) |

**정지 해제 후 다시 자율주행 하려면 버튼4(해제 → MANUAL) → 버튼5(→ AUTONOMOUS) 순서로 두 번 눌러야 합니다.** (해제하자마자 자동으로 자율주행이 재개되지 않음 — 안전을 위한 의도된 동작)

동작 원리 (`f1tenth_stack/joy_toggle_teleop.py`):

- **MANUAL**: 조이스틱 축 값을 `teleop`에 계속 발행 (mux priority 100)
- **AUTONOMOUS**: `teleop`에 침묵 → mux가 `/autonomy/ackermann_cmd`(control_pkg 출력, priority 10)로 자동 전환
- **ESTOP**: `/estop_cmd`에 speed=0/steer=0을 30Hz로 끊임없이 발행 (mux priority 200, 항상 최우선) — mux의 `locks` 기능 대신 실제 0-명령 topic을 쓴 이유는, lock은 낮은 우선순위 topic을 막기만 할 뿐 스스로 0을 쏘지 않아서 mux가 마지막 주행 명령을 그대로 흘려보낼 위험이 있기 때문. 동시에 `/safety/emergency_stop`(std_msgs/Bool)도 발행되어 `control_pkg`·`recovery_supervisor_node`가 각자 자율주행 출력을 0으로 만드는 **이중 안전장치** 역할을 함

### Terminal 2 — SLAM 매핑 (지도 생성 시)

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(ros2 pkg prefix f1tenth_stack)/share/f1tenth_stack/config/f1tenth_online_async.yaml
```

`map→odom` TF와 `/map`을 실시간 생성합니다.

### Terminal 3 — 지도 저장

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### (선택) Terminal 4 — 시각화

```bash
rviz2
```

자율주행 시에는 상위 플래너/제어(`planning_ws`·`control_ws`)가
`ackermann_mux`의 navigation 입력(토픽 `drive`)에 명령을 발행합니다.

## Config

`src/f1tenth_stack/config/`:

- `joy_teleop.yaml` — 조이스틱 축/버튼 매핑 (`toggle_button: 5`, `estop_button: 4`)
- `mux.yaml` — teleop(100)/estop(200)/navigation(10) 우선순위
- `vesc.yaml` — VESC 게인·포트·wheelbase·IMU yaw odometry 설정
- `sensors.yaml` — 라이다 IP/포트 등
- `f1tenth_online_async.yaml` — slam_toolbox 매핑 파라미터

### 실행 전 하드웨어 값 확인

| 파일 | 항목 | 현재 값 | 확인 |
|---|---|---|---|
| `vesc.yaml` | `port` | `/dev/ttyACM0` | udev 규칙 쓰면 `/dev/sensors/vesc`로 변경 |
| `vesc.yaml` | `wheelbase` | `0.33` | 차량 실측값 |
| `sensors.yaml` | `ip_address` / `ip_port` | `192.168.0.10` / `10940` | 실제 Hokuyo와 일치 확인 |
