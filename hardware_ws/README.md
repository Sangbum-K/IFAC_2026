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

- `joy` / `joy_teleop` — 조이스틱 입력 → `/teleop`
- `vesc_driver_node` — VESC 시리얼 통신
- `ackermann_to_vesc_node` — Ackermann 명령 → VESC
- `vesc_to_odom_node` — VESC 속도 + IMU 상대 yaw 기반 `/odom` + `odom→base_link` TF
- `urg_node` — 라이다 → `/scan`
- `ackermann_mux` — teleop/navigation 먹스 → `ackermann_drive`
- `static_baselink_to_laser` — `base_link→laser` static TF (0.27, 0, 0.11)

> `throttle_interpolator`는 원본과 동일하게 정의만 되어 있고 기본 비활성(주석)입니다.

조작: **LB(버튼4) = teleop 데드맨**, **RB(버튼5) = navigation 데드맨**.

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

- `joy_teleop.yaml` — 조이스틱 축/버튼·데드맨 매핑
- `mux.yaml` — teleop/navigation 우선순위
- `vesc.yaml` — VESC 게인·포트·wheelbase·IMU yaw odometry 설정
- `sensors.yaml` — 라이다 IP/포트 등
- `f1tenth_online_async.yaml` — slam_toolbox 매핑 파라미터

### 실행 전 하드웨어 값 확인

| 파일 | 항목 | 현재 값 | 확인 |
|---|---|---|---|
| `vesc.yaml` | `port` | `/dev/ttyACM0` | udev 규칙 쓰면 `/dev/sensors/vesc`로 변경 |
| `vesc.yaml` | `wheelbase` | `0.33` | 차량 실측값 |
| `sensors.yaml` | `ip_address` / `ip_port` | `192.168.0.10` / `10940` | 실제 Hokuyo와 일치 확인 |
