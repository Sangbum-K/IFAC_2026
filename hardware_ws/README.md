# hardware_ws

실차 하드웨어 및 F1TENTH-style bringup 워크스페이스입니다.

## Packages

- `f1tenth_stack`: 메인 launch/config 패키지
- `vehicle_bringup`: URDF/TF, e-stop, 기존 bringup 유틸리티
- `vehicle_teleop`: joystick 입력을 `/ackermann_cmd`로 변환
- `vesc/vesc_driver`: VESC 드라이버
- `vesc/vesc_ackermann`: `ackermann <-> vesc`, `vesc -> odom`
- `urg_node`: Hokuyo UST-10LX 드라이버

## Build

```bash
cd /home/seok/IFAC_ws/hardware_ws
colcon build
```

## Source

```bash
source /opt/ros/humble/setup.zsh
source /home/miru/IFAC_ws/hardware_ws/install/setup.zsh
```

## Run

수동주행 하드웨어 bringup:

```bash
ros2 launch f1tenth_stack bringup.launch.py env:=real_car sensor_interface:=ethernet
```

이 launch는 아래 노드를 함께 실행합니다.

- `joy_node`
- `vehicle_teleop/ackermann_joy_teleop`
- `vesc_driver`
- `vesc_ackermann/ackermann_to_vesc_node`
- `vesc_ackermann/vesc_to_odom_node`
- `urg_node`
- `robot_state_publisher`
- `tf2_ros/static_transform_publisher`
- `vehicle_bringup/estop_node`

기존 하위 launch도 여전히 남아 있습니다.

- `vehicle_bringup/manual_drive.launch.py`
- `vehicle_bringup/mapping_stack.launch.py`
- `vehicle_bringup/hardware.launch.py`

## Config

메인 파라미터 파일:

- `src/f1tenth_stack/config/params.yaml`
- `src/f1tenth_stack/config/environments/real_car.yaml`
- `src/f1tenth_stack/config/environments/mapping.yaml`

조이스틱 축/버튼 매핑과 속도 제한은 `ackermann_joy_teleop` 섹션에서 조정합니다.
