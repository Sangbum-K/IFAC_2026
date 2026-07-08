# IFAC_ws

ROS 2 Humble 기반 F1TENTH 실차/매핑 워크스페이스입니다. 현재 기준 메인 실행 진입점은 `hardware_ws/src/f1tenth_stack`이며, `joy -> ackermann -> vesc -> odom -> slam_toolbox` 흐름을 한 번에 올릴 수 있게 정리되어 있습니다.

## Structure

- `hardware_ws`: 실차 하드웨어, teleop, VESC, LiDAR, TF/URDF bringup
- `localization_ws`: `slam_toolbox`, `AMCL`, `map_server` 기반 localization
- `perception_ws`: 인지 실험용 워크스페이스
- `planning_ws`: 경로 계획 실험용 워크스페이스
- `control_ws`: 제어 실험용 워크스페이스
- `integration_ws`: 통합 실험용 워크스페이스

핵심 패키지는 아래와 같습니다.

- `f1tenth_stack`: F1TENTH-style 상위 launch/config 패키지
- `vehicle_bringup`: e-stop, URDF/TF, 기존 bringup 유틸리티
- `vehicle_teleop`: `/joy -> /ackermann_cmd` 변환 teleop 노드
- `localization_bringup`: SLAM/Localization launch 패키지

## Build

실차 주행과 매핑에 필요한 최소 빌드 순서는 아래입니다.

```bash
cd /home/seok/IFAC_ws/hardware_ws
colcon build

cd /home/seok/IFAC_ws/localization_ws
colcon build
```

## Source

실행 전에는 아래 순서로 source 합니다.

```bash
source /opt/ros/humble/setup.zsh
source /home/miru/IFAC_ws/hardware_ws/install/setup.zsh
source /home/miru/IFAC_ws/localization_ws/install/setup.zsh
```

`hardware_ws`만 쓸 때는 마지막 줄은 생략해도 됩니다.

## Run

하드웨어 수동주행 bringup:

```bash
ros2 launch f1tenth_stack bringup.launch.py env:=real_car sensor_interface:=ethernet
```

SLAM Toolbox 매핑:

```bash
ros2 launch f1tenth_stack mapping.launch.py env:=mapping sensor_interface:=ethernet
```

위 `mapping.launch.py`는 아래를 함께 실행합니다.

- `joy_node`
- `vehicle_teleop/ackermann_joy_teleop`
- `vesc_driver`
- `vesc_ackermann/ackermann_to_vesc_node`
- `vesc_ackermann/vesc_to_odom_node`
- `urg_node`
- `robot_state_publisher` / TF
- `estop_node`
- `slam_toolbox`

## Parameters

F1TENTH-style 파라미터 엔트리포인트:

- `hardware_ws/src/f1tenth_stack/config/params.yaml`
- `hardware_ws/src/f1tenth_stack/config/environments/mapping.yaml`
- `hardware_ws/src/f1tenth_stack/config/environments/race_map.yaml`
- `hardware_ws/src/f1tenth_stack/config/environments/real_car.yaml`
- `hardware_ws/src/f1tenth_stack/config/environments/sim.yaml`

Localization 설정:

- `localization_ws/src/localization_bringup/config/localization/slam_toolbox_mapping.yaml`
- `localization_ws/src/localization_bringup/config/localization/slam_toolbox_localization.yaml`
- `localization_ws/src/localization_bringup/config/localization/amcl.yaml`
- `localization_ws/src/localization_bringup/config/localization/map_server.yaml`

## Notes

- 이 레포는 ROS 2 Humble 기준으로 정리되어 있습니다.
- `f1tenth_stack`은 `f1tenth/f1tenth_system`의 구조를 참고했지만, 드라이버와 launch는 현재 레포에 맞게 재구성한 것입니다.
- 실전 F1TENTH 확장 체크리스트는 [F1TENTH_CHECKLIST.md](/home/seok/IFAC_ws/F1TENTH_CHECKLIST.md)에 정리되어 있습니다.
