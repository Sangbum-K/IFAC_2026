# localization_ws

SLAM Toolbox와 AMCL 기반 localization 워크스페이스입니다.

## Package

- `localization_bringup`: `slam_toolbox`, `AMCL`, `map_server` launch 패키지

## Build

```bash
cd /home/seok/IFAC_ws/localization_ws
colcon build
```

## Source

```bash
source /opt/ros/humble/setup.zsh
source /home/miru/IFAC_ws/hardware_ws/install/setup.zsh
source /home/miru/IFAC_ws/localization_ws/install/setup.zsh
```

## Run

직접 실행:

```bash
ros2 launch localization_bringup localization.launch.py backend:=slam_toolbox mode:=mapping
ros2 launch localization_bringup localization.launch.py backend:=slam_toolbox mode:=localization
ros2 launch localization_bringup localization.launch.py backend:=amcl mode:=localization
```

권장 실행:

```bash
ros2 launch f1tenth_stack mapping.launch.py env:=mapping sensor_interface:=ethernet
```

이 방식은 하드웨어 bringup과 `slam_toolbox`를 함께 실행합니다.

## Config

- `src/localization_bringup/config/localization/slam_toolbox_mapping.yaml`
- `src/localization_bringup/config/localization/slam_toolbox_localization.yaml`
- `src/localization_bringup/config/localization/amcl.yaml`
- `src/localization_bringup/config/localization/map_server.yaml`
