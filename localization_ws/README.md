# localization_ws

Localization 및 map-based pose estimation 계층을 위한 워크스페이스입니다.

## Includes

- `localization_bringup`: `slam_toolbox`, `AMCL`, `map_server` 기반 localization bringup 패키지

## Build

```bash
cd /home/seok/IFAC_ws/localization_ws
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch localization_bringup localization.launch.py backend:=slam_toolbox mode:=mapping
ros2 launch localization_bringup localization.launch.py backend:=slam_toolbox mode:=localization
ros2 launch localization_bringup localization.launch.py backend:=amcl mode:=localization
```
