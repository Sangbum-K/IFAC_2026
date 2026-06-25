# hardware_ws

실차 하드웨어 계층을 위한 워크스페이스입니다.

## Includes

- `vehicle_bringup`: VESC, LiDAR, TF, URDF, joystick, emergency stop을 묶는 bringup 패키지

## Expected External Drivers

아래 외부 드라이버 패키지를 `hardware_ws/src/` 아래에 추가하는 구조를 권장합니다.

- `vesc_driver`
- 사용하는 LiDAR 드라이버 패키지
- 필요 시 camera driver

현재 `vehicle_bringup` launch는 외부 드라이버 패키지가 없더라도 TF, URDF, joystick, e-stop 계층만 먼저 띄울 수 있게 되어 있습니다.

## Build

```bash
cd /home/seok/IFAC_ws/hardware_ws
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch vehicle_bringup hardware.launch.py
```

실차 bringup 예시:

```bash
ros2 launch vehicle_bringup hardware.launch.py env:=real_car
```
