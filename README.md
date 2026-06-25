# IFAC_ws

F1TENTH 대회용으로 확장 가능한 ROS 2 분리형 스택입니다. 현재는 `perception -> planning -> control -> integration` 흐름이 실제로 실행되는 최소 골격으로 정리되어 있습니다.

## Structure

- `perception_ws`: 센서 입력을 받아 인지 결과를 publish하는 노드
- `planning_ws`: 인지 결과를 받아 계획을 생성하는 노드
- `control_ws`: 계획을 받아 제어 명령을 생성하는 노드
- `integration_ws`: 서비스 호출과 전체 스택 실행을 담당하는 노드
- `hardware_ws`: VESC, LiDAR, TF, URDF, joystick, emergency stop을 위한 하드웨어 bringup 워크스페이스
- `localization_ws`: `slam_toolbox`, `AMCL`, `map_server` 기반 localization 워크스페이스
- `launch/`: 루트에서 참고용으로 보는 launch 파일

## Build

각 워크스페이스는 분리되어 있으므로 아래 순서로 빌드하고 source 해야 합니다.

```bash
cd /home/seok/IFAC_ws/perception_ws
colcon build
source install/setup.bash

cd /home/seok/IFAC_ws/planning_ws
colcon build
source install/setup.bash

cd /home/seok/IFAC_ws/control_ws
colcon build
source install/setup.bash

cd /home/seok/IFAC_ws/integration_ws
colcon build
source install/setup.bash

cd /home/seok/IFAC_ws/hardware_ws
colcon build
source install/setup.bash

cd /home/seok/IFAC_ws/localization_ws
colcon build
source install/setup.bash
```

한 터미널에서 순서대로 source 해야 `integration_pkg` launch가 다른 패키지 실행 파일을 모두 찾을 수 있습니다.

## Run

전체 스택 실행:

```bash
ros2 launch integration_pkg stack.launch.py
```

하드웨어 bringup:

```bash
ros2 launch vehicle_bringup hardware.launch.py
ros2 launch vehicle_bringup hardware.launch.py env:=real_car
```

Localization bringup:

```bash
ros2 launch localization_bringup localization.launch.py backend:=slam_toolbox mode:=mapping
ros2 launch localization_bringup localization.launch.py backend:=slam_toolbox mode:=localization
ros2 launch localization_bringup localization.launch.py backend:=amcl mode:=localization
```

환경별 실행:

```bash
ros2 launch integration_pkg stack.launch.py env:=sim
ros2 launch integration_pkg stack.launch.py env:=mapping
ros2 launch integration_pkg stack.launch.py env:=race_map
ros2 launch integration_pkg stack.launch.py env:=real_car
```

파라미터 파일 위치:

- `perception_ws/src/perception_pkg/config/params.yaml`
- `planning_ws/src/planning_pkg/config/params.yaml`
- `control_ws/src/control_pkg/config/params.yaml`
- `integration_ws/src/integration_pkg/config/params.yaml`

환경별 override 파일 위치:

- `*/config/environments/sim.yaml`
- `*/config/environments/mapping.yaml`
- `*/config/environments/race_map.yaml`
- `*/config/environments/real_car.yaml`

현재 `integration_pkg`의 launch는 기본 `params.yaml`을 읽은 뒤, 선택한 `env` YAML을 덮어씁니다.
특히 `mapping`은 SLAM용, `race_map`은 저장된 map 기반 주행용, `real_car`는 실차 bringup용 시작점으로 보면 됩니다.

개별 노드 실행:

```bash
ros2 run perception_pkg perception_node
ros2 run planning_pkg planning_node
ros2 run control_pkg control_node
ros2 run integration_pkg integration_node
```

## Current Scope

현재 구현은 실제 차량 제어 전의 최소 통신 검증용 골격입니다.

- `perception_pkg`는 mock object 문자열을 publish합니다.
- `planning_pkg`는 perception 결과를 받아 mock plan을 publish합니다.
- `control_pkg`는 planning 결과를 받아 mock control command를 publish합니다.
- `integration_pkg`는 planning/control 서비스를 주기적으로 호출합니다.

실전 F1TENTH용으로 확장할 때 필요한 항목은 [F1TENTH_CHECKLIST.md](/home/seok/IFAC_ws/F1TENTH_CHECKLIST.md) 에 정리해두었습니다.
