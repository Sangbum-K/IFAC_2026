# control_ws

`control_pkg` — `/local_waypoints`(ae_hyu_msgs/WpntArray)를 추종하는
pure pursuit 컨트롤러. planning_ws의 시뮬 전용 `waypoint_tracker_node.py`에서
검증한 로직의 실차 C++ 포팅이다.

```
/local_waypoints ──▶ control_node ──▶ /ackermann_cmd (실차, vesc_ackermann)
odom(환경별)     ──▶      │              /drive      (시뮬, f1tenth_gym)
/safety/emergency_stop ───┘   E-stop 시 즉시 정지 명령
```

## 기능

- **Pure pursuit**: 속도 비례 lookahead, raceline 속도 프로파일 추종
  (spliner 회피 궤적은 planning의 local_trajectory_node가 이미 병합해 줌)
- **벽 충돌/스턱 복구**: 전진 명령 중 0.6 s 이상 정지하면 경로 방향으로
  기수가 돌아가게 후진 후 감속 재개. 실차 환경(real_car.yaml)에서는
  실험 검증 전까지 `recovery/enable: false`
- **autonomous 모드**: 실차는 OFF로 부팅(조종기 주행 방해 안 함).
  `ros2 service call /control/set_mode example_interfaces/srv/Trigger`로 토글.
  OFF 전환 시 정지 명령을 한 번 발행 후 침묵 (teleop이 토픽 소유)
- **E-stop**: `/safety/emergency_stop`(latched Bool, hardware_ws estop_node)
  수신 시 모드 무관 정지 명령 지속 발행
- **안전 상한**: `pp/max_speed_mps`(실차 4.0 = hardware vehicle.max_speed),
  로컬 경로 0.5 s 두절 시 자동 정지

## 빌드 (ae_hyu_msgs 의존 — planning_ws 먼저)

```bash
source /opt/ros/humble/setup.bash
source ~/IFAC_ws/planning_ws/install/setup.bash   # ae_hyu_msgs
cd ~/IFAC_ws/control_ws && colcon build
source install/setup.bash
```

## 실행

```bash
# 실차 (hardware_ws bringup + localization + planning 스택이 떠 있어야 함)
ros2 run control_pkg control_node --ros-args \
  --params-file $(ros2 pkg prefix control_pkg)/share/control_pkg/config/params.yaml \
  --params-file $(ros2 pkg prefix control_pkg)/share/control_pkg/config/environments/real_car.yaml
# 자율주행 시작/정지 토글:
ros2 service call /control/set_mode example_interfaces/srv/Trigger

# 시뮬 (f1tenth_gym) — planning의 Python 트래커 대신 이 노드로:
ros2 launch planning_pkg sim_race.launch.py tracker:=false &
ros2 run control_pkg control_node --ros-args \
  --params-file $(ros2 pkg prefix control_pkg)/share/control_pkg/config/params.yaml \
  --params-file $(ros2 pkg prefix control_pkg)/share/control_pkg/config/environments/sim.yaml
```

검증(gym): 3.5 m/s 주행 횡편차 ~0, 클릭 장애물 회피 |d|=0.69 m,
set_mode OFF 즉시 정지, E-stop 즉시 정지 확인 (2026-07-09).
