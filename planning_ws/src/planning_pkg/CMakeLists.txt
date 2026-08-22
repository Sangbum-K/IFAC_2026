cmake_minimum_required(VERSION 3.8)
project(planning_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(f1tenth_planning_msgs REQUIRED)

include_directories(include)

# --- raceline 추종 파이프라인 (기존 planner_pkg에서 이식, 헤더/빌드 복원) ---
# raceline CSV -> /global_waypoints (2 Hz)
add_executable(global_trajectory_node src/global_trajectory_node.cpp)
ament_target_dependencies(global_trajectory_node
  rclcpp f1tenth_planning_msgs ament_index_cpp
)

# odom -> Frenet (s, d) 변환, /car_state/frenet/odom (40 Hz)
add_executable(frenet_frame_node src/frenet_frame_node.cpp)
ament_target_dependencies(frenet_frame_node
  rclcpp nav_msgs builtin_interfaces f1tenth_planning_msgs
)

# global waypoints 80개 로컬 윈도우 -> /local_waypoints (40 Hz).
# spliner_node의 회피 궤적(/planner/avoidance/otwpnts)이 유효하면 splice
add_executable(local_trajectory_node src/local_trajectory_node.cpp)
ament_target_dependencies(local_trajectory_node
  rclcpp nav_msgs std_msgs f1tenth_planning_msgs
)

# 장애물 회피 스플라인 플래너 (ForzaETH/unicorn spliner C++ 포팅)
# /perception/obstacles -> /planner/avoidance/otwpnts (20 Hz)
add_executable(spliner_node src/spliner_node.cpp)
ament_target_dependencies(spliner_node
  rclcpp nav_msgs std_msgs visualization_msgs f1tenth_planning_msgs
)

# 벽 충돌/진행불능(stuck) 판정·복구 감독 (mina Task 4).
# /planner/recovery/{state,target_speed} — 실차 control_node가 소비
add_executable(recovery_supervisor_node src/recovery_supervisor_node.cpp)
ament_target_dependencies(recovery_supervisor_node
  rclcpp nav_msgs std_msgs visualization_msgs f1tenth_planning_msgs
)

# waypoints/centerline RViz 마커
add_executable(planner_visualizer_node src/planner_visualizer_node.cpp)
ament_target_dependencies(planner_visualizer_node
  rclcpp visualization_msgs geometry_msgs ament_index_cpp f1tenth_planning_msgs
)

install(TARGETS
  global_trajectory_node
  frenet_frame_node
  local_trajectory_node
  spliner_node
  recovery_supervisor_node
  planner_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 맵은 더 이상 패키지에 설치하지 않는다.
# 워크스페이스 공용 폴더 <ws>/maps 가 단일 위치다 (mapping.launch.py가 저장하고,
# localization/tuner/sim launch가 전부 거기서 읽는다). 패키지에도 사본을 두면
# 같은 이름의 맵이 두 벌 생겨 어느 쪽이 쓰였는지 조용히 갈린다 — 08-05 주행에서
# 실제로 맵이 어긋난 원인과 같은 종류의 사고다.
# 대회 맵은 그 폴더를 교체하거나 maps_dir:= / --maps-dir 로 가리킨다.

# pitwall 텔레메트리 (import pitwall — 노드와 같은 디렉터리에 설치)
install(FILES scripts/lib/pitwall.py
  DESTINATION lib/${PROJECT_NAME}
)

# 시뮬레이션 데모 노드 (rclpy): sim_race.launch.py에서 사용.
# tracker: control_pkg/control_node 가 이미 sim 프로파일까지 갖춘 대체
# 구현이다 (integration stack env:=sim 은 그쪽을 쓴다). 이 노드는 gym 단독
# 데모용으로만 남아 있으며 recovery lease 계약을 두 벌 유지해야 한다.
# virtual_obstacle = 정적 장애물 + perception 주입(tracking merge / scan overlay).
# opponent_driver = gym 2-agent 상대 조종 (path/ftg/manual).
# sim_odom_drift = gym GT를 드리프트 있는 휠 오도메트리로 바꿔 particle_filter가
# 보정할 odom 레그를 만든다 (sim_race.launch.py pf:=true).
install(PROGRAMS
  scripts/sim/waypoint_tracker_node.py
  scripts/sim/virtual_obstacle_node.py
  scripts/sim/opponent_driver_node.py
  scripts/sim/sim_odom_drift_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Predictive Spliner 상대 예측 노드 (시뮬/실차 공용 런타임)
install(PROGRAMS scripts/planner/opponent_predictor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 실차 인터페이스 브릿지 (planner_stack env:=real_car에서만 실행):
# perception 팀 laser-frame 장애물 -> frenet
# (/pf/pose/odom은 localization particle_filter가 직접 발행 — pose 브릿지 제거됨)
install(PROGRAMS
  scripts/planner/perception_obstacle_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/planner/obstacle_bridge_policy.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    safety_invariants
    test/test_safety_invariants.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    tuner_safety_model
    test/test_tuner_safety_model.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    line_variant_routing
    test/test_line_variant_routing.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    line_resolver
    test/test_line_resolver.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    obstacle_bridge_policy
    test/test_obstacle_bridge_policy.py
    TIMEOUT 60
  )
endif()

ament_package()
