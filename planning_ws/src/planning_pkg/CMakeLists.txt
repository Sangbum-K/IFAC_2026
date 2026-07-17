cmake_minimum_required(VERSION 3.8)
project(planning_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ae_hyu_msgs REQUIRED)

include_directories(include)

# --- 팀 골격 노드 (인터페이스 자리표시자, /perception/objects -> /planning/plan) ---
add_executable(planning_node src/planning_node.cpp)
ament_target_dependencies(planning_node
  rclcpp
  std_msgs
  example_interfaces
)

# --- raceline 추종 파이프라인 (기존 planner_pkg에서 이식, 헤더/빌드 복원) ---
# raceline CSV -> /global_waypoints (2 Hz)
add_executable(global_trajectory_node src/global_trajectory_node.cpp)
ament_target_dependencies(global_trajectory_node
  rclcpp ae_hyu_msgs ament_index_cpp
)

# odom -> Frenet (s, d) 변환, /car_state/frenet/odom (40 Hz)
add_executable(frenet_frame_node src/frenet_frame_node.cpp)
ament_target_dependencies(frenet_frame_node
  rclcpp nav_msgs builtin_interfaces ae_hyu_msgs
)

# global waypoints 80개 로컬 윈도우 -> /local_waypoints (40 Hz).
# spliner_node의 회피 궤적(/planner/avoidance/otwpnts)이 유효하면 splice
add_executable(local_trajectory_node src/local_trajectory_node.cpp)
ament_target_dependencies(local_trajectory_node
  rclcpp nav_msgs ae_hyu_msgs
)

# 장애물 회피 스플라인 플래너 (ForzaETH/unicorn spliner C++ 포팅)
# /perception/obstacles -> /planner/avoidance/otwpnts (20 Hz)
add_executable(spliner_node src/spliner_node.cpp)
ament_target_dependencies(spliner_node
  rclcpp nav_msgs visualization_msgs ae_hyu_msgs
)

# waypoints/centerline RViz 마커
add_executable(planner_visualizer_node src/planner_visualizer_node.cpp)
ament_target_dependencies(planner_visualizer_node
  rclcpp visualization_msgs geometry_msgs ament_index_cpp ae_hyu_msgs
)

install(TARGETS
  planning_node
  global_trajectory_node
  frenet_frame_node
  local_trajectory_node
  spliner_node
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

# 개발용 맵 (대회 맵은 이 폴더를 교체하거나 도구의 --maps-dir로 지정)
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# RViz raceline 시각화 노드 (rclpy).
#   ros2 run planning_pkg raceline_viz_node.py --ros-args -p raceline_csv:=...
install(PROGRAMS scripts/global_traj_sim/ros2/raceline_viz_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# pitwall 텔레메트리 (import pitwall — 노드와 같은 디렉터리에 설치)
install(FILES scripts/lib/pitwall.py
  DESTINATION lib/${PROJECT_NAME}
)

# 시뮬레이션 데모 노드 (rclpy): sim_race.launch.py에서 사용.
# tracker는 control 팀 노드로 대체 예정, virtual_obstacle은 perception 대역
# opponent_driver는 gym 2-agent 상대 조종, scan_overlay는 perception 주입
install(PROGRAMS
  scripts/sim/waypoint_tracker_node.py
  scripts/sim/virtual_obstacle_node.py
  scripts/sim/virtual_opponent_node.py
  scripts/sim/opponent_driver_node.py
  scripts/sim/scan_overlay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Predictive Spliner 상대 예측 노드 (시뮬/실차 공용 런타임)
install(PROGRAMS scripts/planner/opponent_predictor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# RViz 인터랙티브 레이스라인 튜너 (raceline_tuner.launch.py)
install(PROGRAMS scripts/tuner/raceline_tuner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
