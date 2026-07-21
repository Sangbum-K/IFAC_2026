cmake_minimum_required(VERSION 3.8)
project(integration_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(f1tenth_planning_msgs REQUIRED)
find_package(f1tenth_perception_msgs REQUIRED)

add_executable(integration_node src/integration_node.cpp)
ament_target_dependencies(integration_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  ackermann_msgs
  f1tenth_planning_msgs
  f1tenth_perception_msgs
)

install(TARGETS integration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# real_car.launch.py가 자율 스택 시작 전에 띄우는 파티클 필터 수렴 대기 노드.
install(PROGRAMS scripts/wait_for_localization.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
