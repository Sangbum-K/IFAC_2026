cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(ae_hyu_msgs REQUIRED)

# pure pursuit + 스턱 복구 컨트롤러 (planning_ws의 sim tracker 실차 포팅)
# 빌드 전 planning_ws install을 source해야 ae_hyu_msgs를 찾는다.
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node
  rclcpp
  std_msgs
  example_interfaces
  nav_msgs
  ackermann_msgs
  ae_hyu_msgs
)

install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
