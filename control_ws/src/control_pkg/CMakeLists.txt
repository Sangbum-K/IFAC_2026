cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(ae_hyu_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(control_node src/control_node.cpp)
target_compile_features(control_node PUBLIC cxx_std_17)
ament_target_dependencies(control_node
  ackermann_msgs
  ae_hyu_msgs
  ament_index_cpp
  example_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
