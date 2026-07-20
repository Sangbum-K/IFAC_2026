cmake_minimum_required(VERSION 3.8)
project(monitor_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(f1tenth_planning_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)

include_directories(include)

# 랩타임/CTE/속도/조향/장애물 요약 -> /monitor/* (10 Hz)
add_executable(monitor_node src/monitor_node.cpp)
ament_target_dependencies(monitor_node
  rclcpp std_msgs f1tenth_planning_msgs nav_msgs ackermann_msgs rviz_2d_overlay_msgs
)

install(TARGETS
  monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
