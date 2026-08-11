cmake_minimum_required(VERSION 3.8)
project(particle_filter_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_path(
  RANGELIBC_ROOT
  NAMES includes/RangeLib.h
  PATHS "$ENV{HOME}/range_libc"
  NO_DEFAULT_PATH
)
if(NOT RANGELIBC_ROOT)
  message(FATAL_ERROR
    "range_libc C++ headers not found. Clone it to $HOME/range_libc "
    "with ./setup_deps.sh before building particle_filter_cpp.")
endif()

add_library(particle_filter_cpp_core src/mcl_core.cpp)
target_compile_features(particle_filter_cpp_core PUBLIC cxx_std_17)
target_include_directories(particle_filter_cpp_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(particle_filter_cpp_node src/particle_filter_node.cpp)
target_compile_features(particle_filter_cpp_node PUBLIC cxx_std_17)
target_compile_definitions(particle_filter_cpp_node PRIVATE USE_CUDA=0)
target_include_directories(particle_filter_cpp_node SYSTEM PRIVATE ${RANGELIBC_ROOT})
target_link_libraries(particle_filter_cpp_node particle_filter_cpp_core)
ament_target_dependencies(particle_filter_cpp_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

install(TARGETS
  particle_filter_cpp_core
  particle_filter_cpp_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_mcl_core test/test_mcl_core.cpp)
  if(TARGET test_mcl_core)
    target_compile_features(test_mcl_core PUBLIC cxx_std_17)
    target_link_libraries(test_mcl_core particle_filter_cpp_core)
  endif()
  ament_add_pytest_test(test_pf_profiles test/test_pf_profiles.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(particle_filter_cpp_core)
ament_package()
