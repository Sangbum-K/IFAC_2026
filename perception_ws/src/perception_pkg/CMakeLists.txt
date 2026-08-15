cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  # 인지 파이프라인은 스캔당 예산이 빠듯하다 (process_rate_hz 20Hz).
  # 최적화 없이 빌드하면 EDT/투영이 그 예산을 넘긴다.
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # FMA 자동 결합 금지. aarch64(Jetson) GCC 기본값은 -ffp-contract=fast 라서
  # `angle_min + i*angle_increment` 같은 식이 FMA 하나로 융합되고, 그 결과 각도가
  # 파이썬(numpy) 기준과 마지막 비트에서 갈린다. cos가 0 근처인 빔에서는 그 차이가
  # 상쇄로 증폭돼 포인트 x좌표가 수 ULP 어긋나고, 클러스터 분할 -> 트랙 ID -> 정적/동적
  # 판별까지 프레임을 거치며 발산한다 (원본 대비 A/B 재생에서 실측).
  # 이 스택의 인지 파라미터는 전부 그 기준 동작에 맞춰 튜닝돼 있으므로
  # 재현성을 속도보다 우선한다. 아키텍처/컴파일러가 바뀌어도 결과가 같아진다.
  add_compile_options(-ffp-contract=off)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(f1tenth_perception_msgs REQUIRED)
find_package(f1tenth_planning_msgs REQUIRED)

# ── 코어 알고리즘 (ROS 비의존) ───────────────────────────────────────────
# 전처리 / 클러스터링 / 트래킹 / 분류 / 트랙경계 / EDT.
# ROS 타입을 쓰지 않으므로 단위 테스트나 오프라인 재생 도구에서 그대로 쓸 수 있다.
add_library(perception_core STATIC
  src/preprocessing.cpp
  src/clustering.cpp
  src/tracker.cpp
  src/classifier.cpp
  src/track_filter.cpp
  src/distance_transform.cpp
)
target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(perception_core Eigen3::Eigen)

# ── 노드 ────────────────────────────────────────────────────────────────
add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
target_link_libraries(obstacle_detector_node perception_core)
ament_target_dependencies(obstacle_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2
  tf2_ros
  f1tenth_perception_msgs
  f1tenth_planning_msgs
)

add_executable(obstacle_viz_node src/obstacle_viz_node.cpp)
ament_target_dependencies(obstacle_viz_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  f1tenth_perception_msgs
)

install(TARGETS
  obstacle_detector_node
  obstacle_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nlohmann_json REQUIRED)
  ament_add_gtest(test_tracker_reacquisition
    test/test_tracker_reacquisition.cpp
  )
  target_link_libraries(test_tracker_reacquisition perception_core)
  ament_add_gtest(test_classifier_visibility
    test/test_classifier_visibility.cpp
  )
  target_link_libraries(test_classifier_visibility perception_core)
  ament_add_gtest(test_wall_distance_map
    test/test_wall_distance_map.cpp
  )
  target_link_libraries(test_wall_distance_map perception_core)
  ament_add_gtest(test_obstacle_radius_gate
    test/test_obstacle_radius_gate.cpp
  )
  target_link_libraries(test_obstacle_radius_gate
    perception_core
    nlohmann_json::nlohmann_json
  )
  target_compile_definitions(test_obstacle_radius_gate PRIVATE
    RADIUS_GATE_FIXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/run_0809_231809_61_2_radius_gate.json"
  )
endif()

ament_package()
