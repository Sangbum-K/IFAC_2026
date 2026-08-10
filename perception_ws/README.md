# perception_ws

LiDAR 장애물 인지 패키지 `perception_pkg` (ROS 2 Humble, **C++ / ament_cmake**).

| 실행파일 | 역할 |
|---|---|
| `obstacle_detector_node` | `/scan` → 전처리 → TF(map) → 벽·트랙 필터 → 클러스터링 → 트래킹 → `/perception/obstacles` |
| `obstacle_viz_node` | `ObstacleArray` → RViz `MarkerArray` |

소스 구성:

| 경로 | 내용 |
|---|---|
| `include/perception_pkg/`, `src/{preprocessing,clustering,tracker,classifier,track_filter,distance_transform}.cpp` | ROS 비의존 코어 알고리즘 (`perception_core`) |
| `src/obstacle_detector_node.cpp`, `src/obstacle_viz_node.cpp` | ROS 노드 |
| `config/params.yaml`, `config/environments/*.yaml` | 공통 / 환경별 파라미터 |

빌드 (`f1tenth_perception_msgs`·`f1tenth_planning_msgs`가 있는 `planning_ws`를 먼저
빌드·source 해야 한다 — `build_all.sh`가 그 순서를 지킨다):

```bash
colcon build --base-paths src
```

`--base-paths src` 없이 빌드하면 워크스페이스 루트 잔재 때문에 패키지가 조용히
안 잡힌다.
