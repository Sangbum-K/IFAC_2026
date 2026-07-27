# monitor_pkg

주행 상태 모니터 — 랩타임/CTE/속도/조향/장애물 요약을 RViz 오버레이로 표시.
(ae_hyu_monitor 이식, 팀 스택 토픽 인터페이스에 맞게 수정)

## 실행

```bash
# sim (RViz 포함)
ros2 launch monitor_pkg monitor.launch.py

# 실차 — 차량 NUC에서 노드만
ros2 launch monitor_pkg monitor.launch.py env:=real_car rviz:=false

# 노트북에서 (같은 ROS_DOMAIN_ID)
rviz2 -d <planning_ws>/src/monitor_pkg/rviz/monitor.rviz
```

`rviz/monitor.rviz`는 sim_race.rviz 기반(맵/스캔/차량/장애물/플래너 마커)에
모니터 오버레이를 얹은 구성이라 이 창 하나로 전체 상황 파악 가능.

## 실차 초기 위치 지정도 이 창에서 (pf.rviz 병합)

`ros2 launch integration_pkg real_car.launch.py`가 띄우는 RViz가 이것입니다.
예전에는 `localization.launch.py`가 `pf.rviz`를 따로 띄웠는데, 초기 위치를
찍는 창과 주행을 보는 창이 나뉘어 있을 이유가 없어서 PF 디스플레이를
`monitor.rviz`로 옮겼습니다:

| 디스플레이 | 토픽 | 기본 |
|---|---|---|
| Particles (PoseArray, 주황) | `/pf/viz/particles` | on |
| InferredPose (Arrow, 초록) | `/pf/viz/inferred_pose` | on |
| FakeScan (PF 예측 스캔) | `/pf/viz/fake_scan` | **off** — `/scan`과 겹쳐 보면 수렴 품질 확인 |
| TF | — | **off** — map→odom→base_link 연결 확인용 |

툴바 `2D Pose Estimate`(→ `/initialpose`)와 `Fixed Frame: map`은 원래
있었습니다. `Publish Point`(→ `/clicked_point`)로 파티클 전체 재분산.

랩타임/CTE 오버레이는 자율 스택이 뜬 뒤부터 채워지므로, 초기 위치를 찍는
동안에는 비어 있는 게 정상입니다.

## 장애물 + 회피 라인

`Obstacles` 디스플레이(`/perception/obstacle_markers`)는
`perception_pkg/obstacle_viz_node`가 채웁니다. 예전에는 이 발행자가 sim에만
있어서 실차에서 "회피 라인은 휘는데 뭘 피하는지 안 보이는" 상태였습니다.

- 정적: 초록 원기둥 + `#id`
- 동적: 빨강 원기둥 + 노랑 속도 화살표 + `#id D 1.2m/s`
- 원기둥 지름 = 인지된 클러스터 지름 → `AvoidanceSpline`이 왜 그만큼
  벌어졌는지 눈으로 대조 가능
- 코스팅(외삽) 중인 트랙은 confidence만큼 투명해짐

`ObstaclesDetected`(기본 off)는 sim 전용입니다 — sim에서는 `Obstacles`가
`virtual_obstacle_node`의 ground-truth라 실제 인지 결과를 분리해 둡니다.

## 필요 패키지

```bash
sudo apt install ros-humble-rviz-2d-overlay-msgs ros-humble-rviz-2d-overlay-plugins
```

패키지가 없어도 `planning_ws` 빌드는 진행되지만, 이 모니터 노드는 스킵됩니다.
오버레이를 쓰려면 위 패키지를 설치한 뒤 `planning_ws`를 다시 빌드하세요.

## 표시 항목

- **Lap Info**: 현재 랩 수, 현재/직전 랩타임 · **Fastest/Mean Lap Time**
- **CTE**: Current / Max / Mean(10lap) — `/car_state/frenet/odom`의 d 절댓값
- **Speed / Steering**: PieChart (m/s, rad)
- **Obstacle**: Static/Dynamic 감지 여부

## 토픽 구독 (환경별 — config/environments/*.yaml)

| 용도 | sim | real_car / race_map |
|---|---|---|
| odom (속도) | `/ego_racecar/odom` | `/pf/pose/odom` (localization particle_filter) |
| 조향 입력 | `/drive` | `/ackermann_cmd` |
| 장애물 | `/perception/obstacles` | `/perception/obstacles_frenet` (bridge) |

공통: `/car_state/frenet/odom` (랩/CTE), `/global_waypoints` (트랙 길이)

## 토픽 발행

`/monitor/lap_info`, `/monitor/fastest_lap_time`, `/monitor/mean_lap_time`,
`/monitor/current_cte`, `/monitor/max_cte`, `/monitor/mean_cte`,
`/monitor/obstacle_info` (OverlayText),
`/monitor/current_speed`, `/monitor/steering_input` (Float32)
