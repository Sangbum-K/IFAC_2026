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

## 필요 패키지

```bash
sudo apt install ros-humble-rviz-2d-overlay-plugins
```

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
