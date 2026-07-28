# monitor_pkg

주행 상태 모니터 — 랩타임/CTE/속도/조향을 RViz 오버레이로 표시.
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

`rviz/monitor.rviz`는 sim_race.rviz 기반(맵/스캔/차량/플래너 마커)에
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

## 차량 위치/방향 (화살표)

`OdomHeadingReal`(`/pf/pose/odom`) / `OdomHeadingSim`(`/ego_racecar/odom`)
Odometry 디스플레이. 둘 다 항상 켜 두고, 발행되는 쪽만 그려진다
(`EgoModelSim`/`EgoModelReal` 과 같은 방식).

- 하늘색 작은 화살표(총 길이 0.34m — 차체 55cm보다 짧다)가 차 위에 얹힌다
- `Keep: 1` — 최신 화살표 하나만. 기본값 100 이면 지나온 자리마다 화살표가
  쌓여 트랙이 화살표로 뒤덮인다
- `Position/Angle Tolerance: 0` — 매 메시지 갱신 (정지 중에도 방향이 보인다)
- Covariance 타원은 꺼 뒀다

## 주행 궤적 (실제로 지나온 흔적)

`DrivenPath` (`/monitor/driven_path`, MarkerArray) — 차가 실제로 지나온 자리를
선으로 남긴다. 점마다 **그때의 속도로 색을 칠하므로**
`scripts/global_traj_sim`이 뽑는 시뮬 궤적 그림(viridis 컬러맵)과 같은 눈으로
읽을 수 있다. 레이스라인(`GlobalRaceline`)과 겹쳐 보면 어디서 안쪽으로
파고들고 어디서 밀려나는지 바로 보인다.

- **직전 랩은 회색 고스트**로 남는다 (`trail/keep_previous_laps`, 기본 1).
  랩마다 라인이 얼마나 흔들리는지 비교용.
- 랩을 통과하면(스타트라인 크로스) 현재 궤적이 고스트로 넘어가고 새로 그린다.
- `2D Pose Estimate`로 위치를 다시 찍는 등 **자세가 1 m 이상 튀면** 궤적을
  잇지 않고 끊는다 (맵을 가로지르는 가짜 직선 방지).
- 색 기준은 `trail/color_mode`:

  | 값 | 의미 |
  |---|---|
  | `speed` (기본) | 실제 속도 viridis. 스케일 상한은 `trail/v_max`, 0이면 레이스라인 최대 vx에 자동으로 맞춘다 |
  | `cte` | 초록=레이스라인 위, 빨강=`trail/cte_max`(기본 0.5 m)만큼 벗어남 |
  | `speed_error` | 초록=계획대로, 빨강=계획보다 느림 |

`RacelineDeviation` (`/monitor/deviation_markers`) — 궤적의 각 점과 **같은 s의
레이스라인 점**을 잇는 사다리. 살이 길고 붉을수록 많이 벗어난 구간이다.
화면이 지저분해서 **기본 off**, 필요할 때 RViz에서 체크. `deviation/stride`로
밀도 조절.

## 구간별 속도 기록 (CSV)

트랙을 s 방향 `segment/length_m`(기본 1 m) 구간으로 잘라 실제 속도와
레이스라인 목표 속도를 비교 기록한다. 현재 구간은 `SegmentInfo` 오버레이에
실시간으로 뜬다:

```
Seg 19 [19.00-20.00m] | v 6.46 / plan 5.95 (+0.51) m/s | d 0.23m
```

랩을 완주할 때마다 `segment/log_dir`(기본 `~/monitor_logs`) 아래
실행 시각 폴더에 CSV가 쌓인다:

```
~/monitor_logs/20260728_145649/
├── laps.csv                 # 랩별 요약 (랩타임, 평균/최대 CTE, 평균속도, 최악 구간)
├── lap_001_segments.csv
└── lap_002_segments.csv
```

`lap_XXX_segments.csv` 열:

| 열 | 내용 |
|---|---|
| `seg_idx`, `s_start_m`, `s_end_m` | 구간 위치 |
| `samples`, `dt_s` | 샘플 수, 구간 통과 소요시간(다음 구간 진입까지) |
| `v_mean_mps`, `v_max_mps`, `v_min_mps` | 실제 속도 |
| `v_plan_mean_mps`, `v_err_mean_mps` | 레이스라인 목표 속도와 그 차이(실제−계획) |
| `cte_mean_m`, `cte_max_m` | 그 구간의 횡방향 편차 |

`laps.csv`의 `worst_seg_idx`는 목표 대비 가장 느렸던 구간 — 어디서 시간을
잃었는지 바로 찍힌다. 랩 0(출발점→스타트라인 워밍업)은 기록하지 않는다.
`/global_waypoints`가 없으면(플래너 미실행) 트랙 길이를 모르므로 구간 기록은
건너뛴다.

끄려면 `monitor/segment/log_enabled: false` 또는 `monitor/trail/enabled: false`.

## 장애물 표시는 빼 뒀다

인지 장애물(초록/빨강 원기둥)과 Static/Dynamic 텍스트 요약은 모니터에서
제거했다 — 화면이 지저분해지고, 이 모니터의 목적(주행 품질 계측)과 별개다.

발행자인 `perception_pkg/obstacle_viz_node` 자체는 살아 있으므로, 다시 보고
싶으면 RViz에서 MarkerArray 디스플레이를 추가하고 토픽을
`/perception/obstacle_markers` 로 잡으면 된다. 회피 라인(`AvoidanceSpline`,
`/planner/avoidance/markers`)은 그대로 남아 있다.

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
- **차량 위치/방향**: 텍스트가 아니라 `OdomHeadingReal/Sim` 화살표 (하늘색, 항상 on)
- **DrivenPath**: 실제 주행 궤적(속도 컬러맵) + 직전 랩 회색 고스트
- **RacelineDeviation**: 레이스라인 이탈량 사다리 (기본 off)
- **SegmentInfo**: 현재 s 구간의 실제속도 vs 레이스라인 목표속도

## 토픽 구독 (환경별 — config/environments/*.yaml)

| 용도 | sim | real_car / race_map |
|---|---|---|
| odom (속도) | `/ego_racecar/odom` | `/pf/pose/odom` (localization particle_filter) |
| 조향 입력 | `/drive` | `/ackermann_cmd` |

공통: `/car_state/frenet/odom` (랩/CTE/구간 s), `/global_waypoints`
(트랙 길이 + 구간별 목표속도 · 이탈량 기준선)

## 토픽 발행

`/monitor/lap_info`, `/monitor/fastest_lap_time`, `/monitor/mean_lap_time`,
`/monitor/current_cte`, `/monitor/max_cte`, `/monitor/mean_cte`,
`/monitor/segment_info` (OverlayText),
`/monitor/current_speed`, `/monitor/steering_input` (Float32),
`/monitor/driven_path`, `/monitor/deviation_markers` (MarkerArray, transient_local —
RViz를 나중에 켜도 지금까지의 궤적이 바로 뜬다)
