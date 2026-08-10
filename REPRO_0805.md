# 08-05 실차 주행 분석 & 심 재현

`rosbag_0805/` 3런(총 320초, 12랩 x 3)을 전 스택(하드웨어·컨트롤·퍼셉션·
로컬라이제이션·플래닝)에 걸쳐 분석하고, 그 주행을 심에서 재현할 수 있게 만든
결과다. 이 문서가 "그날 차가 실제로 무엇이었나"의 기준이다.

재현 실행:

```bash
ros2 launch integration_pkg stack.launch.py env:=sim_real_car rviz:=true
```

맵·레이스라인 기본값이 08-05 실차 것으로 잡혀 있어 인자를 더 줄 필요가 없다.

---

## 0. 먼저 — 이 저장소는 깨진 상태였다

분석 전에 고쳐야 했던 것들. 전부 "조용히 틀리는" 종류라 따로 적는다.

| # | 문제 | 증상 |
|---|---|---|
| 1 | `obstacle_detector_node.py`에 **미해결 merge conflict 마커** | 파일이 파싱조차 안 됨 (`SyntaxError: line 73`). 퍼셉션 노드가 아예 실행 불가 |
| 2 | `perception/real_car.yaml` **중복 키** | `max_cluster_radius_m: 0.40`이 뒤쪽 `0.60`에 조용히 덮임. YAML은 뒤 키가 이김 |
| 3 | `perception/sim_real_car.yaml`이 **노드가 선언 안 한 이름** 사용 | `map_filter_enabled` / `map_wall_dist_min_m` / `cluster_eps` → rclpy가 **에러 없이 무시**. 맵 벽 필터가 꺼진 채로 "켰다고 믿는" 상태 |
| 4 | `sim_odom_drift_node.py` 실행권한 없음 | `pf:=true` launch가 즉시 죽음 (`executable not found`) |
| 5 | `sim_ws`의 `sensor_publish_rate_hz` 소스 수정이 **빌드 안 됨** | 설치본은 `create_timer(0.004)` 하드코딩 → gym이 250 Hz로 계속 돌고 있었음 (아래 §3) |
| 6 | `planning_pkg/maps/` 소스가 비어 있는데 `CMakeLists`는 계속 설치 | install에 남은 건 예전 빌드 잔재라, clean 빌드하면 sim 맵이 전부 사라짐 → **맵을 `<ws>/maps` 한 곳으로 통합**(아래) |

⑥ 맵은 원래 두 곳(`<ws>/maps`, `planning_pkg/maps`)에서 순서대로 찾았다. 사본이
두 벌이면 같은 이름 맵이 어느 쪽이 쓰였는지 조용히 갈리는데, 이번 08-05 사고가
정확히 그 종류다. 이제 **`<ws>/maps` 한 곳만 본다**:

- `CMakeLists`의 `install(DIRECTORY maps ...)` 제거 — 맵은 패키지에 설치하지 않는다
- `sim_race` / `raceline_tuner` / `build_map` 이 전부 `<ws>/maps` 만 탐색
  (`sim_race`·`stack` launch에 `maps_dir:=` 로 대회 맵 폴더 지정 가능)
- `mapping.launch.py`의 이름-가림 검사에서 `planning_pkg/maps` 항목 제거
- HEAD에만 있던 `Spielberg_map` / `Budapest_map` / `my_track` 을 `<ws>/maps`로 이관

결과: **맵을 추가·수정할 때 재빌드가 필요 없다.** 빌드가 필요한 건 raceline·
centerline CSV 쪽뿐이다.

①은 같은 기능(맵 벽 필터)이 **두 벌** 들어 있어서 생긴 충돌이었다:

- upstream: `use_map_filter` / `cb_map` / `reject_wall_points`
- stashed: `map_filter_enabled` / `on_map` / `drop_wall_points`

upstream을 남기고 stashed를 지웠다. upstream만 갖고 있는 것들이 결정적이다 —
맵 프레임 검증, `awaiting_map` 발행 보류(fail-open으로 벽 파편을 흘리지 않음),
그리고 `/map` 구독 전용 `ReentrantCallbackGroup`(run_0803_213313에서 latched
`/map`을 78초간 못 받아 벽 필터가 통째로 죽은 기아 버그의 수정).

---

## 1. 08-05 실차가 실제로 무엇이었나 (bag 실측)

### 트랙 / 레이스라인

| 항목 | 값 |
|---|---|
| `/map` | 345 x 128 @ 0.05 m, origin (-8.101, -0.818) |
| `/global_waypoints` | 337점, s_max **33.57 m**, vx **[2.73, 5.94] m/s** |
| 코리도 폭 (d_left+d_right) | [1.115, 2.665] m, 평균 1.673 |
| `/tf_static` | `base_link -> laser` (0.27, 0, 0.11) |

> **⚠️ 이 맵은 저장소에 없었다.** 커밋된 `my_track0805`는 336x129 /
> origin (-9.297, -0.737)로 **x가 ~1.5 m 어긋난 다른 트랙**이다. 레이스라인도
> 337점 vs 334점으로 다른 생성물이다. 즉 차 위에서 맵을 다시 뽑고 그대로
> 달린 뒤 커밋하지 않았다.

### 주행 (3런 공통)

| 지표 | run_225904 | run_231051 | run_232916 |
|---|---|---|---|
| 명령 속도 max / p95 / mean | 5.94 / 5.58 / 3.96 | 5.94 / 5.60 / 3.93 | 5.94 / 5.58 / 4.04 |
| 달성 속도(`/odom`) max / p95 / mean | 5.28 / 5.01 / 3.48 | 5.28 / 5.01 / 3.35 | 5.42 / 5.04 / 3.51 |
| 조향 max | 0.340 rad (19.5°) | 0.348 rad (19.9°) | 0.307 rad (17.6°) |
| 횡오차 p50 / p95 / max | 0.073 / 0.316 / 0.531 | 0.078 / 0.437 / 0.704 | 0.078 / 0.272 / 0.609 |
| e-stop 활성 | **3.0%** (114회) | 0% | 0% |

- **명령을 못 따라간다.** 5.94 명령 → 5.28 달성 (~11% 미달). 종방향 plant
  한계(`A_ACC_MAX 2.2`, 30 A 전류포화)와 정합.
- 조향은 캘리브 한계 20.7°에 거의 닿았다 (`TUNING_STATUS.md` §4 — 손대지 말 것).
- 랩: **fastest 9.201 s, mean 21.021 s, mean CTE 0.215 m** (monitor_node).

### 로그가 말해주는 것 (`/rosout`)

| 메시지 | 횟수 | 의미 |
|---|---|---|
| `velocity_scale 1.083 exceeds certified maximum 1.000` | ~103/런 | 런타임에 1.083 요청 → **1.0으로 clamp됨**. 실효 스케일은 1.0이고 5.94는 CSV 자체의 상한이다 |
| `iters per sec: 23, possible: 33` | 955 | PF가 23 Hz로 돌았다 (CPU 제한) |
| `MAP steering rate clipped` | 43 | 조향률 리미터 포화 |
| `Rejected PF scan update: correction 0.31m ... exceeds guard` | 7 | PF 스캔 보정 거부, odom 추정 유지 |
| `TF is not close enough to the LaserScan source stamp (0.315s > 0.300s)` | — | 퍼셉션이 TF 지연으로 스캔을 버림 |
| `Frenet stale: ... local publish stopped` | 2 | 플래너 일시 정지 |

### 퍼셉션 — 이번 주행엔 **장애물 인지가 꺼져 있었다**

`/perception/obstacles` 메시지 **4923개가 전부 빈 배열**이다. 3런 320초 동안
장애물이 한 번도 안 잡혔다. 원인은 버그가 아니라 의도된 설정이다:

```yaml
min_cluster_size: 100000   # perception/real_car.yaml
```

스캔 빔 수(~1080)보다 큰 값이라 클러스터가 절대 안 생긴다. 노드를 죽이지 않고
이렇게 막는 이유는 spliner가 장애물 스트림이 `obs_timeout_sec`(0.5s) 끊기면
`perception_stale` → fail-safe 정지를 내기 때문이다. **빈 스트림은 계속 흘려야
한다.**

즉 **08-05 주행은 회피가 한 번도 개입하지 않은 순수 레이스라인 추종**이다.
코드 기본값(`min_cluster_size: 5`)만 보고 "회피가 돌고 있었다"고 읽으면 틀린다.

또한 퍼셉션은 **15.6 Hz**로 돌았다 — `/scan` 39.9 Hz의 약 1/2.5. 스캔 2.5개당
1개만 처리한 셈이다.

### IMU 단위가 ROS 규약을 어긴다

`/sensors/imu/raw`(`sensor_msgs/Imu`)는 SI 단위여야 하는데:

| 필드 | 실측 | 실제 단위 |
|---|---|---|
| `linear_acceleration.z` | 평균 **1.0028** | **g** (m/s²가 아님) |
| `angular_velocity.z` | p95 **103** | **deg/s** (rad/s가 아님) |
| `header.frame_id` | `''` | 비어 있음 |

`vesc.yaml`의 `imu_angular_velocity_scale: 0.01745`(deg→rad)가 **자이로만**
보정한다. 가속도는 아무도 안 고친다. 지금은 가속도를 쓰는 소비자가 없어서
문제가 안 되지만, bag을 분석할 때 |a_lat| p95가 0.73으로 보이면 그건
**0.73 g = 7.2 m/s²**다. mu 0.75 가정과 정합한다.

---

## 2. 코드 ↔ 실차 차이와 그 처리

| 항목 | 실차 08-05 | sim_real_car (기존) | 조치 |
|---|---|---|---|
| 맵 | bag 안에만 존재 | `my_track` | bag에서 추출 → `my_track0805_asrun` |
| 레이스라인 | 337점, s 33.57 | 334점, x 1.5 m 어긋남 | bag에서 추출 |
| `velocity_scale` | **1.0** | 0.742 | → 1.0 (sim이 26% 느렸다) |
| control `max_speed_mps` | 6.0 | 4.0 | → 6.0 (5.94 프로파일 상단이 잘렸다) |
| gym `/scan` | 39.9 Hz | **250 Hz** | → 40 Hz (`sim_ws` 재빌드) |
| gym `/tf` | 77.6 Hz | **2100 Hz** | → ~40 Hz |
| `bridge/use_map_filter` | true | false | → true |
| `bridge/wall_dist_min_m` | 0.15 | 0.20 | → 0.15 |
| `bridge/max_source_age_sec` | 0.40 | 0.20 | → 0.40 |
| 장애물 인지 | **OFF** | ON | → OFF (실차와 동일) |

`/scan` 250 Hz는 단순한 "빠름"이 아니었다. `/tf`가 레그당 250 Hz로 쏟아지면
구독자의 tf 버퍼(depth 100)가 0.2초 만에 포화해 `obstacle_detector`가 스캔을
통째로 버린다 — 실차 bag에도 같은 경고가 찍혀 있던 그 실패 모드다.

---

## 3. 재현 결과 (sim 90초 vs 실차 121초)

| 지표 | REAL run_225904 | SIM sim_real_car |
|---|---|---|
| 명령 속도 max | 5.94 | **5.94** |
| 명령 속도 p95 | 5.58 | 5.62 |
| 달성 속도 max / p95 | 5.28 / 5.01 | 5.63 / 5.44 |
| 조향 max / p95 [rad] | 0.340 / 0.266 | 0.255 / 0.216 |
| 조향률 p95 [rad/s] | 1.27 | 0.73 |
| frenet s 범위 | [0.0, **33.7**] | [0.0, **33.7**] |
| 랩 수 | 12 | 12 |
| 횡오차 p50 / p95 / max | 0.073 / 0.316 / 0.531 | 0.055 / 0.232 / 0.307 |
| 장애물 검출 | 0 / 1901 msgs | 0 / 1551 msgs |

s 범위와 랩 수가 정확히 일치한다 = 같은 트랙, 같은 레이스라인이다. 명령
프로파일 상단(5.94)도 정확히 재현된다.

**남은 격차는 plant(차 자체)다:**

- sim이 명령 속도를 더 잘 따라간다 (미달 5% vs 실차 11%). gym에는 모터 전류
  포화·구동계 손실이 없다.
- sim이 25~40% 더 정확하게 추종한다 (횡오차 p95 0.232 vs 0.316). 서보 지연·
  타이어 모델 불일치가 없다.
- 실차 조향률에는 max 23.8 rad/s 같은 스파이크가 있다 (p95는 1.27). sim은
  컨트롤러 리미트 2.0에 깔끔히 물린다.

즉 **"플래너·컨트롤러가 무엇을 명령하는가"는 재현되고, "차가 그걸 얼마나
못 따라가는가"는 sim이 낙관적**이다. 튜닝 판단을 sim만으로 하면 실차에서
더 크게 벌어진다고 보면 된다.

### 파라미터로 못 맞추는 잔차

| 항목 | 실차 | sim |
|---|---|---|
| 조향 LUT | `NUC6_glc_pacejka` | `SIM_linear` |
| `/odom` | 50 Hz (VESC) | 41 Hz (gym에 묶임) |
| PF | 27~31 Hz (CPU 제한) | 40 Hz |
| LiDAR 잡음 | Hokuyo 반사·결측·지면반사 | 빔당 1 cm 백색잡음 |
| 가감속 | 비대칭 (가속 2.2 / 감속 1.3) | gym `accl_constraints`는 대칭 |

감속 비대칭은 control `sim_real_car.yaml`의 `vehicle/max_decel_mps2: 1.3`이
명령단에서 대신 잡는다.

---

## 4. bag에서 맵·레이스라인 꺼내기

맵과 레이스라인이 차 위에서 생성되고 커밋되지 않는 일이 반복되므로 도구를
만들어 뒀다. bag의 `/map`과 `/global_waypoints`에 둘 다 통째로 들어 있다
(`WpntArray`가 build_map CSV의 9개 열을 전부 담는다).

```bash
python3 tools/bag_extract_run.py rosbag_0805/run_0805_225904_0.db3 \
    --name my_track0805_asrun
```

출력 (launch가 이름 하나로 찾아가는 규칙에 맞춰 나간다):

```
<ws>/maps/<name>.{pgm,yaml}                       # 워크스페이스 공용 맵 폴더
planning_pkg/config/global_trajectory/<name>_iqp_<suffix>.csv
planning_pkg/config/centerline/<name>_centerline.csv
```

맵은 패키지에 설치되지 않으므로 맵만 놓고 보면 재빌드가 필요 없다. CSV 두 개는
설치 대상이라 `colcon build` 후에 반영된다.

맵 왕복(OccupancyGrid → pgm/yaml → map_server 재해석)이 셀 단위로 무손실임을
확인했다. 세 bag의 맵은 동일하다.

주의 두 가지:

- **suffix의 margin/mu는 추정치다.** bag에는 생성 파라미터가 안 남는다
  (`WpntArray`는 결과만 담는다). 기본값은 같은 날 매니페스트에서 물려받았고,
  기하·속도는 bag 그대로라 주행 재현에는 영향이 없다. `tuner:=true`의 mu
  표시에만 쓰인다.
- **중앙선은 유도값이다.** bag에 중앙선이 없어 레이스라인 + 왼쪽법선 *
  `(d_left-d_right)/2`로 만들었다. 시각화 전용이라 이 근사로 충분하다.

---

## 5. 회피를 다시 켜려면

`perception/environments/sim_real_car.yaml`에서 이 한 줄만 지운다:

```yaml
min_cluster_size: 100000
```

`params.yaml` 기본값 5로 돌아간다. 그 순간 sim은 '08-05 재현'이 아니라
'회피 개발' 모드가 된다 — 실차는 그날 회피를 쓰지 않았기 때문이다.

벽/장애물 구별 A/B는 yaml을 고치지 말고 launch 인자로:

```bash
ros2 launch integration_pkg stack.launch.py env:=sim_real_car map_filter:=false
```

---

## 6. 남은 것 (이 작업에서 손대지 않음)

- **속도 캘리브(`speed_to_erpm_gain`)는 여전히 미완료**다 (`CLAUDE.md` /
  `TUNING_STATUS.md` §5 ⑤). 위 "달성 속도"는 전부 `/odom` = 같은 eRPM gain
  기반이라, 지면 대비 실제 속도는 외부 기준으로 재야 한다. 11% 미달은
  "명령 eRPM 대비 실제 eRPM"이지 "명령 m/s 대비 실제 m/s"가 아니다.
- IMU 가속도 단위(g) 정규화 — 지금은 소비자가 없어 무해하다.
