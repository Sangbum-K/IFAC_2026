# run_0818 회피 수정 — 진행 상황

분석 대조본: https://claude.ai/code/artifact/a5cb0af0-328a-4a57-bde7-41e80065f573

두 백 모두 직접 재분석함. 리포트 = `run_0818_205917` (179.9 s, 20:59),
내 분석 = `run_0818_232744` (62.9 s, 23:27). 같은 날 다른 빌드.

## 상태 (2026-08-19 최종)

**코드 항목 전부 완료.** 클린 빌드 green, planning 테스트 **52개 전부 통과**.
남은 것은 `#0`(main 머지 — PR 필요, 코드 아님)과 `#12`(버림)뿐이다.

⚠️ **아직 실행 검증(백 재생 A/B)은 안 했다.** 정적 경로는 전부
`if (obs.is_static)` / `else` 가지로 갈라 구조적으로 불변이고 소스 불변식
테스트로 못 박았지만, 실제 거동은 재생으로 확인해야 한다. 특히:
- **#11(배율 1.25)** — a_lat p90 이 포락선(8.76)의 86~105% 라 여유 없음
- **#7(잔여 마진)** — 회랑이 좁아짐, `lane_wall_no_room` 이 1위 기각 사유
이 둘은 A/B 전 실차 금지.

## 빌드

```bash
source /opt/ros/humble/setup.zsh
cd planning_ws   && colcon build --base-paths src --packages-select f1tenth_planning_msgs f1tenth_perception_msgs planning_pkg
source planning_ws/install/setup.zsh
cd perception_ws && colcon build --base-paths src --packages-select perception_pkg --cmake-args -DBUILD_TESTING=OFF
```

⚠️ `perception_pkg` 테스트는 `nlohmann_json` 이 없어 빌드 불가
(`CMakeLists.txt:91`, `if(BUILD_TESTING)` 안). `sudo apt install nlohmann-json3-dev`
필요. 그때까지 `-DBUILD_TESTING=OFF` 로 컴파일만 검증 중.

## 완료

### #1 성공/거부 카운터 분리 ✅ 빌드 green
`NoteOutcome()` 신설 → `/pitwall/outcome_*` 로 분리.
- `spliner_node.hpp:453` 선언 + `outcome_counts_` 멤버
- `spliner_node.cpp` `NoteOutcome()` 정의, `PublishRejectCounts()` 에서 발행
- 전환된 6곳: `ok_lane_switch` `ok_free_spline` `ok_escape_arc` `trailing`
  `cluster_merge_blocks_track` `ok_early_switch(_partial)`
- trace 마지막 필드에는 `+` 접두사로 남김 (파서 호환 유지)

### #2 vs 순간이동 차단 ✅ 빌드 green
- `perception_obstacle_bridge_node.py` — 클립을 LPF **앞**으로, 그리고 클립이
  아니라 **기각**(직전 속도 유지, `vel_held` 플래그로 시드 경로도 건너뜀)
- `config/environments/real_car.yaml` — `bridge/max_abs_vs_mps: 10.0 → 4.0`

### #5 분류 창 시간화 ✅ 빌드 green  ← 효과 최대
프레임 카운터 → 누적 시간. 실측률(33%)과 무관하게 계약이 지켜짐.
- `tracker.hpp` — `dynamic_streak/static_streak(int)` → `dynamic_time/static_time(double)`,
  `pos_hist_t` (측정 시각 deque) 추가, `pos_hist_len` → `pos_hist_sec`,
  `last_meas_dt` 추가, `Retired` 도 동일, `set_pos_hist_len` → `set_pos_hist_sec`
- `tracker.cpp` — 생성자/update/reacquire 에서 두 deque 짝 유지, 시간으로 prune,
  `last_meas_dt` 는 `last_measurement_stamp` 덮기 **전에** 계산
- `classifier.{hpp,cpp}` — `n_confirm/n_release/dynamic_release(int frames)` →
  `t_confirm/t_release/t_dynamic_release(double sec)`. 측정 간격 상한 `kMaxMeasGapSec = 0.25`
- `obstacle_detector_node.cpp` — `refresh_frame_windows()` 에서 이 넷의 환산 제거
  (`track_timeout` 만 프레임 유지 — `misses` 와 비교하므로 그쪽은 맞음).
  `t_confirm_/t_release_/t_dynamic_release_/t_pos_hist_` 멤버 신설,
  `*_sec` 우선 / 없으면 옛 프레임값을 `kLegacyTuneHz(15.4)` 로 환산
- 테스트 갱신 + `ConfirmTimeIsIndependentOfMeasurementRate` 신규 (핵심 계약)

**예상 효과** (재현 모델): 이동 트랙 동적 확정 46%→72% (205917), 27%→82% (232744)
**오염 검증 결과**: 정지 트랙 → 동적 오분류 0/53, 0/20 (모든 설정에서)

### #7 멈춘 상대의 잔여 모션 마진 ✅ 빌드 green
`is_static` 이 서면 모션 전파가 완전히 꺼져 재출발이 안 보이는 문제.
- `ObsClsTrack.ever_dynamic` 신설 (메시지에 `ever_dynamic` 이 없어 spliner 가 자체 기억)
- `WasEverDynamic(id)` 헬퍼
- `PredictObstacleMovement()` — 한때 동적이던 정적 트랙만 s 범위를 유계 마진만큼 확장
- 파라미터 `spliner/ever_dynamic_residual_vs: 0.3` (정지 트랙 |vs| p90),
  `ever_dynamic_residual_max_m: 0.20` (0 이면 기능 off)
- ⚠️ A/B 시 `reject_lane_wall_no_room`(현 3.8~4.1/s) 증가 여부 반드시 확인

### #6 `by_vs` 지속 조건 ✅ 빌드 green
- `ObsClsTrack.vs_low_run` + `spliner/cls_vs_hold: 10`
- 문턱(`cls_vs_reset`)은 0.1 유지 — 올리는 안은 폐기(이동 트랙 26~36% 오분류)
- ⚠️ 이 경로는 `vs_hist >= kClsVsWindow(10)` 인 트랙만 탄다. run_0818 의 정지
  트랙은 관측 p50 1~2 개라 도달 못 함 — 진짜 멈춘 상대차용. **미검증**

### #3 회피상실 홀드 만료 ✅ 빌드 green — **진단 정정됨**
처음엔 "trailing 속도 프로파일 스킵 시 상한 해제"로 봤는데 **틀렸다.**
체인을 단계별로 재보니:

```
t       otwpnts선두  local_wpnts선두  ackermann명령  실측v
15.80      -            3.32            2.96        3.00
15.90     3.41          6.64            3.37        3.08   <- local 이 튐
16.10     3.41          6.72            4.50        3.37
16.20     3.79          3.03            4.78        3.81
```

**플래너는 3.41 을 발행했는데 `local_waypoints` 가 6.72(레이스라인 최고속)로 뛰었다.**
원인은 spliner 가 아니라 `local_trajectory_node` 의 회피상실 홀드 만료:
- t=15.33 마지막 트레일링 → 15.33~16.16 **perception 이 전방 8m 에 아무것도 못 봄**
  (0.86s — id 7 이 죽고 id 9 가 태어나기까지)
- 홀드가 15.38~15.83 동안 상한 3.32 로 눌러 주다가 **0.5s 초과로 만료**
- 15.86~16.16 상한 없음 → 6.72 통과 → 명령 3.37 → 4.78
- 16.19 id 9 가 0.96m 앞에서 등장. 최근접 0.15m

수정: `local_trajectory/avoidance_loss_hold_sec: 0.5 → 1.0`
기준을 "관측 분포의 배수"가 아니라 **검출기 계약**으로 바꿨다 — detector 가
`track_timeout_sec(0.80)` 동안 트랙을 살려 두므로 플래너가 먼저 풀면 안 된다.
공백 커버율 38~40% → 46~53%.
⚠️ 근본 해결은 이 값이 아니라 공백 자체(#5 실측률, #15 재등록).

### #9 gap 안쪽 ×0.85 감속 제거 (동적만) ✅ 빌드 green
`spliner_node.cpp` — `if (obs.is_static)` 로 감쌌다. 정적은 비트 동일.
정적 물체는 비켜주지 않으니 "시간을 버는 감속"이 성립하지만, 동적에는
정반대다(속도 우위·횡 예산이 줄고 상황은 안 바뀜).
`trailing_gap_m` 1.5→2.2 는 **철회** — 뒤로 물러나지 않을 거면 넓힐 이유가 없고
정지거리·비상정지 합성 장애물 위치까지 건드려 위험만 는다.

### #4 동적 트레일링 속도 바닥 3.0 m/s ✅ 빌드 green
`allow_stop` 의 `else` 가지에만 추가 — 정적 경로 구조적으로 불변.
- `spliner/dyn_trailing_floor_mps: 3.0`
- **해제 조건 2개 필수** (없으면 갇혔을 때 그대로 박음):
  ① `emergency_stop` 이면 안 걸림
  ② `geom_blocked`(lane_switch/free_spline 기하 하한 초과)면 해제

### #8 트레일링 감속도 ✅ 빌드 green — **범위 축소 + 값 정정**
**리팩터가 불필요했다.** `v_allow` 식
`rem >= v·T + (v²-v_target²)/2a` 이 **이미 "늦게 밟기"** 다 —
`engage = max(6.0, cur_vs*1.6+3.0)` 밖에서는 속도 캡이 **아예 없고**, 안에서만
이 곡선이 걸린다. 즉 2단계 계약을 새로 만들 필요가 없었고 바꿀 것은 `a` 하나.
→ `ComputeTargetsStatic/Dynamic` 분리는 **하지 않았다** (필요 없어짐).

⚠️ **값 정정**: 앞서 "실측 감속 p90 3.2, 그러니 2.5" 라고 했는데 **틀렸다.**
그 3.2 는 PF 속도 미분 노이즈였다. **감속 요구 구간만** 골라 속도대별로 재면:

| 속도 [m/s] | p50 | p90 |
|---|---|---|
| 0.5~1.5 | 0.81 | 1.10 |
| 1.5~2.5 | 1.06~1.10 | 1.34~1.75 |
| 2.5~3.5 | 1.33~1.52 | 2.03~2.11 |
| 3.5~5.0 | 1.55 | 2.17~2.58 |

감속은 **속도 의존**이고 2.5 는 어느 구간에서도 p90 이상이다 —
`trailing_decel` 주석의 run_0806 경고(2.5 로 정지점 0.8~1.3 m 오버슛)가 맞았다.
→ `spliner/trailing_decel_dyn: 1.5` (3~5 m/s 대 p50). 정적은 1.0 그대로.
   `decel_eff` 로 4곳(v_allow 2 + 램프충돌 정지곡선 2) 분기.
   효과: rem 5 m 에서 허용속도 3.03 → 3.57 (레이스라인 4.66 의 65% → 77%).

### #10 회피 실패해도 횡이동 유지 (동적) ✅ 빌드 green
08-18 에 `hold_retry_d` 를 정적/준정적으로 좁힌 근거는 "움직이는 상대에 0.7초
전 목표 d 를 붙잡으면 2 m/s 에서 1.4 m 어긋난다" 였고, **미뤄 둔 설계 질문**은
"상대가 움직이는 동안 목표 d 를 어떻게 갱신하나" 였다. 답을 넣고 확장:
- 목표는 **횡(d)** 이므로 상대의 종방향 이동은 목표를 어긋나게 하지 않는다.
  어긋남을 만드는 건 vd 뿐 → `d_goal = retry_d_target_ + retry_d_vd_ * age`
- `NoteRetryTarget(d, obs_vd)` 로 기록 시점 vd 를 함께 저장
- 창은 정적 `kRetryDHoldSec 0.70` / 동적 `kRetryDHoldDynSec 0.30` (vd 오차 누적 방지)
- 실측 |vd| p90 0.30 m/s → 0.30s 창이면 보정 후 잔여 오차 0.09 m (차 반폭 0.15 미만)
- 정적/준정적 경로는 vd 보정항이 없어 **비트 동일**

### #11 추월 속도 배율 1.0 → 1.25 (동적 전용) ✅ 빌드 green
- `active_speed_scale_ = (in_sector && ot_boost_ok) ? ot_speed_scale_ : 1.0`
  (`ot_boost_ok = !is_static || qs_active_`) — 정적은 항상 1.0
- 물리 캡(sqrt(a_lat/k)·조향 한계)이 그대로 걸려 직선에서만 살아난다
- ⚠️ A/B 필수: a_lat p90 이 포락선(8.76)의 86~105% 라 여유 없음

### #13 예측기 무출력 진단 로깅 ✅ (부활은 보류)
`opponent_predictor_node.py` — 게이트 실패 시 `ego_s/ego_vs/stamp_ns/age/
timeout/future_tol/require_stamp/use_sim_time` 을 함께 찍는다.
지연은 p50 18ms 로 멀쩡했으므로 다른 조건이 원인. **#12 를 버렸으므로
예측기 부활은 보류** — 살리면 전폭 튜브 71% 가 같이 돌아온다.

### #14 초대형 클러스터 분할 ✅ 빌드 green
- `clustering.{hpp,cpp}` — `split_oversized_cluster()` 신설.
  각도순 점열을 **최대 간격**에서 나누고 재귀(max_depth 4).
  조각이 `min_cluster_size` 미만이면 버린다 → 파편만 남지 않는다.
  하나도 안 남으면 종전대로 삭제(= 진짜 벽).
- `obstacle_detector_node.cpp` — erase 대신 분할, `cluster_split_enabled`(true)로 A/B
- 진단 카운터 `oversize_seen_/recovered_/dropped_` 누적을 로그에 같이 찍는다
  (기존 로그가 THROTTLE 이라 횟수를 셀 수 없었다)
- ⚠️ 유령 증가 확인 필수 — `wall_dist_min_m` 0.25→0.30 때 덩어리가 쪼개져
  게이트를 빠져나가며 유령이 **늘었던** 이력과 같은 방향의 위험

### #15 재등록 시 분류 이력 승계 ✅ 빌드 green
- `ClassifyObstacles` 진입부에서, 처음 보는 id 면 **방금 사라진 동적 트랙**의
  외삽 위치(`s_last + vs*age`)와 `cls_inherit_dist_m(0.6)` 안인지 찾아 승계
- `cls_inherit_timeout_sec(0.5)` 안에서만, `ever_dynamic` 트랙만 대상
  → 나란히 선 콘끼리 이력이 새는 것을 원천 차단 (정적은 대상 아님)

### #16 hold 길이 상대속도 연동 ✅ 빌드 green
**이미 구현돼 있었다** — `travel = v_obs * (reach / closing)`, `closing` 은
`qs_min_closing_mps` 로 하한, 결과는 `lookahead_m_` 로 캡. `qs_active_` 에만
걸려 있던 게이트를 `!closest_obs.is_static`(동적 전체)으로 넓혔다.
정적은 조건에서 제외해 비트 동일.

### #17 후순위 3건 — **계측 결과 2건은 변경 불필요로 종결** ✅

**ApexFor `min(target, balanced)` 되치기 → 변경하지 않음.**
리포트는 "남는 좌측 0.22~0.59 m 를 안 써서 탈출 각도 부족"이라 했는데,
`lane_wall_no_room` 로그를 직접 파싱해 보면 병목이 apex 목표가 아니다:

| | d_hold (벽 클램프 결과) | 필요 | 부족분 |
|---|---|---|---|
| 205917 (35건) | 0.17 | 0.52 | p50 0.46 m |
| 232744 (11건) | -0.21 | -0.79 | p50 0.58 m |

`d_hold` 는 이미 `std::clamp(..., 0, wall - boundary_margin)` 에 **걸린 값**이다.
부족분은 벽이 허락하지 않는 양이므로 apex 목표를 키워도 clamp 가 같은 값을 낸다.
오히려 `min` 을 풀면 차가 벽에 붙어 `lane_wall_no_room`(1위 기각 사유)이 악화된다.

**`escape_active` 전 구간 false → 결함 아님, 조건 미충족.**
`BuildEscapeArc` 첫 게이트가 `!closest_obs.is_static → reject` 다 —
**원호 탈출은 정적 전용**이고 동적에는 원리적으로 안 돈다(`:1461` 주석도 같은
말). 정적에서도 `dist_front ∈ (0.2, lookahead]` 와 `is_visible` 을 요구한다.
두 백의 기각 분포가 그대로 설명한다:
`escape_not_visible_static` 324/19 · `escape_dist_out_of_range` 138/15 ·
`escape_geom_floor` 38/3. 즉 "호출은 되는데 내부 게이트에서 떨어진다"이고,
그 게이트들은 각각 rosbag_0811 fail-open 방어와 "이미 옆에 들어옴" 판정이라
**손대면 안 되는 것들**이다. 232744 의 `escape_dist_out_of_range` 2.19/s 는
게이트 결함이 아니라 **늦은 검출의 증상**이다(#5/#14 소관).

**측위** — 별건 분리 유지. 검증 시 LOST 구간(232744 t>54.1) 제외.

### 마감 점검에서 추가로 고친 것

**#11 의 실제 누수** — `active_speed_scale_` 이 **매 사이클 리셋되지 않았다.**
장애물 분기 안에서만 대입되므로 장애물 없는 사이클에는 직전 값이 남는다.
1.0 일 때는 무해했지만 1.25 로 켜면 정적 경로·커밋 재생으로 새는 통로가 된다.
바로 옆 `qs_active_` 와 같은 자리에 리셋 추가. 소비처 전수 감사 결과:
- 빌더 5곳(3178/3683/3975/4768/4934)은 대입 **이후** → 해당 장애물 배율 ✓
- `BlendPlanSpeed`(6155) ← 커밋 홀드(1300)는 대입 **이전** → 이제 결정론적 1.0 ✓
- 커밋 재생(1336)은 `in_geom_ceiling` 로 기하 단계를 건너뜀 → 재부스트 없음 ✓

**내가 깼던 기존 불변식 2개** (전체 스위트를 돌려서 발견):
- `params.yaml` 기본은 fail-safe 여야 함 → `ot_speed_scale` 을 base 1.0 으로
  되돌리고 환경 오버레이(real_car/sim/sim_real_car)에서 1.25 를 켰다
- sim 은 real_car 를 미러링해야 함 → `max_abs_vs_mps 4.0`,
  `cluster_split_enabled` 를 sim 계열에도 반영

**분리 불변식 테스트 3개 신설** (`test_safety_invariants.py`, 소스 검사 방식):
`test_overtake_speed_scale_is_dynamic_only` /
`test_dynamic_only_trailing_relaxations_are_gated` /
`test_static_trailing_floor_is_not_lowered_by_dynamic_floor`

## 남은 항목

| # | 항목 | 비고 |
|---|---|---|
| 0 | `99b5389` main 머지 | **코드 아님 — PR 필요.** main 에 없는 실질 커밋 1개, 파일 3개 |
| 12 | 예측 튜브 ±1e3 | ❌ **버림** — 단 #13 예측기 부활도 같이 보류 |
| — | **백 재생 A/B** | 미실행. 아래 회귀 지표로 확인할 것 |
| — | `nlohmann-json3-dev` | perception 테스트 빌드 선행 조건 (#5·#14 회귀 테스트) |

## 정적 회귀 감시 지표 (매 변경마다)

| 지표 | 기준선 (205917 / 232744) | 허용 |
|---|---|---|
| 정적 대상 트레일링 발행속도 p50 | 2.52 / 2.50 | ±0.1 |
| 정적 대상 트레일링 ≤0.05 비율 | 2% / 0% | 증가 금지 |
| 정지 트랙 → 동적 오분류 | 0 / 0 | **0 유지** |
| 정적 회피 성공 건수 | — | 감소 금지 |

## 절대 건드리지 말 것

- `is_static` 을 준정적에서 true 로 뒤집기 → 모션 전파가 꺼짐 (`:1789`, `:1793`)
- `DoSpline` 을 동적에 열기 (`:1465`) — 명시적 결정
- 코스팅 프레임에서 정적 기구 열기 — `rosbag_0811` fail-open
- `wall_dist_min_m` 0.25 상향 — 0.30 에서 유령 **증가** 실측
- 정적 트레일링 바닥 2.5 — 지금 정확히 지켜지는 중, 회귀 감시 지표로 사용

## 분석 스크립트

`/tmp/claude-1000/-home-sangbum-IFAC-ws/f7a57802-535b-4ec3-b64f-aeac14c6b3b2/scratchpad/`
에 `cmp.py`(두 백 비교) `replay205917.py`(classifier 재현) `contam.py`(오염 검토)
`persist.py`(지속 조건) `stopgate2.py`(트레일링 속도) 등. 영구 보관하려면
`tools/bag_analysis_0818/` 로 옮길 것.
