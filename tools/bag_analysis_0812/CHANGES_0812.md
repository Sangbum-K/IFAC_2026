# 0812 회피 실패 수정 — 파일별 정리

`run_0812_013630` 포렌식(1:15 / 2:20 / 3:14 관통, 2:37 회피 실패) 결과 반영.
mina `8fd816c`(회피 사이드 커밋 만료 + detector 스캔당 처리) 병합 포함.

**상태**: planning_pkg 빌드 OK · perception_pkg 빌드 OK · control_pkg 빌드 OK ·
`pytest src/planning_pkg/test` **43/43 통과** · 커밋 안 함(작업트리 13개 파일).

---

## 한눈에

| 파일 | 무엇 | 얻는 것 | 잃는 것 |
|---|---|---|---|
| `planning/spliner_node.cpp` | 사이드 잠금 양보 + 커밋 수명 + 조기 시작 탐색 | 관통 3건 원인 제거, 회피 속도 4.98→7.07 m/s | 가로지르기 사전필터 1개 제거 → 기하/충돌 검증에 전적 의존 |
| `planning/spliner_node.hpp` | `OtSideCommitFresh` 선언 + 1.0 s 상수 | 커밋이 영구 라치되지 않음 | 1.0 s 가 미실측 판단값 |
| `planning/local_trajectory_node.{cpp,hpp}` | 회피 상실 시 속도 홀드 0.5 s | 계획 자기무효화 차단, 계획 대비 손해 0 | 진짜 장애물 소멸 시 재가속 0.5 s 지연 |
| `planning/perception_obstacle_bridge_node.py` | 존재=`is_visible`, 분류 미확정=정적 | 검출→계획 0.246 s 단축, 실물 217건 회복 | 유령 억제가 `min_hits` 단독에 의존 |
| `planning/environments/{real_car,sim,sim_real_car}.yaml` | 그립 예산 5.0~5.5 → 7.85 | 회피 기하 여유 ~40%↑, escape 성립 | 실측 p99(8.76) 대비 마진 11%로 축소 |
| `control/control_node.cpp` + `params.yaml` | 조향포화 백스톱에 경로오차 AND | 회피 중 속도 캡 오발 제거 | 포화 단독 캡 없음 → 반 사이클 늦게 반응 |
| `perception/environments/{real_car,sim,sim_real_car}.yaml` | `process_rate_hz` 20 → 60 | `min_hits` 지연 0.19→0.075 s | CPU +5%p, 노이즈 억제 창 75 ms로 축소 |

---

## planning_ws — `src/spliner_node.cpp`

세 가지가 한 파일에 들어갑니다.

### ① 사이드 잠금 양보 (내 수정)

**문제.** "이미 왼쪽으로 나가 있으면 오른쪽으로 못 바꾼다"는 규칙이
`|d| > 0.20 m`로 "나가 있음"을 판정했습니다. 그런데 평상시 추종 오차만으로도
`|d| > 0.20`인 시간이 **42%** — 회피 의도가 아니라 흔들림에 걸립니다.

더 나쁜 것은 발동 위치입니다. `DoSpline`에서 이 거부는 **"잠긴 쪽이 막혔고
반대쪽은 성립한다"를 이미 확인한 뒤**에 실행됐습니다. 남는 선택이
"막힌 쪽 붙잡고 감속" 하나뿐 = 관통. 실측 7회 전부 직전 로그가
`OT side ... -> 반대쪽 폴백 (성립)`이었습니다.

**수정.**
- `DoSpline` — 거부(`return false`) 제거. 로그/카운터만 남김
  (`side_switch_crossed_while_committed`).
- `TryLaneSwitch` — 루프 **전에** `side_clears()`로 잠긴 쪽 성립 여부를 평가,
  안 되면 잠금 해제 (`lane_side_switch_yielded`).

| | |
|---|---|
| 얻는 것 | 1:15 · 2:20 관통의 직접 원인 제거. 반대쪽 성립을 확인한 뒤에만 전환. |
| 잃는 것 | 가로지르기 **사전** 필터가 없어짐. 0808 방어가 `BlendGeomFloor`(가로지르는 큰 진폭으로 곡률 계산) + `CheckPathCollision` 두 검증기에 전적으로 의존. 좌우 채터링 가능성. |
| 되돌리기 | 카운터 두 개로 발동 추적. 채터링이 보이면 `side_clears`에 히스테리시스 추가. |

`side_switch_d_tresh`는 **0.20 유지**. 진짜 회피 `|d|`(p25 0.28 / p50 0.31)와
정상 주행 `|d|`(p95 0.26 / max 0.32)가 겹쳐 분리 가능한 값이 없습니다.
0.35로 올리면 진짜 회피의 53%가 "회피 중 아님"이 되어 0808 가로지르기를
다시 켭니다.

### ② 조기 시작: 통과하는 최대 스팬 탐색 (내 수정)

**문제.** 회피를 일찍 시작하면 필요 곡률이 거리의 제곱으로 줄어 감속 없이
피할 수 있습니다(`v_max = L·√(a_lat/6A)` — L은 선형, a_lat은 √). 그런데 코드가
**가장 긴 스팬 하나만 시험하고 실패하면 통째로 포기**했습니다.
`BlendKappaMax`가 구간 **최댓값**을 돌려주므로 늘린 스팬 끝에 코너가 하나만
걸리면 전부 취소 — 곡률 0.01이면 예산 99% 남는데 0.75면 29%만 남습니다.
(당신이 지적한 "일찍 시작했다가 코너에서 자기모순" 의 정체가 이것입니다.)

**수정.** 긴 쪽 → 짧은 쪽으로 내려가며 통과하는 첫 스팬을 채택.
코너가 앞에 있으면 그 앞까지만 늘어납니다. 게이트 기준은 불변, 후보만 증가.

| | |
|---|---|
| 얻는 것 | 같은 그립에서 회피 통과 속도 4.98 → **7.07 m/s**. 코너 근처에서도 부분 확장(`ok_early_switch_partial`). |
| 잃는 것 | 사이클당 최대 32회 반복(무시 가능). 부분 확장은 진폭이 작아 여유가 줄 수 있음 — 카운터로 전체/부분 구분 가능. |
| 남은 것 | 벽 게이트 `room >= |d_hold|`가 구조적으로 과보수(램프 구간의 최소 벽거리를 최대 |d|와 비교). `TODO(후속)`으로 코드에 명시. |

### 미나 — 커밋 수명 `OtSideCommitFresh()` (병합)

**문제.** `CommitOtSide`가 세팅만 하고 **지우는 코드가 없었습니다.**
첫 회피에서 박힌 `last_ot_side_`가 주행 끝까지 남습니다.

**수정.** 1.0 s 수명. `TryLaneSwitch` · `DoSpline` 가드와 `SideKeepActive`
진입에 AND. 그리고 **스탬프 갱신을 `isfinite(s_center)` 분기 밖으로** 빼냄 —
안 빼면 s가 NaN인 프레임에서 진행 중인 회피가 만료돼 보입니다(진짜 버그).

**내 수정과 축이 다릅니다.** 프레임 단위 재현(30 Hz, 7,597 사이클):

```
|d| > 0.20 (옛 가드 켜짐)          3,206  42.2%
  미나 수명 통과 후 켜짐            2,226  29.3%   ← 980 해제
  남은 것 중 장애물 있음            1,540  69%     ← 사고 구간

장애물 있을 때 커밋 age:  p50 0.00  p90 0.00  ≤1.0s 98.6%
장애물 없을 때 커밋 age:  p50 0.43  p90 4.46  ≤1.0s 65.2%
```

장애물 접근 53회 중 `|d|`가 0.20을 처음 넘는 순간에 커밋이 낡아 있던 건
**3회**(115.7 / 143.5 / 152.1 s)뿐이고 **관통 사고 4건은 전부 age = 0.00**.
즉 미나 수정은 **진입할 때의 첫 사이드 결정**을 고치고, 내 수정은
**회피 도중의 코스 수정**을 고칩니다. 둘 다 필요합니다.

| | |
|---|---|
| 얻는 것 | 영구 라치 제거. 새 장애물의 첫 사이드 판단이 옛 커밋에 오염되지 않음(3/53 회복). |
| 잃는 것 | 1.0 s는 `obs_timeout_sec(0.30)`의 3배라는 근거뿐 **미실측**. 인지 공백이 1 s를 넘으면 사이드 유지가 풀림 — 이후 재선택은 기하 검증에 의존. |

**병합 후 세 경로 전부 확인:**
- 낡음 → 잠금 없음 → 양쪽 후보 시도 ✓
- 신선 + 잠긴 쪽 성립 → 전환 차단 (**0808 방어 유지**) ✓
- 신선 + 잠긴 쪽 막힘 → 양보 → 반대쪽 허용 ✓

### 손대지 않은 것 (검토 결과)

- **`mid_pass` (976행)** — 같은 `|d| > 0.20`을 쓰지만 목적이 반대입니다. 오탐이
  회피를 **막지 않고 허용**하는 fail-open이고, 주석대로 동적 상대에 대한 유일한
  방어입니다. 수명 게이트를 붙이면 추월 중 공격 모드가 끊깁니다 → 그대로 둠.
- **③④ 진폭 완화 / 속도 캡** — [spliner_node.cpp:1778-1848](../../planning_ws/src/planning_pkg/src/spliner_node.cpp#L1778)에
  **이미 구현돼 있었습니다** (`d_hold_min` 완화, `entry_speed_cap`, 진행분
  크레딧). 중복 코드 넣지 않음.

---

## planning_ws — `include/planning_pkg/spliner_node.hpp`

`OtSideCommitFresh()` 선언 + `kOtSideCommitValidSec{1.0}` 상수. 두 축이
겹치지 않는다는 실측 근거를 주석에 남김.

얻는 것: 없음(선언). 잃는 것: 없음. 다만 1.0 s는 다음 백에서 확정 대상.

---

## planning_ws — `src/local_trajectory_node.cpp` + `include/.../local_trajectory_node.hpp`

**문제 (3:14).** 2단계 실패였습니다. **1단계** — 회피 계획이 "이 속도면 된다"로
만들어졌는데 그 계획대로 가속하니 다음 사이클에 같은 회피가 불가능해졌습니다
(t=194.63, 자기무효화). **2단계** — 그 직후 인지가 0.25 s 끊겨 복구 불가.
※ 앞서 "드롭아웃이 직접 원인"이라 했던 건 순서가 반대였습니다. 자기무효화가
먼저입니다.

**수정.** `otwpnts`가 비면 직전 회피 경로 선두 속도(앞 5점 최솟값)를
`local_trajectory/avoidance_loss_hold_sec`(0.5) 동안 상한으로 유지.
`avoidance_loss_hold` 텔레메트리 발행.

| | |
|---|---|
| 얻는 것 | **감속이 아닙니다** — 계획이 설계된 속도를 유지할 뿐이라 계획 대비 손해 0. "더 밟지 말라"는 것뿐. 자기무효화 루프 차단. |
| 잃는 것 | 장애물이 **진짜로** 사라진 경우에도 0.5 s 동안 재가속 못 함. 직선 복귀 시 최대 0.5 s 손실. |
| 되돌리기 | `avoidance_loss_hold_sec: 0.0` = 옛 fail-open. |

---

## planning_ws — `scripts/planner/perception_obstacle_bridge_node.py`

**문제.** `confidence`는 존재 확률이 아닙니다 — `static_streak / n_release` =
**"정적인지 판단이 얼마나 진행됐나"**. 그런데 브리지가 `confidence < 0.2`면
장애물을 **버렸습니다.** 라이다가 실제로 보고 있는 물체를 분류가 안 끝났다는
이유로 지운 것입니다.

**수정.**
- `is_visible`이면 통과 — 존재는 측정이 증명합니다.
- `min_confidence`는 **외삽(coasting) 트랙에만** — 유령 제거 역할 유지.
- 분류 미확정은 **삭제 대신 정적 취급** — 정적/동적 오분류 방지 역할 유지.
- 스위치: `bridge/visible_bypasses_confidence` (기본 True).

| | |
|---|---|
| 얻는 것 | 검출→계획 지연 **0.246 s** 단축(`min_confidence 0.2 × n_release_sec 1.23`). 실물 217건 회복. 코스팅 처리·오분류 방지 둘 다 종전과 동일. |
| 잃는 것 | measured 트랙에 confidence 백스톱이 안 걸림 → **유령 억제가 `min_hits` 단독**. 분류 미확정을 정적으로 보므로 동적 상대를 잠깐 정적으로 판단(회피 방향이 보수적으로 감). |

---

## planning_ws — `config/environments/{real_car,sim,sim_real_car}.yaml`

**문제.** 당신 가설 — "타이어가 더 돌아갈 수 있는데 안전조건이 세게 걸려 시도도
못 한다" — 이 데이터로 확인됐습니다. 실측 횡가속도 p99 **8.76 m/s²**,
**9.5까지 슬립 포화 신호 없음**(조향 응답비 R로 확인). 조향은 한 번도 제약이
아니었습니다 — 최대 20.50° / 21.07° 사용, 사고 구간에서 필요했던 건 6~12°.
그런데 회피 예산은 5.0~5.5. 약 **40% 과보수**.

**수정 (세 프로파일 동일).**

| 파라미터 | 이전 | 이후 |
|---|---|---|
| `spliner/escape_arc_lat_acc` | 5.5 | **7.85** |
| `spliner/blend_lat_acc_max` | 5.0 | **7.85** |
| `spliner/escape_arc_max_heading_rad` | 0.70 | **0.80** |

`path_lat_acc_max`는 7.85 유지 — 문서상 물리 한계 mu 0.90 × g = 8.83이고
올리려면 `build_map` MU_PROFILE → manifest → **궤적 재생성**까지 필요.

| | |
|---|---|
| 얻는 것 | 회피 기하 여유 ~40%↑. `escape` 기동이 처음으로 성립 가능. `blend_use_spare_grip`의 잔여 그립(`√(path²−lat_ref²)`)도 함께 커짐. |
| 잃는 것 | 실측 p99(8.76)와 예산(7.85) 사이 마진이 **11%**로 축소. 노면·타이어 상태 변화에 민감해짐. |
| 감시 | 회피 중 `/pitwall/lat_err` < 0.35 m. 넘으면 예산이 아니라 추종이 한계. |

세 프로파일을 다 맞춘 이유: `test_sim_profiles_mirror_real_car`가 표류를
잡습니다(실제로 잡혔습니다).

---

## control_ws — `src/control_node.cpp` + `config/params.yaml`

**문제.** 백스톱(속도 캡)이 두 조건 **OR**였습니다: ① 경로오차 큼,
② **조향 포화**. ②가 문제입니다 — 조향 포화는 "지금 세게 돌린다"는 뜻이고
회피할 때는 당연히 세게 돌립니다. 회피 진입 순간 속도가 깎였습니다.

**수정.** `map/backstop_sat_min_path_error_m: 0.12` 추가 → 포화 분기를
**AND**로 묶음. 경로오차 임계(0.25)보다 낮게 둬서 포화는 여전히 선행 지표.

| | |
|---|---|
| 얻는 것 | 회피 중 속도 캡 오발 제거. `escape_now` 면제와 중복되지 않는 일반 경로에도 적용. |
| 잃는 것 | 조향 포화 **단독**으로는 더 이상 캡 안 걸림 → 경로오차가 0.12에 도달할 때까지 반 사이클 늦게 반응. |
| 되돌리기 | `0.0` = 옛 OR 동작. `>= backstop_path_error_m`은 검증에서 거부. |

---

## perception_ws — `config/environments/{real_car,sim_real_car}.yaml` (미나)

`process_rate_hz: 20.0 → 60.0` = 스로틀 사실상 해제, 스캔당 1회(~40 Hz).

**근거를 코드로 확인했습니다:** 초 단위 창은 `refresh_frame_windows()`
([obstacle_detector_node.cpp:601](../../perception_ws/src/perception_pkg/src/obstacle_detector_node.cpp#L601))이
실측 EMA로 프레임 수를 재환산하므로 Hz가 바뀌어도 벽시계 창이 보존됩니다.
반면 **`min_hits`는 의도적으로 프레임 도메인**
([:251](../../perception_ws/src/perception_pkg/src/obstacle_detector_node.cpp#L251))이라
Hz를 올리면 그대로 빨라집니다. 미나가 이걸 이용했습니다.
`min_period = 1/60 = 16.7 ms < 스캔 최단 20 ms`도
[:680](../../perception_ws/src/perception_pkg/src/obstacle_detector_node.cpp#L680)에서
성립 — 40.0은 25 ms라 부족합니다.

| | |
|---|---|
| 얻는 것 | `min_hits(3)` 벽시계 지연 0.19 → **0.075 s**. 6.5 m/s에서 1.2 m → 0.5 m. |
| 잃는 것 | CPU 7.3% → ~12%(1코어). **노이즈 억제 창이 75 ms로 축소.** |

## perception_ws — `config/environments/sim.yaml` (병합 보완)

미나 커밋이 `real_car` · `sim_real_car`만 바꿔서 `sim.yaml`이 20.0에 남았고,
`test_sim_profiles_mirror_real_car`가 실패했습니다. 60.0으로 동기화 →
43/43 통과. (미나 커밋의 "기존 실패 2건"이 이 계열로 보입니다 — 이 미러 테스트는
내 브랜치 `2d1c653`에서 들어왔습니다.)

---

## 결합 위험 — 이것만은 봐야 합니다

`min_hits` 창 축소(미나)와 confidence 백스톱 우회(내 ⑥)가 **같은 방향**으로
작용합니다. 결합하면 유령 억제가 **75 ms 안의 3연속 프레임 하나**에만 걸립니다.
게다가 ⑥은 분류 미확정을 정적으로 처리하므로 유령이 **회랑을 막는** 형태로
나옵니다 — 0811에서 겪은 그 증상입니다.

지연 예산은 더해집니다(겹치지 않음 — `n_release_`가 초 단위로 재환산되므로):

| | 절약 | 부족분 0.28 s 대비 |
|---|---|---|
| 미나 (`min_hits`) | 0.115 s | 41% |
| 내 ⑥ (confidence) | 0.246 s | 88% |
| 합계 | **0.361 s** | 넘김 |

**물리면 대응:** `min_hits: 3 → 5` (0.125 s @40 Hz — 옛 0.19 s보다 여전히 빠름).
지연 이득 대부분을 지키면서 노이즈 억제를 복구합니다. 선제 적용은 안 했습니다 —
두 저자의 의도를 넘는 변경이라 실차에서 증상을 본 뒤 판단.

---

## 첫 주행 감시 지표

| 지표 | 기대 |
|---|---|
| `/pitwall/lat_err` (회피 중) | < 0.35 m — 넘으면 그립 예산이 아니라 추종이 한계 |
| `reject_escape_geom_floor` | 감소, `escape_active` 가 처음으로 true |
| `ok_early_switch` / `_partial` | 증가. `_partial` 비중이 곧 코너 근접도 |
| `lane_side_switch_yielded` / `side_switch_crossed_while_committed` | 발동하되 **좌우 채터링 없이** |
| `avoidance_loss_hold` | 인지 공백에서 engage |
| 유령 정적 장애물 | 회랑 차단 발생 시 `min_hits` 상향 |

## 미해결 / 후속

- `kOtSideCommitValidSec 1.0` — 미실측. 회피 중 경로생성 공백 실측으로 확정.
- `room >= |d_hold|` 벽 게이트 구조적 과보수 — 코드에 `TODO(후속)`.
- `path_lat_acc_max` → mu 0.85(8.34) — 궤적 재생성 필요.
- ⑨ 트레일링 프로파일이 한 사이클만 생존 — 미확인, ③ 이후 재측정.
- 조향 중립 런간 ±2.5° 튐 — 기계 슬립, 소프트 보정 금지 (`TUNING_STATUS.md` §11).
