# global_traj_sim

ROS 없이 `.venv`만으로 도는 global trajectory 생성 + 시뮬레이션 파이프라인.
`f1tenth_gym_ros`의 맵(yaml/png)에서 centerline을 추출해 `planner_pkg`의
`global_trajectory_node`가 읽는 것과 동일한 9열 세미콜론 CSV
(`s_m; x_m; y_m; d_right; d_left; psi_rad; kappa_radpm; vx_mps; ax_mps2`)를
생성하고, pure pursuit + kinematic bicycle 모델로 폐루프 추종 시뮬레이션을 돌린다.

---

## ⚡ 빠른 시작 — `build_map.py` (맵 하나 → CSV 원샷)

**맵으로 레이스라인 만드는 경로는 이거 하나다.** step1(트랙 추출)+step2(IQP
라인·속도)+step3(시뮬)+step4(저장)를 한 파일로 합친 것이라, 아래 step1~4 파일은
파이프라인 이해·수정용 참고 구현으로만 남아 있다.

> 전제(STEP 0): 맵 `.png`(또는 `.pgm`) + `.yaml` 을 `<workspace>/maps/` 에 미리
> 넣어둘 것. 인자로 주는 맵 이름은 `maps/<이름>.yaml` 의 **확장자 뺀 파일명**과
> 같아야 하고, yaml의 `image:` 필드도 **maps/ 안의 실제 파일**이어야 한다.

```bash
cd ~/IFAC_ws/planning_ws/src/planning_pkg/scripts/global_traj_sim
PYTHONNOUSERSITE=1 ~/IFAC_ws/.venv/bin/python build_map.py my_track
```

짧은 코너 뒤 직선성을 우선하는 종방향 편향 후보는 기본 IQP를 덮어쓰지 않고
별도 SP CSV로 만든다. `--raceline sp`를 주면 SP를 메인 후보로 생성·검증할 수
있으며, 자동 선택되지 않으므로 통과한 파일을 `trajectory_csv_file:=...`로
명시해야 한다. 작은 반경 때문에 코너 최저속과 제동 요구가 커지는 절충이 있어
기본값은 계속 `iqp`다.

```bash
PYTHONNOUSERSITE=1 ~/IFAC_ws/.venv/bin/python build_map.py my_track --raceline sp
```

**전체 절차(venv 준비·scikit-image 설치·빌드·RViz 튜너)와 이 기계 특유의 함정은
[`../../maps/README.md`](../../maps/README.md) 가 단일 기준 문서다.** 요약하면:

- `PYTHONNOUSERSITE=1` 없이 돌리면 `~/.local`의 numpy 2.2.6이 잡혀
  dist-packages scipy/matplotlib이 `_ARRAY_API not found`로 죽는다.
- `skimage`(medial-axis 중앙선)가 필요한데, pip으로 깔면 numpy 2.x를 딸려
  깔아서 같은 문제를 만든다 → 깐 뒤 venv의 numpy를 uninstall해서 되돌릴 것.

### 기본값은 소스에, 일회성 스윕은 CLI에 둔다

> 📄 **"CSV 속도가 왜 이 값인가"는 [SPEED_PARAMS.md](SPEED_PARAMS.md) 를 먼저 볼 것.**
> 어느 상수가 속도를 정하고 어느 게 아무 효과도 없는지, 실차 백 실측 근거와
> 게이트 스윕표까지 한 곳에 정리돼 있다. 오래된 표는 0808 기하의 기록이고,
> 현재 0821 기하에서는 `MU_PROFILE`도 실제로 랩타임을 움직인다.

**트랙 생성 파라미터**는 `build_map.py` 상단 `설정` 블록에 모여 있다
(`MU` 1.50, `MU_PROFILE` 1.40, `MARGIN` 0.08, `RACELINE` `"iqp"`, `MAKE_SP`
True, `SP_CURV_W` 12.0, `INFIELD_FILL_M` 0.8, `MIN_ISLAND_M2` 0.3 …).
기본 `mkcsv <맵>`은 이 값을 사용한다. `--mu-profile` 등은 스윕용이며 실제
사용값이 generation manifest에 남는다. 확정값은 상단 상수와 근거 주석으로
옮겨 다음 `mkcsv`에서도 재현되게 한다. 표와 의미는 maps/README.md 참고.

급코너 진입 감속은 절대 `s` 위치를 저장하지 않고 새 맵의 곡률에서 매번 다시
검출한다. 기본값은 peak `|kappa|>=0.50`인 코너에 한해 `|kappa|>=0.32`가 처음
보이는 진입점의 4 m 전부터 2 m 뒤까지 기존 완성 속도 프로파일의 `0.96`을
상한으로 적용한다. 그래서 급코너에서 먼 직선과 완만한 코너는 직접 제한하지
않는다. 일회성 비교는 `--sharp-entry-scale`, `--sharp-entry-preview`,
`--sharp-entry-hold`로 하고, 실제 적용 구간·속도 하락·예상 랩타임 증가는
generation manifest의 `validation.sharp_corner_entry`에 기록된다.

**라인-벽 이격은 이제 상수가 아니라 구간별로 다르다.** SLAM 트랙은 폭이
2배 넘게 변하는데(my_track 0.9~2.15 m) 균일 `SAFE_DIST` 하나로는 넓은 곳에서
여유를 버리고 좁은 곳에서는 최적화가 '정중앙 고정'으로 꺼진다. 지금은
`ADAPTIVE_SAFE_DIST=True`에서 각 지점의 코리도 폭 × `SAFE_DIST_WIDTH_FRAC`
(0.30)을 `[SAFE_DIST_MIN 0.22, SAFE_DIST_MAX 0.55]`로 자른 값을 쓰고, 차 전장
창으로 rolling-min 해서 좁은 게이트 앞에서 미리 완화한다. 좁혀서 얻은 여유는
`ADAPTIVE_SPEED_BY_WIDTH`가 속도로 갚는다 —
`v ≤ (벽여유 − 차 반폭) / lat_err_slope`. 이 기울기는 이제 단일 상수가 아니라
직전 곡률로 `LAT_ERR_STRAIGHT_S`(0.030) ↔ `LAT_ERR_CORNER_S`(0.065) 사이를
보간한 값이다 (옛 `LAT_ERR_PER_MPS` 는 없어졌다 — 자세한 건 SPEED_PARAMS.md).
`ADAPTIVE_SAFE_DIST=False`로 두면 예전 균일 `SAFE_DIST` 동작이다.

> **실차 첫 주행 팁:** 보수적으로 시작 → `V_MAX 5.0 / MU 0.6`. 잘 돌면
> V_MAX 올리고 MU 조여가며 반복.

**차량 상수와 배포 프로파일은 `../deployment_profiles.py`가 단일 기준이다.**
`build_map.py`에 베껴 적지 말 것. 프로파일 숫자(velocity_scale, control 속도
cap, MAP/L1 lookahead, steering LUT …)는 launch가 읽는 바로 그 YAML에서
읽는다. 2026-08-05 이전에는 build_map(0.97/6.0), tuner(0.83/5.0), 런타임
YAML(0.742/4.0) 세 벌이 전부 달랐고, CSV는 실차가 달리지 않는 조건으로
검증받고 있었다.

> `WHEELBASE`/`MAX_STEER`는 출력 CSV에 영향이 **없다** — 폐루프 시뮬에서
> "실차 조향 한계로 이 라인을 따라갈 수 있나"(lat err)를 보는 데만 쓰인다.
> control YAML의 `vehicle/wheelbase_m` 등이 이 실측 상수와 어긋나면 프로파일
> 해석 단계에서 죽는다(검증한 차 ≠ 달리는 차).
>
> 폐루프 검증은 `--profiles`로 고른 gate 프로파일을 **모두** 통과해야 CSV를
> 쓴다. 기본 `real`은 실제 NUC6 LUT를 쓰는 C++ real-car MAP/L1만 gate이고
> Python simulation tracker는 실행·보고만 한다(advisory). 예전처럼 둘 다
> gate로 하려면 `--profiles all`. 실차 CSV가 시뮬 프로파일 실패로 막히던
> 문제를 없애면서도 결과는 계속 보인다.
>
> 2026-08-21부터 실차 추종 시뮬의 footprint 예측은 advisory다. 0821-1은
> 모델 `|d|max≈0.63 m` 대 실측 `≈0.16 m`, 안정적이었던 0821-6.65도 모델이
> 크게 과대예측했기 때문이다. CSV 형식, 정적 차체-벽 footprint, 곡률,
> 조향/조향속도, plant, 마찰원은 계속 hard gate이고 Python sim profile의
> 폐루프 footprint도 hard gate다.
>
> 종방향 plant는 `ackermann_to_vesc`의 DRIVE/COAST/BRAKE 중재를 그대로
> 모델링한다(임계값은 `hardware_ws .../vesc.yaml`에서 읽음). 제동 전류
> 1 A당 감속(`BRAKE_DECEL_PER_AMP`)은 **아직 미실측이라 0**이므로 계획은
> 코스팅 감속만 쓴다 — 실측 후 올리면 프로파일이 다시 빨라진다.

### 끝나면

- 그림 : `outputs/<맵>_trajectory.png` — 라인/속도/시뮬 궤적 눈으로 확인
- CSV  : `outputs/` + `config/global_trajectory/` 두 곳에 저장 (후자를 튜너·런타임이 읽음)
- 회피 계층용 : `config/centerline/<맵>_centerline.csv`, `_walls.csv`
- 세트 표식 : `<맵>_generation_manifest.json` — main/SP/centerline/walls SHA-256

모든 파일을 먼저 stage+`fsync`한 뒤 SP, 보조 파일, main 순서로 승격하며
런타임 main CSV가 데이터 commit point다. 매니페스트는 그 뒤에 기록된다.
SP가 검증에 실패하거나 `MAKE_SP=False`이면 안전한 main은 계속 생성하되,
같은 suffix의 기존 SP는 commit 전에 숨겨 stale 추월선이 남지 않게 한다.
단, 여러 파일 rename 자체는 하나의 원자 연산이 아니다. 자동 SP 선택을 끈
현재 안전 기본값에서는 중간 상태가 SP를 활성화하지 않지만, SP 파일을
명시적으로 연결하는 런타임은 두 CSV 헤더의 `generation_id` 존재/일치와
매니페스트 SHA-256을 확인해 전원상실 중간 pair를 거부한다.

### `make_raceline.sh`

워크스페이스 위치를 자동으로 찾고 `PYTHONNOUSERSITE=1`로 `build_map.py`를
호출한다. `./make_raceline.sh <맵이름> [--dir cw|ccw|auto]`로 사용할 수 있다.
네트워크가 막힌 환경에서는 의존성을 미리 준비해야 한다.

다음 단계(튜너 편집 → 빌드 반영)는 [`../tuner/README.md`](../tuner/README.md) 참고.
런타임에 반영하려면 CSV 생성 후:
```bash
cd ~/IFAC_ws/planning_ws && colcon build --packages-select planning_pkg && source install/setup.bash
```

---

## (참고) 단계별 step 스크립트 — 구 파이프라인

> 이 기계에서 맵을 만들 땐 위의 `build_map.py`를 쓴다. 아래는 파이프라인을
> 단계별로 뜯어보거나 다른 환경으로 옮길 때의 참고용.

```bash
cd <workspace root>          # IFAC_ws-main
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt   # + scikit-image (build_map.py용)
.venv/bin/python planning_ws/src/planning_pkg/scripts/global_traj_sim/build_map.py Spielberg_map
```

구형 `step4_main.py`는 48fe25a 에서 삭제됐다. caabf06 이 오래된 브랜치에서
되살려 왔길래 2026-08-11 에 다시 지웠다 (같은 커밋이 되살린 RViz 튜너 3파일은
d374a94 가 이미 제거 — 이건 그 네 번째다).
지운 이유는 실행 가드(`--allow-unsafe-legacy`)가 있음에도 남는 위험이다:
그 스크립트는 `build_map.py` 와 **같은 런타임 경로**(`config/global_trajectory/`,
`config/centerline/`)에 **같은 파일명 규칙**으로 쓰면서 manifest 도 폐루프
게이트도 남기지 않는다. 한 번 돌면 게이트 통과본이 게이트 없는 CSV 로 덮이고
manifest 의 sha256 만 옛 파일을 가리킨 채 남는다. `--v-max` 를 달리 주면
같은 맵에 CSV 가 둘 생겨 launch 의 glob 이 "여러 CSV 매칭"으로 죽는다.

기능적으로도 대체됐다: 그 구현의 중앙선 추출은 안·바깥 경계 **중점** 방식이라,
인필드가 아웃필드 벽까지 뻗는 '게이트'에서 중점이 한쪽 벽으로 붕괴해 곡률이
튄다 (R~0.3 m 스파이크). `build_map.py` 는 이걸 medial-axis(골격)로 바꿔
해결했다 — 옛 결과와 비교해야 하면 `git show 48fe25a^:...step4_main.py` 로
꺼내 쓰고, 저장 경로는 반드시 `--out` 으로 돌릴 것.

### Raceline 모드: `--raceline {center,iqp}` (기본 iqp)

- `center` — step1 중앙선을 그대로 사용 (경로 최적화 없음)
- `iqp` — **TUM 스타일 최소곡률 IQP 최적화** (UNICORN 팀과 동일 접근).
  중앙선 각 점을 법선 방향 alpha만큼 이동, 트랙 폭 제약 안에서 곡률
  제곱합 최소화. 경계 있는 최소제곱(scipy lsq_linear)을 반복해 푼다.
  Spielberg 실측: **랩타임 51.64s -> 49.41s (-4.3%)**, max|kappa| 0.73 -> 0.45.
  `--safe-dist 0.30`(차 반폭+여유)으로 벽 접근 한계 조절 — UNICORN의
  safety_width에 해당. 줄이면 빠르지만 벽에 붙는다.

### 결과 확인

정적 미리보기는 `build_map.py`가 저장하는 `outputs/*_trajectory.png`,
폐루프 검증은 `outputs/*_tracking.png` 를 본다.

주행하면서 보려면 시뮬을 띄운다:

```bash
ros2 launch planning_pkg sim_race.launch.py map:=<맵이름>
```

속도 프로파일을 손으로 고치려면 웹 튜너를 쓴다 (라인 기하는 여기 build_map.py,
속도만 튜너 담당):

```bash
tune <맵이름>          # 또는: python3 tools/racelinetuner.py <맵이름>
```

지원 맵: Spielberg처럼 트랙이 폐곡선 링 형태인 맵 (`f1tenth_gym_ros/maps/`).
levine 같은 실내 corridor 맵은 free space가 이미지 경계와 붙어 있어
현재의 링 검출 방식으로는 처리되지 않는다.

## 출력

- `outputs/<map>_max..._margin..._mu....csv` — global trajectory CSV.
  같은 파일이 `planner_pkg/config/global_trajectory/`에도 복사되므로 ROS2 환경에서
  `ros2 launch planner_pkg planner_pkg_launch.py trajectory_csv_file:=<파일명>`으로 바로 사용 가능.
- `outputs/<map>_trajectory.png` — 트랙 경계 + 속도 색상 궤적 + 주행 경로
- `outputs/<map>_tracking.png` — 속도 프로파일 추종, 횡오차, 조향각

## 파이프라인 구성

| 단계 | 파일 | 대응하는 ROS 노드 |
|---|---|---|
| **맵 → CSV 전체 (실사용)** | **`build_map.py`** | (오프라인 raceline 생성) |
| 맵 → centerline 추출 | `step1_track.py` | (오프라인 raceline 생성 단계) |
| 속도 프로파일 + CSV | `step2_global_trajectory.py` | `global_trajectory_node` 입력 생성 |
| 로컬 윈도우(130 wpt) + 배포 제어기 + 차량 모델 | `build_map.py` | `local_trajectory_node` + C++ `control_node` / Python tracker |

`../deployment_profiles.py`가 배포 파라미터를 런타임 YAML에서 해석한다.
C++ 프로파일은 MAP/L1 lookahead와 실제 NUC6 steering LUT를 명령 역변환과
plant 정방향 응답 양쪽에 사용한다. Python 프로파일은
`clip(0.5·v, 0.6, 1.2)` pure pursuit를 사용한다. 속도 프로파일은 각 구간
양끝 중 더 큰 횡가속도와 종가속도를 합쳐 `mu·g` 안으로 반복 투영하고,
폐루프 모델도 같은 friction circle을 적용한다. 포화가 한 번이라도 필요했던
제어 요구는 물리적으로 따라갈 수 없는 것으로 fail-closed 처리한다.
맵 좌표 변환은 YAML `origin: [x, y, yaw]` 회전을 포함한다. 작은 섬/오목부
정리는 중앙선 골격용 사본에만 적용하며, corridor와 최종 차량 footprint는
기둥·고정 장애물을 보존한 원본 occupancy 마스크로 검사한다.

---

## Head-to-Head local planner 비교 (path selection vs frenet)

기존 `local_trajectory_node`는 raceline을 그대로 자르는 추종 전용이라
H2H 회피/추월 계층을 새로 구현했다. 모든 튜닝 파라미터는 `params.yaml`
한 곳에 있고, 각 항목에 튜닝 가이드 주석이 달려 있다.

| 파일 | 내용 |
|---|---|
| `params.yaml` | **전체 파라미터 + 튜닝 주석** (Traxxas 순정/Jetson NX 근거 포함) |
| `step5_frenet.py` | Frenet 변환 (frenet_frame_node 미러) + 회피경로 필드 복원 |
| `step6_planner_base.py` | 공통: 시공간 충돌체크, 코리도 계산, 속도 재프로파일, 출력 포맷 |
| `step7_planner_path_selection.py` | 방식 1: 코리도 비율 lattice 후보 + 비용 선택 |
| `step8_planner_frenet.py` | 방식 2: Werling (s(t), d(t)) 다항식 샘플링 |
| `step9_opponent.py` | 상대 차량 + 관측 노이즈/지연 (Obstacle.msg 필드 규격) |
| `step10_simulate_h2h.py` | 단일 시나리오 실행 (`--planner`, `--plot`) |
| `step11_compare.py` | Monte Carlo 배치 비교 → CSV/PNG 리포트 |

```bash
# 단일 실행 (플롯 포함)
.venv/bin/python planning_ws/global_traj_sim/step10_simulate_h2h.py \
    --planner frenet --seed 0 --plot

# 후처리 재생: 시뮬 로그를 follow-cam GIF로 저장 (outputs/h2h_*.gif)
.venv/bin/python planning_ws/global_traj_sim/step10_simulate_h2h.py \
    --planner frenet --seed 0 --animate --anim-speed 2 --anim-fps 20

# 배치 비교 (planner x 상대속도 x 노이즈 x 시드)
.venv/bin/python planning_ws/global_traj_sim/step11_compare.py --seeds 5 --jobs 4

# 실전형 레이스: frenet 차량 vs path_selection 차량 동시 출발 (step12_race_h2h.py)
# 두 차 모두 완전한 planner 스택, 동일 raceline 속도 한계, 관측 노이즈 기본 ON.
# 속도 우위가 없으므로 planner 실력 차이만 승부에 반영된다.
.venv/bin/python planning_ws/global_traj_sim/step12_race_h2h.py --seed 0 --plot --animate
.venv/bin/python planning_ws/global_traj_sim/step12_race_h2h.py --series 10   # 승수 집계
```

step10_simulate_h2h.py는 "느린 상대를 추월"하는 통제 실험(속도차 = 파라미터),
step12_race_h2h.py는 "동급 상대와 레이스"하는 실전 실험이다. 레이스 그리드/랩 수 등은
params.yaml의 race: 섹션에서 튜닝.

### 디버깅 노트: frenet이 초기에 랩당 3초 느렸던 이유

첫 노이즈 레이스 시리즈에서 frenet이 0:8로 완패했는데(베스트 랩 54.1s vs
51.0s), 원인 추적 과정이 교훈적이라 기록해둔다:

1. **가설 1 (틀림)**: 노이즈 관측에 하드 threshold를 건 이진 blocked 판정이
   replan마다 뒤집히는 플리커. → 연속 램프(_block_penalty)로 고쳤지만
   랩타임 변화 없음. 코드 품질 개선일 뿐 병목이 아니었다.
2. **진짜 원인 1 — a_lat 하드 리젝트**: 속도유지형(quartic) 종방향 후보는
   코너 앞에서 미리 감속하지 않으므로, 코너 접근 시 "현재 속도 유지" 후보가
   전부 횡가속 한계를 초과 → 후보의 99.8%가 기각되어 매 replan이 follow
   폴백으로 빠짐. 기각 대신 실행 단계의 velocity_profile_on_path가
   friction-circle 상한으로 재보정하도록 변경.
3. **진짜 원인 2 — v_cap 버그**: 궤적 종단 이후 구간 속도 상한을
   np.maximum(v_prof, v_end)로 풀어주려 했으나 v_prof가 이미 v_end로
   수렴하는 프로파일이라 호라이즌 전체가 종단 속도로 눌렸다. 직선에서
   raceline 속도를 못 내던 원인. 궤적 커버 구간/이후 구간을 분리해 해결.

수정 후: 3승:5승, 베스트 랩 52.3s vs 51.0s. 교훈 — "노이즈에 약하다"는
현상의 원인이 노이즈 처리 로직이 아니라 solo 페이스 결함일 수 있다.
비교 실험에서는 반드시 단독 주행 기준선부터 확인할 것.

참고: 렌더 백엔드가 Agg(무화면)라 실시간 창은 안 뜨고, 시뮬을 다 돌린 뒤
로그를 재생해 GIF로 저장하는 방식이다. 라이브로 보려면 RViz 파이프라인 사용.

### 실차(ROS2) 연동 포인트

- **입력**: planner는 frenet odom(`/car_state/frenet/odom`)과
  `ObstacleArray`(Obstacle.msg 필드명 그대로) dict를 받는다.
  실차에선 `opponent.observe()`만 perception 노드 출력으로 교체.
- **출력**: `Wpnt.msg` 전체 필드(80개, ds=0.1m)를 채워서 내보내므로
  `crazy_controller`를 수정 없이 그대로 사용 가능
  (controller가 x/y/vx 외에 s, psi, kappa, d_left/d_right도 소비함).
- **behavior 상태**: planner dbg의 `mode`(global/overtake/follow/estop)는
  `BehaviorState.msg`의 GLOBAL/AVOID 상태에 대응 → 포팅 시 behavior
  노드로 분리 가능.
- **계산 예산**: Python 기준 평균 3~5 ms/plan (Jetson NX에서 20 Hz 여유,
  첫 호출은 워밍업으로 ~50 ms). C++ 포팅 시 40 Hz
  (`local_trajectory/loop_rate_hz`) 그대로 가능.
