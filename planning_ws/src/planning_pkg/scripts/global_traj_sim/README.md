# global_traj_sim

ROS 없이 `.venv`만으로 도는 global trajectory 생성 + 시뮬레이션 파이프라인.
`f1tenth_gym_ros`의 맵(yaml/png)에서 centerline을 추출해 `planner_pkg`의
`global_trajectory_node`가 읽는 것과 동일한 9열 세미콜론 CSV
(`s_m; x_m; y_m; d_right; d_left; psi_rad; kappa_radpm; vx_mps; ax_mps2`)를
생성하고, pure pursuit + kinematic bicycle 모델로 폐루프 추종 시뮬레이션을 돌린다.

## 설치 (어떤 환경에서든)

```bash
cd <workspace root>          # IFAC_ws-main
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## 실행

```bash
.venv/bin/python planning_ws/global_traj_sim/step4_main.py --map Spielberg_map
```

옵션: `--v-max 7.0 --mu 0.85 --margin 0.15 --laps 1 --out <dir>`

### Raceline 모드: `--raceline {center,iqp}` (기본 iqp)

- `center` — step1 중앙선을 그대로 사용 (경로 최적화 없음)
- `iqp` — **TUM 스타일 최소곡률 IQP 최적화** (UNICORN 팀과 동일 접근).
  중앙선 각 점을 법선 방향 alpha만큼 이동, 트랙 폭 제약 안에서 곡률
  제곱합 최소화. 경계 있는 최소제곱(scipy lsq_linear)을 반복해 푼다.
  Spielberg 실측: **랩타임 51.64s -> 49.41s (-4.3%)**, max|kappa| 0.73 -> 0.45.
  `--safe-dist 0.30`(차 반폭+여유)으로 벽 접근 한계 조절 — UNICORN의
  safety_width에 해당. 줄이면 빠르지만 벽에 붙는다.

### RViz 시각화

생성된 raceline은 RViz에서 본다 (별도 시각화 노드는 레이스라인 튜너로 통합):

```bash
# 결과 확인 + 드래그 편집 (맵 + 미리보기 + 인터랙티브 핸들)
ros2 launch planning_pkg raceline_tuner.launch.py csv:=<파일명>.csv
# 또는 시뮬 주행과 함께: ros2 launch planning_pkg sim_race.launch.py tuner:=true
```

정적 미리보기는 step4가 저장하는 `outputs/*_trajectory.png` 참고.

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
| 맵 → centerline 추출 | `step1_track.py` | (오프라인 raceline 생성 단계) |
| 속도 프로파일 + CSV | `step2_global_trajectory.py` | `global_trajectory_node` 입력 생성 |
| 로컬 윈도우(80 wpt) + pure pursuit + 차량 모델 | `step3_simulate.py` | `local_trajectory_node` + `crazy_controller` + f1tenth sim |

컨트롤러/차량 파라미터는 `crazy_controller` 설정을 따름:
wheelbase 0.324 m, L1 = clip(0.3·v + q_l1, t_clip_min, t_clip_max).
속도 프로파일은 friction-circle 기반 forward-backward pass (mu·g 횡가속 한계,
가속 5.5 / 제동 7.0 m/s²).

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
