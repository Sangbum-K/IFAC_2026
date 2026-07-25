# Raceline Tuner

RViz에서 레이스라인(라인 + 속도)을 인터랙티브하게 튜닝하는 도구.
(HMCL unicorn `raceline_tuner`의 ROS2 구현)

- 🟣 **보라 구체 드래그** → 라인을 좌우로 이동 (드래그의 **횡방향 성분만** 사용,
  주변이 코사인 falloff로 부드럽게 따라오고 벽 여유 안으로 자동 클램프)
- 🟢 **초록 큐브 위/아래 드래그** → 그 구간 속도 조절 (핸들 높이 = 속도 × 0.3)
- **핸들 우클릭 메뉴**: 속도 재계산(마찰한계) / 실행취소(최근 5회) / 전체 리셋 / CSV 저장
- 미리보기 라인: 파랑(느림) → 빨강(빠름), 앵커마다 속도 텍스트

관련 파일:
[`raceline_tuner_node.py`](raceline_tuner_node.py) ·
[`launch/raceline_tuner.launch.py`](../../launch/raceline_tuner.launch.py) ·
RViz 설정은 [`rviz/sim_race.rviz`](../../rviz/sim_race.rviz) 하나로 통합 (튜너 디스플레이 포함)

---

## 실행 (두 가지 방법)

```bash
source /opt/ros/humble/setup.zsh          # zsh에서 setup.bash는 깨짐
source ~/IFAC_ws/planning_ws/install/setup.zsh

# 1) 튜너 단독 — 맵 + 튜너 + RViz (gym 불필요). 인자 없이 = 대회맵 my_track
ros2 launch planning_pkg raceline_tuner.launch.py

# 다른 맵이면 map:= 하나만 바꾸면 된다 (CSV는 자동으로 찾음)
ros2 launch planning_pkg raceline_tuner.launch.py \
    map:=Spielberg_map stride:=25 falloff:=8.0 v_max:=15.0 mu:=0.85

# 2) 시뮬과 함께 — 같은 RViz에서 주행 보면서 편집
ros2 launch planning_pkg sim_race.launch.py map:=my_track tuner:=true
```

`sim_race.launch.py tuner:=true` 는 `map`/`suffix` 로 정해진 **iqp CSV**를
자동으로 물고 들어가지만, stride/falloff/v_max/mu 는 **노드 기본값
(25 / 8.0 m / 12.0 / 0.85) 고정**이다. 편집이 목적이면 단독 실행 쪽을 쓸 것.

### launch 인자 (기본값 = my_track 기준)

| 인자 | 기본값 | 의미 |
|---|---|---|
| `map` | `my_track` | 배경 맵 (`maps/<이름>.yaml`, 확장자 제외) |
| `csv` | `''` (자동) | 비우면 `<map>_iqp_*.csv` 를 glob으로 찾는다. sp 라인 편집 시엔 직접 지정 |
| `stride` | `10` | 핸들 간격 [wpt]. ds=0.1 m 이므로 10 = **1.0 m** |
| `falloff` | `3.0` | 드래그 영향 반경 [m] |
| `v_max` | `6.0` | "속도 재계산" 상한 [m/s] |
| `mu` | `0.75` | "속도 재계산" 마찰계수 |
| `min_wall` | `0.35` | 라인–벽 최소 여유 [m] |

맵/CSV가 없으면 launch가 **가능한 맵 목록과 함께 즉시 에러**를 낸다 (노드가
조용히 죽지 않음). 현재 리포에 있는 조합:

| 맵 | CSV (iqp) | 생성 조건 |
|---|---|---|
| `my_track` (대회맵) | `my_track_iqp_max6.00ms_margin0.08m_mu0.75.csv` | 6 m/s · mu 0.75 |
| `my_map` | `my_map_iqp_max6.00ms_margin0.15m_mu0.75.csv` | 6 m/s · mu 0.75 |
| `Spielberg_map` | `Spielberg_map_iqp_max15.00ms_margin0.15m_mu0.85.csv` | 15 m/s · mu 0.85 |
| `Budapest_map` | `Budapest_map_iqp_max15.00ms_margin0.15m_mu0.85.csv` | 15 m/s · mu 0.85 |

`_sp_*.csv`(추월 라인)도 `csv:=<맵>_sp_*.csv` 로 지정하면 같은 방식으로 편집된다.

---

## ⚠️ 알아둘 것

1. **`v_max` / `mu` 는 편집할 라인을 뽑을 때 쓴 값과 맞춰야 한다.**
   "속도 재계산" 메뉴가 이 값으로 마찰한계를 다시 푼다. my_track 라인은
   6 m/s · mu 0.75 로 만든 것이라 기본값이 그렇게 맞춰져 있지만, **Spielberg 등
   다른 맵을 편집할 땐 `v_max:=15.0 mu:=0.85` 를 같이 넘겨야** 속도가 반토막 나지 않는다.
   (라인만 옮기고 속도는 그대로 두려면 이 메뉴를 아예 누르지 않으면 된다.)

2. **`falloff` 는 트랙 길이에 맞춰야 한다.**
   my_track 전장이 **29.3 m** 뿐이라 노드 기본값 ±8 m 로 드래그하면 트랙의
   **절반 이상**이 끌려온다. 그래서 launch 기본값을 3.0 으로 낮춰 뒀다.
   Spielberg(339 m)처럼 큰 트랙은 `falloff:=8.0 stride:=25` 가 맞다.

3. **`min_wall` 0.35 m 가 좁은 트랙에선 빡빡하다.**
   my_track의 좌우 폭 합은 0.85~1.46 m(평균 1.09 m). 양쪽 0.35 m를 빼면
   **가장 좁은 구간에 남는 여유가 0.15 m**뿐이라, 거기선 드래그가 아예
   안 먹는 것처럼 보인다. 게다가 CSV의 `d_left/d_right` 는 이미 벽에서
   margin 0.08 m 를 뺀 값이라 **실제 벽까지는 0.43 m** 확보된다.
   차폭 30 cm 기준이면 `min_wall:=0.25` 정도까지는 안전하다.

4. **드래그는 MOUSE_UP에서만 반영된다.** 끌고 있는 동안 라인이 안 움직여도 정상.
   놓는 순간 falloff 적용 + 스무딩 + `s/psi/kappa` 재계산이 한 번에 돈다.

---

## 저장 → 빌드 → 주행 반영 (검증됨)

RViz에서 아무 핸들이나 **우클릭 → CSV 저장**. 그다음 리포 루트에서
`./build_all.sh` (HOW_TO_RUN 1장과 동일) 하면 **그대로 주행에 반영된다.**

경로를 따라가면:

```
[튜너 저장]  src/planning_pkg/config/global_trajectory/my_track_iqp_*.csv
                 │  ./build_all.sh  →  colcon build (install(DIRECTORY config))
                 ▼
[빌드]       install/planning_pkg/share/planning_pkg/config/global_trajectory/…
                 │  global_trajectory_node 가 ament share 경로로 읽음
                 ▼
[주행]       /global_waypoints → local_trajectory_node → control_node
```

확인한 것:

- 튜너는 **install이 아니라 소스 트리**의 CSV를 덮어쓴다 (그래야 재빌드로 반영됨).
- `CMakeLists.txt` 의 `install(DIRECTORY config ...)` 가 **매 빌드마다 변경분을
  다시 복사**한다. 파일이 새로 생기든 내용만 바뀌든 `colcon build` 한 번이면 install에 반영된다
  (심링크가 아니므로 빌드를 건너뛰면 **옛 라인으로 주행한다**).
- `global_trajectory_node` 는 `get_package_share_directory("planning_pkg")` +
  `/config/global_trajectory/<파일명>` 을 읽는다
  ([global_trajectory_node.cpp:83](../../src/global_trajectory_node.cpp#L83)).
- 로더는 `#` 주석 줄을 건너뛰고 `;` 로 자르며 공백을 trim 한다 → 튜너가 쓰는
  `# tuned by raceline_tuner_node` 헤더와 `; ` 구분자 그대로 잘 읽힌다.

**파일명이 바뀌지 않는 한** 추가 설정은 필요 없다 (튜너는 같은 파일에 덮어쓴다):

- `ros2 launch integration_pkg real_car.launch.py map:=my_track` → `<맵>_iqp_*.csv`
  를 glob으로 찾으므로 자동.
- `map:=` 없이 실행 → [`config/params.yaml`](../../config/params.yaml) 의
  `global_trajectory/trajectory_csv_file` 값(현재 `my_track_iqp_max6.00ms_margin0.08m_mu0.75.csv`)
  을 쓴다. `tuner/output_csv` 로 **다른 파일명에 저장했다면 여기도 같이 고쳐야 한다.**

주행 중 로그에서 `Loading trajectory from: .../install/.../my_track_iqp_...csv` 를
확인하면 확실하다.

백업 주의:

- 원본은 첫 저장 때 한 번 `*.bak` 으로 백업하고 그 뒤엔 덮어쓴다.
  **여러 번 저장해도 `.bak`은 그 세션의 최초 상태 하나뿐**이고, 튜너를 다시 켜고
  저장하면 그 `.bak`마저 덮어써진다. 중간 상태를 남기려면 직접 `cp` 하거나
  `tuner/output_csv` 로 다른 파일에 쓸 것.
- `.bak` 은 `*.csv` 로 끝나지 않으므로 launch의 glob(`*_iqp_*.csv`)에 걸리지 않는다.

---

## 새 맵으로 튜닝하기

1. **맵 파일 넣기** — `.png` + `.yaml` 을 [`maps/`](../../maps/) 에 복사.
   `.yaml` 의 `image:` 줄이 실제 png 파일명과 같은지 확인.
2. **raceline CSV 넣기** — 9열(`;` 구분) CSV를 `config/global_trajectory/` 에 복사.
   보통은 [`../global_traj_sim/`](../global_traj_sim/README.md) 의 `build_map.py`
   가 두 곳(`outputs/` + `config/`)에 자동으로 떨궈 준다.
3. **빌드** — 맵은 심링크가 아니라 **빌드 시 복사**되므로, 새 맵은 빌드해야 RViz에 뜬다.
   ```bash
   cd ~/IFAC_ws/planning_ws && colcon build --packages-select planning_pkg
   source install/setup.zsh
   ```
4. **실행** — `map:=<맵이름>` 만 넘기면 CSV는 자동으로 찾는다 (하드코딩된 맵 이름은 없다).
   그 맵 라인을 뽑을 때 쓴 `v_max` / `mu` 도 같이 넘길 것.
5. **편집 → 저장 → 다시 빌드**.

맵 생성부터의 전체 절차는 [`maps/README.md`](../../maps/README.md) 참고.

---

## CSV 포맷 (9열, `;` 구분)

```
s_m ; x_m ; y_m ; d_right ; d_left ; psi_rad ; kappa_radpm ; vx_mps ; ax_mps2
```

| 열 | 의미 |
|----|------|
| `s_m` | 누적 거리 (wpt 간격 0.1 m) |
| `x_m`, `y_m` | 좌표 |
| `d_right`, `d_left` | **현재 라인 기준** 우/좌 남은 트랙 폭 |
| `psi_rad` | 헤딩 |
| `kappa_radpm` | 곡률 |
| `vx_mps` | 속도 |
| `ax_mps2` | 종방향 가속도 |

`#` 로 시작하는 줄은 주석. 저장 시에도 헤더 2줄이 다시 붙는다.

라인을 옮기면 `s / psi / kappa` 와 `d_left / d_right` 는 자동 갱신되고,
**속도 재계산** 메뉴는 마찰한계(`v ≤ √(mu·g/κ)`) + 가감속 한계로
forward-backward 프로파일을 다시 푼 뒤 `ax` 까지 채운다.

---

## 파라미터 바꾸기

노드 파라미터는 아래와 같고, 대부분 launch 인자로 노출돼 있다.
`ros2 param set` 은 생성자에서만 읽으므로 **안 먹는다** — launch 인자로 넘기거나
`launch/raceline_tuner.launch.py` 의 `parameters=[...]` 를 고칠 것.

| 파라미터 | 노드 기본 | launch 인자 (기본) | 의미 |
|----------|-----------|---------------------|------|
| `tuner/raceline_csv` | `''` | `csv:` / `map:` 에서 결정 | 편집할 CSV 절대경로 |
| `tuner/output_csv` | `''` | — (launch 편집 필요) | 비면 입력 파일에 덮어씀. 지정하면 그쪽에 저장(`.bak` 안 만듦) |
| `tuner/handle_stride` | 25 | `stride:` (10) | 핸들 간격 [wpt] |
| `tuner/falloff_window_m` | 8.0 | `falloff:` (3.0) | 드래그 영향 반경 [m] |
| `tuner/min_wall_dist_m` | 0.35 | `min_wall:` (0.35) | 라인–벽 최소 여유 [m] |
| `tuner/v_max_mps` | 12.0 | `v_max:` (6.0) | 속도 재계산 상한 [m/s] |
| `tuner/mu` | 0.85 | `mu:` (0.75) | 마찰계수 |
| `tuner/a_acc_max` | 5.5 | — (launch 편집 필요) | 최대 가속 [m/s²] |
| `tuner/a_dec_max` | 7.0 | — (launch 편집 필요) | 최대 감속 [m/s²] |
| `tuner/vel_handle_scale` | 0.3 | — (launch 편집 필요) | 속도 핸들 높이 = v × scale [m] |

`sim_race.launch.py tuner:=true` 로 띄우면 CSV 경로만 넘기므로 **나머지는 전부
노드 기본값**(25 / 8.0 / 0.35 / 12.0 / 0.85)이라는 점에 주의.

---

## 잘 안 될 때

| 증상 | 원인 / 확인 |
|---|---|
| `raceline CSV가 없습니다` / `가능한 맵: …` | launch가 CSV를 못 찾은 것. `map:=` 이름 확인, 또는 `csv:=` 직접 지정 |
| `맵이 없습니다: ….yaml` | 새 맵을 `src/planning_pkg/maps/` 에 넣고 `colcon build` 를 안 함 (맵은 복사됨) |
| `CSV 9열 아님` | 센터라인/벽 CSV를 넘긴 것. `config/global_trajectory/` 의 `_iqp_`/`_sp_` 파일이어야 함 |
| 핸들은 보이는데 라인이 안 움직임 | 벽 여유 클램프에 걸린 것. `min_wall:=0.25` 로 낮춰 볼 것 (위 §3) |
| "속도 재계산" 눌렀더니 속도가 확 바뀜 | `v_max:` / `mu:` 가 그 라인 생성값과 다름. 되돌리려면 우클릭 → 실행취소 |
| 미리보기 라인이 사라짐 | 2초마다 재발행하므로 잠깐 기다리면 복구됨 (RViz 재시작 대비) |
| 저장했는데 주행이 그대로 | `./build_all.sh` 를 안 했거나, `tuner/output_csv` 로 다른 파일에 저장해 `params.yaml` 과 어긋남. 주행 로그의 `Loading trajectory from:` 경로 확인 |
