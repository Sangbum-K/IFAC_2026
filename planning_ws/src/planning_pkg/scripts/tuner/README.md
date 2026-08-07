# Raceline Tuner

RViz에서 레이스라인(라인 + 속도)을 인터랙티브하게 튜닝하는 도구.
(HMCL unicorn `raceline_tuner`의 ROS2 구현)

- 🟣 **보라 구체 드래그** → 라인을 좌우로 이동 (드래그의 **횡방향 성분만** 사용,
  주변이 코사인 falloff로 부드럽게 따라오고 벽 여유 안으로 자동 클램프)
- **속도 핸들 위/아래 드래그** → 그 구간 속도 조절 (핸들 높이 = 속도 × 0.3)
- **핸들 우클릭 메뉴**: 속도 재계산(마찰한계) / 실행취소(최근 5회) / 전체 리셋 / CSV 저장

### 화면에 뜨는 것

| 표시 | 내용 |
|---|---|
| 바닥 라인 (`tuned_line`) | 주행 라인 자체 |
| **속도 곡선** (`vel_curve`) | 속도 핸들을 잇는 곡선. 앵커끼리 직선으로 잇지 않고 **전 wpt(0.1 m)를 지나가므로** falloff로 만든 속도 변화가 그대로 부드럽게 보인다 |
| 수직 눈금 (`vel_stems`) | 곡선 높이가 트랙 어느 지점인지 읽히도록 바닥과 연결 (0.5 m 간격) |
| 속도 텍스트 (`vel_text`) | 앵커마다 숫자 |

**색은 셋 다 속도에 연동**된다 (`_speed_color()` 하나를 공유):

```
느림 ──────────────────────────────────────── 빠름
파랑 (0,0.35,1) → 청록 (0,0.9,0.8) → 노랑 (1,0.9,0) → 빨강 (1,0.15,0)
```

범위는 **현재 프로파일의 min~max**라 대비가 항상 최대다 (편집하면 같이 움직인다).
속도 핸들은 높이뿐 아니라 색도 바뀌므로 드래그 후 `setPose`가 아니라 마커를
다시 만들어 덮어쓴다.

관련 파일:
[`raceline_tuner_node.py`](raceline_tuner_node.py) ·
[`launch/raceline_tuner.launch.py`](../../launch/raceline_tuner.launch.py) ·
RViz 설정은 [`rviz/sim_race.rviz`](../../rviz/sim_race.rviz) 하나로 통합 (튜너 디스플레이 포함)

---

## 실행 (두 가지 방법)

```bash
source /opt/ros/humble/setup.zsh          # zsh에서 setup.bash는 깨짐
source ~/IFAC_ws/planning_ws/install/setup.zsh

# 1) 튜너 단독 — 맵 + 튜너 + RViz (gym 불필요). 인자 없이 = 현행 실차 트랙 my_track0807
ros2 launch planning_pkg raceline_tuner.launch.py

# 다른 맵도 map:= 하나면 끝. CSV·v_max·margin·mu·stride·falloff가 자동으로 정해진다
ros2 launch planning_pkg raceline_tuner.launch.py map:=Spielberg_map

# 단축어 (~/.zshrc, mapp/mkcsv 와 같은 자리). sa 로 source 한 뒤:
tune                              # = 기본 맵
tune my_track0727                 # = map:=my_track0727
tune my_track0727 min_wall:=0.25  # 추가 인자는 그대로 전달
tune --help                       # 사용법

# 2) 시뮬과 함께 — 같은 RViz에서 주행 보면서 편집
ros2 launch planning_pkg sim_race.launch.py map:=my_track0807 tuner:=true
```

`sim_race.launch.py tuner:=true` 도 파일명에서 v_max/margin/mu를 읽어 넘긴다.
다만 stride/falloff는 노드 기본값(25 / 8.0 m)이므로 세밀한 편집은 단독
launch가 편하다.

### launch 인자 — **`map:=` 말고는 넘길 게 없다**

`map:=` 하나만 주면 나머지는 CSV에서 유도된다. 명시하면 명시값이 이긴다.

| 인자 | 기본값 | 어디서 유도되나 |
|---|---|---|
| `map` | `my_track0807` | 배경 맵 (`<이름>.yaml`, 확장자 제외). `~/IFAC_ws/maps` → `planning_pkg/maps` 순으로 탐색 |
| `csv` | `''` (자동) | 비우면 `<map>_iqp_*.csv` 를 glob으로 찾는다. sp 라인 편집 시엔 직접 지정 |
| `v_max` | `''` (자동) | **CSV 파일명의 `max<...>ms`** — 그 라인을 뽑을 때 쓴 값이므로 항상 정확하다 |
| `corridor_margin` | `''` (자동) | **CSV 파일명의 `margin<...>m`** — map raycast 실벽 거리에서 빼서 저장할 reserve |
| `mu` | `''` (자동) | **CSV 파일명의 `mu<...>`** |
| `stride` | `''` (자동) | **전장** → 핸들 간격 1.0~2.5 m (핸들 ~30개). 31.3 m → `10`, 339 m → `25` |
| `falloff` | `''` (자동) | **전장의 10%**, 3~8 m 클램프. 31.3 m → `3.13`, 339 m → `8.0` |
| `min_wall` | `''` (0.20) | margin 차감 뒤 저장되는 가용 corridor의 최소 여유 [m] |

띄울 때 터미널에 실제로 쓰인 값이 찍힌다:

```
[raceline_tuner] map=my_track0727 (…/maps/my_track0727.yaml)  csv=my_track0727_iqp_max6.00ms_margin0.08m_mu0.75.csv
[raceline_tuner] 전장 30.5 m -> stride 10 wpt (1.0 m), falloff 3.05 m
[raceline_tuner] v_max=6.0 m/s  margin=0.08 m  mu=0.75  (파일명에서 자동)
```

파일명이 `_max…ms_margin…m_mu….csv` 꼴이 아니면(직접 만든 CSV 등)
v_max/margin/mu를 못 읽는다. v_max/mu는 속도 재계산에, margin은 저장되는
좌우 corridor에 직접 영향을 주므로 필요하면 세 값을 명시한다. margin을
생략하면 보수적인 0.15 m 기본값을 쓴다.

맵/CSV가 없으면 launch가 **가능한 맵 목록과 함께 즉시 에러**를 낸다 (노드가
조용히 죽지 않음). 현재 리포에 있는 조합:

| 맵 | CSV (iqp) | 생성 조건 |
|---|---|---|
| `my_track0807` (현행 실차) | `my_track0807_iqp_max7.00ms_margin0.08m_mu0.68.csv` | 7 m/s · 계획 mu 0.68 |
| `my_map` | `my_map_iqp_max6.00ms_margin0.15m_mu0.75.csv` | 6 m/s · mu 0.75 |
| `Spielberg_map` | `Spielberg_map_iqp_max15.00ms_margin0.15m_mu0.85.csv` | 15 m/s · mu 0.85 |
| `Budapest_map` | `Budapest_map_iqp_max15.00ms_margin0.15m_mu0.85.csv` | 15 m/s · mu 0.85 |

`_sp_*.csv`(추월 라인)도 `csv:=<맵>_sp_*.csv` 로 지정하면 같은 방식으로 편집된다.

---

## ⚠️ 알아둘 것

1. **`v_max` / `mu` 는 손으로 넘기지 마라.** 편집할 라인을 뽑을 때 쓴 값과
   어긋나면 "속도 재계산"이 엉뚱한 프로파일을 만든다(Spielberg 라인에 mu 0.75를
   주면 속도가 반토막 난다). 이제 launch가 **CSV 파일명에서 읽으므로** 비워 두는 게
   항상 맞다. 넘길 이유는 "일부러 다른 조건으로 다시 풀어 보고 싶을 때"뿐이다.

   "속도 재계산"은 `build_map.py` 와 **같은 마찰/가감속 모델**을 쓴다 (g-g 커플링
   포함, 아래 참조). 예전처럼 마찰한계를 넘는 프로파일을 만들지는 않는다.

   ⚠️ **다만 `build_map` 의 폭 연동 속도 제한(`width_speed_cap`)은 아직 튜너에
   없다.** build_map 은 벽 여유가 좁은 곳에서 `(여유 − 차 반폭) / 예상 횡추적오차
   기울기` 로 속도를 따로 깎는다 (my_track0807 은 314점 중 233점이 이 규칙에
   걸린다). 튜너의 투영은 곡률·마찰·가감속만 보므로 2026-08-07 실측 기준
   **"속도 재계산" 한 번에 좁은 구간 속도가 최대 +2.12 m/s (평균 3.68 → 4.35)
   부풀고, 폐루프 게이트도 그대로 통과한다** — 공칭 시뮬에서는 횡오차가 작은데,
   폭 규칙이 사는 이유가 바로 그 공칭과 실제의 차이다. 같은 경로로 라인 드래그
   (`_recompute_velocity_window`)도 편집 구간 속도를 부풀릴 수 있다.
   그러므로 **속도를 올리는 방향의 편집을 저장했으면 실차 투입 전에
   `build_map` 재생성으로 대조할 것.** 속도를 내리는 방향은 안전하다.

2. **`falloff` / `stride` 도 자동이다.** my_track0807 전장이 **31.3 m** 뿐이라
   노드 기본값 ±8 m 로 드래그하면 트랙의 **절반 이상**이 끌려온다.
   launch가 전장의 10%(3~8 m 클램프)로 잡으므로 my_track0807은 3.13,
   Spielberg(339 m)는 8.0이 자동으로 나온다.

3. **`min_wall`은 margin을 뺀 뒤의 가용 corridor 기준이다.**
   기본 0.20 m와 my_track0807의 margin 0.08 m를 합치면 실제 center-to-wall은
   최소 0.28 m다. 차 반폭 0.15 m 및 recovery의 0.25 m 정지 계약보다
   보수적이면서, 생성된 원본(저장 d 최소 약 0.28 m)을 수정하지
   않고 다시 저장할 수 있다. 이 값을 차 반폭 0.15 m 아래로 낮추면 안 되며,
   최종 직사각 footprint/폐루프 gate는 값과 무관하게 항상 수행된다.

4. **드래그는 MOUSE_UP에서만 반영된다.** 끌고 있는 동안 라인이 안 움직여도 정상.
   놓는 순간 falloff 적용 + 스무딩 + `s/psi/kappa` 재계산이 한 번에 돈다.

5. **mu는 두 개다 — 계획 mu와 검증 mu.** 파일명의 `mu<...>`는 `build_map.py`의
   `MU_PROFILE`(계획용, 현재 0.68)이고, 폐루프 게이트의 타이어 한계는 물리
   `MU`(현재 0.75)다. 튜너는 계획 mu를 파일명에서, 검증 mu를 `build_map.py`
   에서 직접 AST로 읽는다 (`generator_physical_mu`). 예전에는 파일명 mu 하나로
   둘 다 해서, **편집하지 않은 생성 직후의 CSV조차** `combined demand 7.58 >
   mu*g 6.67` 로 저장이 거부됐다.

6. **속도 핸들을 올려도 안 올라갈 수 있다 — 그건 정상이고, 이제 로그로 말해준다.**
   수동 속도는 목표가 아니라 **상한**으로만 쓰인다 (`_enforce_velocity_feasibility`).
   그 구간을 마찰이나 가감속 한계가 잡고 있으면 올린 만큼 그대로 잘리고,
   투영이 단조가 아니라 상한을 올린 결과가 **오히려 더 느려질 수도** 있다.
   요청이 실제로 반영되지 않으면 편집을 남기지 않고 원복한 뒤 이유를 찍는다:

   ```
   속도 증가 불가: idx 0, 5.97 m/s 요청 7.00 m/s -> 투영 결과 4.97 m/s.
     이 구간은 마찰(mu=0.68) 또는 가감속 한계(a_acc=1.80/a_dec=1.85)가 잡고 있습니다.
   ```

   **속도를 올리고 싶으면 라인을 먼저 펴라.** 속도는 곡률의 함수다.
   반영된 경우에도 로그는 요청값이 아니라 실제 반영값을 찍는다.

7. **`free_speed:=true` — 핸들이 튕겨 돌아오지 않게 하기.**
   투영을 끄면 드래그한 값이 그대로 남는다 (라인을 옮겨도 수동 속도를
   덮어쓰지 않는다). ax 열은 v 에서 다시 만들어 CSV 자기모순은 막는다.

   ```bash
   ros2 launch planning_pkg raceline_tuner.launch.py free_speed:=true
   ```

   **저장 검사는 그대로다.** 실측 예 (my_track0807, 핸들 +2 m/s):

   | | 화면 | 저장 |
   |---|---|---|
   | idx 0 | 4.97 → **6.97** 유지 | 거부 (합성 8.53 > 6.67, ax 7.92 > 1.80) |
   | idx 60 | 3.63 → **5.63** 유지 | 거부 (합성 14.48 > 6.67) |

   즉 이 모드는 "**보면서 정하는**" 용도지 한계를 푸는 스위치가 아니다.
   한계를 실제로 올리려면 그 근거를 가진 값을 바꿔야 한다:
   `mu:=` / `v_max:=` (튜너 계획), `deployment_profiles` 의 `A_ACC_MAX`
   (재생성 필요), 그리고 런타임 `global_trajectory/validation_mu`.
   ⚠️ 런타임 `global_trajectory_node` 는 CSV 를 읽을 때 마찰원을 **독립적으로
   다시** 검사하고 넘으면 노드가 뜨지 않는다 — 튜너만 풀어도 차는 안 움직인다.

---

## 저장 → 빌드 → 주행 반영 (검증됨)

RViz에서 아무 핸들이나 **우클릭 → CSV 저장**. 그다음 리포 루트에서
`./build_all.sh` (HOW_TO_RUN 1장과 동일) 하면 **그대로 주행에 반영된다.**

저장은 다음 검사를 모두 통과할 때만 원자적으로 반영된다.

- 현재 xy에서 `s/psi/kappa`를 다시 계산하고, 최신 OccupancyGrid에 좌우
  normal ray를 cell-by-cell로 쏴 `d_right/d_left`를 완전히 재작성한다.
  unknown/occupied/map 밖은 모두 벽이며, 파일명의 corridor margin도 다시 뺀다.
- 모든 waypoint가 `min_wall`을 만족하고 차량의 직사각 footprint 전체가
  free인지 검사한다.
- 마찰원·조향각·가감속·끝→시작 ax 일관성을 검사한다.
- `build_map.py`가 선택한 실제 배포 프로파일(`cpp_real_car`,
  `python_sim_tracker`)로 1 lap 폐루프 추종을 실행해 동적 footprint,
  조향 rate, 타이어 요구량까지 통과해야 한다. 모델/맵을 못 읽거나 timeout이면
  기존 CSV를 그대로 보존하고 저장을 거부한다.

경로를 따라가면:

```
[튜너 저장]  src/planning_pkg/config/global_trajectory/my_track0807_iqp_*.csv
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
- ⚠️ **복사만 하고 지우지는 않는다.** 소스에서 지운 CSV는 install에 그대로 남고,
  launch 의 `<맵>_iqp_*.csv` glob 은 그 유령을 정상 파일로 집는다. 2026-08-07 에
  실제로 그 상태였다 — b05264b 에서 삭제한 `my_track_iqp_*.csv` 로 시뮬이 계속
  돌고 있었다. 라인을 지우거나 이름을 바꿨으면
  `rm -rf install/planning_pkg/share/planning_pkg/config/{global_trajectory,centerline}`
  후 재빌드해서 install 을 소스와 정확히 맞출 것.
- `global_trajectory_node` 는 `get_package_share_directory("planning_pkg")` +
  `/config/global_trajectory/<파일명>` 을 읽는다
  ([global_trajectory_node.cpp:83](../../src/global_trajectory_node.cpp#L83)).
- 로더는 `#` 주석 줄을 건너뛰고 `;` 로 자르며 공백을 trim 한다 → 튜너가 쓰는
  `# tuned by raceline_tuner_node` 헤더와 `; ` 구분자 그대로 잘 읽힌다.

**파일명이 바뀌지 않는 한** 추가 설정은 필요 없다 (튜너는 같은 파일에 덮어쓴다):

- `ros2 launch integration_pkg real_car.launch.py map:=my_track0807` → `<맵>_iqp_*.csv`
  를 glob으로 찾으므로 자동.
- `map:=` 없이 실행 → [`config/params.yaml`](../../config/params.yaml) 의
  `global_trajectory/trajectory_csv_file` 값(현재 `my_track0807_iqp_max7.00ms_margin0.08m_mu0.68.csv`)
  을 쓴다. `tuner/output_csv` 로 **다른 파일명에 저장했다면 여기도 같이 고쳐야 한다.**

주행 중 로그에서 `Loading trajectory from: .../install/.../my_track0807_iqp_...csv` 를
확인하면 확실하다.

### 시뮬에서 "튜닝한 속도가 반영이 안 된 것 같을 때"

CSV의 `vx_mps`는 시뮬 주행속도로 **그대로 나가지 않는다.** 기본 sim 경로
(`sim_race.launch.py`, `tracker:=true`)의 배율은 다음과 같다:

```
CSV vx  ×  global_trajectory/velocity_scale (0.6667)  ×  tracker/speed_scale (0.30)
        = 실효 0.20배      예) CSV 6.0 m/s -> 시뮬 약 1.2 m/s
```

`tracker:=false` 로 C++ `control_node` 를 쓰면 뒤쪽이
`vehicle/speed_scale 0.30` + `vehicle/max_speed_mps 4.0` 클램프로 바뀔 뿐
배율이 있는 건 같다 (`environments/sim.yaml`). 둘 다 이 트랙의 연속 헤어핀에서
pure pursuit가 따라가도록 **의도적으로 낮춘 값**이므로, 튜닝 결과를 확인할 때는
시뮬 속도계가 아니라 **RViz 속도곡선/텍스트(= CSV 값)** 를 보고,
시뮬에서는 "비례해서 빨라졌는지"만 확인한다.

백업 주의:

- 원본은 세션의 첫 저장 때 백업한다. 기존 `.bak`을 덮어쓰지 않고
  `.bak.1`, `.bak.2`, …의 첫 빈 이름을 원자적으로 선택한다. 같은 세션에서
  여러 번 저장하면 그 세션 최초 상태 백업 하나만 생긴다.
- `.bak` 은 `*.csv` 로 끝나지 않으므로 launch의 glob(`*_iqp_*.csv`)에 걸리지 않는다.

---

## 새 맵으로 튜닝하기

`mapp` 으로 만든 맵이면 **복사할 것이 없다.** 배경 맵을
`~/IFAC_ws/maps` → `planning_pkg/maps` 순으로 찾기 때문이다
(`build_map.py` / `localization.launch.py` 와 같은 순서, `.pgm` 도 그대로 로드).

```bash
mapp my_track0727                 # -> ~/IFAC_ws/maps/my_track0727.{pgm,yaml}
mkcsv my_track0727 ccw            # -> src/.../config/global_trajectory/my_track0727_iqp_*.csv
cd ~/IFAC_ws && ./build_all.sh    # CSV를 install 로 (맵은 복사 불필요)
tune my_track0727                 # = ros2 launch planning_pkg raceline_tuner.launch.py map:=...
```

맵/CSV를 손으로 넣는 경우:

1. **맵 파일 넣기** — `.png`(또는 `.pgm`) + `.yaml` 을 `~/IFAC_ws/maps/` 또는
   [`maps/`](../../maps/) 에 복사. `.yaml` 의 `image:` 줄이 실제 이미지 파일명과
   같은지 확인. `planning_pkg/maps/` 쪽에 넣었다면 맵은 심링크가 아니라 **빌드 시
   복사**되므로 `colcon build --packages-select planning_pkg` 가 필요하다
   (`~/IFAC_ws/maps` 는 빌드 없이 바로 읽는다).
2. **raceline CSV 넣기** — 9열(`;` 구분) CSV를 `config/global_trajectory/` 에 복사.
   보통은 [`../global_traj_sim/`](../global_traj_sim/README.md) 의 `build_map.py`
   가 두 곳(`outputs/` + `config/`)에 자동으로 떨궈 준다.
   (튜너는 CSV만은 **소스 트리**에서 읽으므로 여기 있어야 한다.)
3. **실행** — `map:=<맵이름>` 만 넘기면 CSV는 자동으로 찾는다 (하드코딩된 맵 이름은 없다).
   그 맵 라인을 뽑을 때 쓴 `v_max` / `mu` 도 같이 넘길 것.
4. **편집 → 저장 → `./build_all.sh`** (CSV를 install 에 반영해야 주행에 반영된다).

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
**속도 재계산** 메뉴는 아래 모델로 forward-backward 프로파일을 다시 푼 뒤
`ax` 까지 채운다.

### 속도 재계산 모델 (= `build_map.py` 의 `velocity_profile()`)

타이어가 낼 수 있는 힘은 `mu·g` 하나뿐이고 **선회와 가감속이 그 예산을 나눠 쓴다**
(마찰원 / g-g diagram):

```
√(a_lat² + a_x²) ≤ mu·g
```

그래서 코너 정점에서 횡가속을 100% 쓰고 있으면 제동에 쓸 예산이 남지 않는다.
forward/backward 패스는 매 지점에서 남은 예산만큼만 가감속을 허용한다:

```python
a_avail = a_max * √(1 - (a_lat / a_lat_max)²)
```

곡률 노이즈는 ±0.5 m **윈도우 최댓값**으로 잡는다. 이동평균을 쓰면 짧은 헤어핀의
정점 곡률이 뭉개져 거기서만 속도가 올라가기 때문이다 (my_track R=1.0 m 정점에서
2.72 → 2.99 m/s). 최댓값은 노이즈를 잡으면서 정점을 깎지 않는다.

my_track 기준 실측 (요구 마찰 = `√(a_lat² + a_x²)/g`, mu 0.75):

| | 요구 마찰 최대 | mu 초과 구간 |
|---|---|---|
| `build_map.py` (기준) | 0.77 | 0.3 m |
| 튜너 — **g-g 도입 전** | **1.59** | **16.3 m / 29.3 m** |
| 튜너 — 현재 | 0.75 | 0 m |

현재 튜너는 어느 지점에서도 `build_map.py` 보다 빠르지 않다 (윈도우 최댓값 때문에
평균 0.25 m/s 더 보수적). 즉 **재계산을 눌러서 라인이 더 공격적이 되는 일은 없다.**

---

## 파라미터 바꾸기

노드 파라미터는 아래와 같고, 대부분 launch 인자로 노출돼 있다.
`ros2 param set` 은 생성자에서만 읽으므로 **안 먹는다** — launch 인자로 넘기거나
`launch/raceline_tuner.launch.py` 의 `parameters=[...]` 를 고칠 것.

| 파라미터 | 노드 기본 | launch 인자 | 의미 |
|----------|-----------|--------------|------|
| `tuner/raceline_csv` | `''` | `csv:` / `map:` 에서 결정 | 편집할 CSV 절대경로 |
| `tuner/output_csv` | `''` | — (launch 편집 필요) | 비면 입력 파일에 덮어씀. 지정하면 그쪽에 저장(`.bak` 안 만듦) |
| `tuner/handle_stride` | 25 | `stride:` (전장에서 자동) | 핸들 간격 [wpt] |
| `tuner/falloff_window_m` | 8.0 | `falloff:` (전장에서 자동) | 드래그 영향 반경 [m] |
| `tuner/min_wall_dist_m` | 0.20 | `min_wall:` (노드 기본) | margin 차감 뒤 저장되는 좌우 가용 corridor 최소 여유 [m] |
| `tuner/corridor_margin_m` | 0.15 | `corridor_margin:` (파일명에서 자동) | map raycast 실벽 거리에서 뺄 reserve [m] |
| `tuner/v_max_mps` | 12.0 | `v_max:` (파일명에서 자동) | 속도 재계산 상한 [m/s] |
| `tuner/mu` | 0.85 | `mu:` (파일명에서 자동) | 마찰계수 |
| `tuner/a_acc_max` | 5.5 | — (launch 편집 필요) | 최대 가속 [m/s²] |
| `tuner/a_dec_max` | 7.0 | — (launch 편집 필요) | 최대 감속 [m/s²] |
| `tuner/vel_handle_scale` | 0.3 | — (launch 편집 필요) | 속도 핸들 높이 = v × scale [m] |

`sim_race.launch.py tuner:=true` 는 CSV 경로와 파일명의
v_max/corridor_margin/mu를 넘기며, stride/falloff/min_wall은 노드 기본값이다.

---

## 잘 안 될 때

| 증상 | 원인 / 확인 |
|---|---|
| `raceline CSV가 없습니다` / `가능한 맵: …` | launch가 CSV를 못 찾은 것. `map:=` 이름 확인, 또는 `csv:=` 직접 지정 |
| `맵이 없습니다: ….yaml` | 에러에 찾아본 폴더와 가능한 맵 목록이 찍힌다. `~/IFAC_ws/maps` 에도 `src/planning_pkg/maps/` 에도 없거나, 후자에 넣고 `colcon build` 를 안 한 것 (맵은 복사됨) |
| `CSV 9열 아님` | 센터라인/벽 CSV를 넘긴 것. `config/global_trajectory/` 의 `_iqp_`/`_sp_` 파일이어야 함 |
| 핸들은 보이는데 라인이 안 움직임 | 최신 map raycast의 벽 여유 클램프에 걸린 것. 로그의 해당 s와 map 정합을 확인할 것. `min_wall`을 차 반폭 0.15 m 아래로 낮추지 말 것 |
| "속도 재계산" 눌렀더니 속도가 확 바뀜 | `v_max:` / `mu:` 가 그 라인 생성값과 다름. 되돌리려면 우클릭 → 실행취소 |
| `CSV 저장`이 거부됨 | 로그의 최초 실패를 확인. map 미수신, min_wall/footprint 침범, 끝→시작 ax 불일치, 또는 배포 제어기 폐루프 추종 실패면 fail-closed로 기존 파일을 보존한다 |
| 미리보기 라인이 사라짐 | 2초마다 재발행하므로 잠깐 기다리면 복구됨 (RViz 재시작 대비) |
| 저장했는데 주행이 그대로 | `./build_all.sh` 를 안 했거나, `tuner/output_csv` 로 다른 파일에 저장해 `params.yaml` 과 어긋남. 주행 로그의 `Loading trajectory from:` 경로 확인 |
