# 새 맵으로 레이스라인 만들기

`maps/`에 `<맵>.png`(또는 `.pgm`) + `<맵>.yaml`을 넣은 뒤, 아래를 순서대로 실행.
(예시는 맵 이름이 `my_map`)

> 이미지 포맷은 png가 아니어도 된다. `build_map.py`는 yaml의 `image:` 필드를
> PIL로 열기 때문에 `.pgm`도 그대로 쓴다 (변환 불필요).

### 0. venv (이 기계 한정 주의사항)

`python3-venv`(ensurepip)가 없어서 pip 있는 venv를 못 만든다. 시스템
패키지를 그대로 쓰는 venv로 만든다:

```bash
cd ~/IFAC_ws && python3 -m venv --system-site-packages --without-pip .venv
```

그리고 **실행할 때 반드시 `PYTHONNOUSERSITE=1`을 붙일 것.** `~/.local`에
numpy 2.2.6이 사용자 설치로 들어 있는데, dist-packages의 scipy·matplotlib은
numpy 1.x로 컴파일돼 있어서 그대로 두면 `_ARRAY_API not found` /
`numpy.core.multiarray failed to import`로 죽는다. 이 변수를 주면 user-site가
빠지면서 numpy 1.21.5 + scipy 1.8.0 + matplotlib 3.5.1 조합이 잡힌다.

(같은 이유로 `make_raceline.sh`는 이 기계에서 쓰지 말 것 — `PYTHONNOUSERSITE`를
안 붙이는 데다 구 파이프라인인 `step4_main.py`를 부른다.)

### 0-1. scikit-image 설치 (한 번만) — ⚠️ numpy 함정

`build_map.py`는 medial-axis 중앙선 추출에 `skimage.morphology.skeletonize`를
쓴다. 없으면 `ModuleNotFoundError: No module named 'skimage'`로 즉사한다.

`.venv/bin/pip`은 없지만 dist-packages의 pip이 보이므로 `python -m pip`으로
venv 안에 설치된다. **버전을 반드시 고정할 것** — 최신 scikit-image는 numpy
2.x를 요구해서 pip이 venv에 numpy 2.2.6을 같이 깔아버리고, 그러면 0번의
`PYTHONNOUSERSITE=1`이 무의미해지면서 dist-packages scipy 1.8이 깨진다.

```bash
cd ~/IFAC_ws
PYTHONNOUSERSITE=1 .venv/bin/python -m pip install "scikit-image>=0.19,<0.20"
# ↑ 그래도 numpy 2.x를 딸려 깐다. 반드시 되돌릴 것:
PYTHONNOUSERSITE=1 .venv/bin/python -m pip uninstall -y numpy
```

`uninstall numpy`는 venv 안의 2.2.6만 지운다(dist-packages 1.21.5는 pip이
"outside environment"라 못 건드림) → 다시 시스템 numpy 1.21.5로 떨어진다.
확인:

```bash
PYTHONNOUSERSITE=1 ~/IFAC_ws/.venv/bin/python -c \
  "import numpy,scipy,skimage; print(numpy.__version__, scipy.__version__, skimage.__version__)"
# 1.21.5 1.8.0 0.19.3  ← 이 조합이 정상
```

### 1. yaml의 `image:`가 **maps/ 안의 실제 파일**을 가리키는지 확인

```bash
cd ~/IFAC_ws/planning_ws/src/planning_pkg/maps
grep image my_map.yaml && ls -l my_map.*
```

`build_map.py`는 `image:` 경로를 **yaml이 있는 디렉터리 기준**으로 연다.
SLAM(`map_saver`)이 `~/maps/`에 저장한 걸 복사해 올 때 파일명만 바꾸고 yaml은
그대로 두면 `FileNotFoundError: .../maps/<딴이름>.pgm`으로 죽는다.
`~/maps/`에 있으니까 되겠지 — 안 된다. 이름을 맞추거나 이미지를 같이 복사할 것.

### 2. 레이스라인 생성
```bash
cd ~/IFAC_ws/planning_ws/src/planning_pkg/scripts/global_traj_sim
PYTHONNOUSERSITE=1 ~/IFAC_ws/.venv/bin/python build_map.py my_map
```

`scipy.interpolate`의 `RuntimeWarning: Setting x[0][66]=x[0][0]` 무더기는
주기 스플라인(`per=1`)이 시작/끝점을 맞추는 정상 동작이다. 무시해도 된다.

로그에서 볼 것:

| 줄 | 의미 |
|---|---|
| `트랙 크기(yaml resolution ...)` | 실측 트랙과 다르면 yaml `resolution`이 틀린 것 |
| `IQP iter n/3: max\|alpha\|` | 반복마다 줄어야 정상 (수렴) |
| `[warn] 최종 벽 여유 0.00 m` | 그 라인이 벽에 붙었다는 뜻 — 아래 5-1 참고 |
| `[dir] 주행 방향 CW/CCW · 출발점 ... 헤딩 ...` | **아래 2-1 반드시 확인** |
| `lap time / \|lat err\| max` | lat err가 크면 실차 조향으로 못 따라가는 라인 |

### 2-1. ⚠️ 주행 방향 확인 (건너뛰지 말 것)

레이스라인은 폐루프라 **어느 쪽으로 도는지**가 CSV에 그대로 박힌다. 방향이
실차 배치와 반대면 `/local_waypoints`가 전부 차 뒤로 깔리고, 컨트롤러
(`control_node.cpp`의 `asin` 기반 eta)는 그걸 못 알아채고 조용히 직진한다.
증상은 "lookahead point가 차가 보는 방향과 반대".

`--dir`를 안 주면 **map (0,0) 최근접점에서 +x를 향하도록** 자동 판정한다.
이건 map origin이 *SLAM을 시작할 때의 차량 pose*일 때만 맞다.
**yaml의 `origin`을 손으로 고쳤다면 그 전제가 깨지므로 반드시 명시할 것:**

```bash
PYTHONNOUSERSITE=1 ~/IFAC_ws/.venv/bin/python build_map.py my_track --dir ccw
```

확인 방법 — `outputs/<맵>_trajectory.png`의 **빨간 화살표(진행 방향)와 굵은
파란 화살표(출발 헤딩)** 를 실차를 놓을 자세와 대조한다. 제목에도
`travel dir CW/CCW, start heading ±NN deg`가 찍힌다.

> 폐루프 시뮬(`lap time`, `lat err`)로는 **절대 못 잡는다.** 시뮬은 초기 헤딩을
> 궤적 자신에서 가져오므로 방향이 뒤집혀 있어도 완벽하게 돈다.
> `_tracking.png`가 깨끗한 건 방향이 맞다는 근거가 아니다.

### 3. 빌드
```bash
cd ~/IFAC_ws/planning_ws
colcon build --packages-select planning_pkg
```

### 4. source
```bash
source /opt/ros/humble/setup.bash
source ~/IFAC_ws/planning_ws/install/setup.bash
```

### 5. RViz 튜너 실행
```bash
ros2 launch planning_pkg raceline_tuner.launch.py map:=<맵이름>
```
CSV는 `<맵>_iqp_*.csv` 로 자동 탐색된다. 기본값(stride 10 / falloff 3.0 /
v_max 6.0 / mu 0.75)은 **my_track 기준**이니, 다른 조건으로 뽑은 라인이면
`v_max:` / `mu:` 를 그 값에 맞출 것 — 사용법은
[`../scripts/tuner/README.md`](../scripts/tuner/README.md).

### 6. 튜너로 편집했으면 다시 빌드
```bash
cd ~/IFAC_ws/planning_ws && colcon build --packages-select planning_pkg
```

---

## 생성물

| 경로 | 내용 |
|---|---|
| `config/global_trajectory/<맵>_iqp_*.csv` | 주행용 레이스라인 (런타임·튜너가 읽음) |
| `config/global_trajectory/<맵>_sp_*.csv` | SP 추월 라인 (`MAKE_SP=True`일 때) |
| `config/centerline/<맵>_centerline.csv`, `_walls.csv` | 중앙선·벽 (회피 계층용) |
| `scripts/global_traj_sim/outputs/<맵>_trajectory.png` | 라인+속도+시뮬 궤적 |
| `scripts/global_traj_sim/outputs/<맵>_tracking.png` | 속도 추종·횡오차·조향각 |

`outputs/`에도 CSV 사본이 떨어지지만 **런타임이 읽는 건 `config/` 쪽**이다.

---

## 파라미터 (build_map.py 상단 `설정` 블록)

CLI 옵션은 `--dir cw|ccw`(주행 방향, 위 2-1 참고) 하나뿐이다.
나머지는 전부 파일 상단 상수를 고쳐서 다시 돌린다.

| 상수 | 기본 | 의미 |
|------|------|------|
| `V_MAX` | 6.0 | 최대 속도 [m/s] |
| `MU` | 0.75 | 노면 마찰 |
| `SAFE_DIST` | 0.25 | IQP 라인-벽 최소 이격 [m] = 차 반폭(≈0.15)+여유 |
| `MARGIN` | 0.08 | CSV의 `d_left`/`d_right`에서 더 빼는 여유 [m] |
| `RACELINE` | `"iqp"` | `iqp`(최소곡률) / `sp`(최단거리) / `center` |
| `MAKE_SP` | True | iqp일 때 SP 추월 라인도 같이 생성 |
| `INFIELD_FILL_M` | 0.8 | infield 오목한 곳 메우기 [m] (0=끄기) |
| `MIN_ISLAND_M2` | 0.3 | 이보다 작은 free 구멍(SLAM 스페클)은 벽으로 메움 [m²] |
| `RESOLUTION_OVERRIDE` | None | yaml resolution 덮어쓰기 |
| `EXPECTED_SIZE_M` | (20, 8) | 참고 기대 크기 — 벗어나면 경고만 |

> `SAFE_DIST`는 라인을 실제로 밀어내고, `MARGIN`은 라인을 안 옮기고 CSV의
> 폭 보고값만 줄인다. 즉 **주행 안전은 `SAFE_DIST`, 회피 여유는 `MARGIN`.**

> CSV 파일명은 `<맵>_iqp_max<V_MAX>ms_margin<MARGIN>m_mu<MU>.csv` 규칙.
> V_MAX/MARGIN/MU를 바꾸면 5번의 `csv:=` 이름도 그에 맞게 바꿀 것.

### 5-1. `[warn] 최종 벽 여유 0.00 m`가 뜨면

좁은 트랙에서 SP(최단거리) 라인이 코너 안쪽 벽에 붙었다는 뜻이다. IQP 라인에
안 떴으면 **주행용 라인은 멀쩡**하고, 추월용 SP 라인만 벽에 붙은 것 —
추월 계층을 켤 거면 `SAFE_DIST`를 올리거나 `MAKE_SP=False`로 끄고 쓸 것.

---

## 현재 대회 맵: `my_track` (2026-07-24 생성 기준)

`my_track.png` + `my_track.yaml` (resolution 0.05, origin `[-7.95, -15.9, 0]`).

```
트랙 크기 15.2 x 6.7 m   (EXPECTED_SIZE_M ~20x8 참고범위보다 작아 경고 없음)
centerline 329 pts, 32.8 m
IQP  max|kappa| 0.956 1/m,  경로 28.4 m
SP   경로 27.0 m,  [warn] 벽 여유 0.00 m  ← 추월 라인만 벽에 붙음
시뮬 lap 6.45 s, v_mean 4.37 / v_max 6.00 m/s, |lat err| max 0.222 m
```

생성 CSV: `my_track_iqp_max6.00ms_margin0.08m_mu0.75.csv` (+ `_sp_`).
