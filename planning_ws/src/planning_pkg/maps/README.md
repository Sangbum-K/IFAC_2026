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

(같은 이유로 `make_raceline.sh`는 이 기계에서 안 된다 — venv에 pip이 없다.)

`skimage`(중앙선 skeletonize)는 dist-packages에 없다. venv에 pip이 없으므로
시스템 pip으로 **별도 폴더에** 받아서 `PYTHONPATH`로 얹는다. numpy를 딸려
올리지 않도록 `--no-deps`로 버전을 고정할 것 (numpy 1.21.5 유지가 핵심):

```bash
pip install --target=~/IFAC_ws/.venv/skimage_target --no-deps \
    "scikit-image==0.19.3" "imageio==2.19.3" "tifffile==2021.11.2" \
    "PyWavelets==1.3.0" "networkx==2.6.3"

# 이후 build_map 실행은 항상 이 두 변수를 같이:
PYTHONNOUSERSITE=1 PYTHONPATH=~/IFAC_ws/.venv/skimage_target \
    ~/IFAC_ws/.venv/bin/python build_map.py <맵> --dir ccw
```

### 1. 맵 만들기 (SLAM) — `mapp <이름>`
```bash
mapp my_track0727     # 조이스틱으로 한 바퀴 -> Ctrl+C
                      # -> ~/IFAC_ws/maps/my_track0727.{pgm,yaml}
```
이미 맵 파일이 있으면 `~/IFAC_ws/maps/` 또는 이 폴더(`planning_pkg/maps/`)에 두고
yaml의 `image:` 줄이 실제 이미지 파일명과 같은지 확인:
```bash
grep image ~/IFAC_ws/maps/my_track0727.yaml
```

### 2. 레이스라인 생성 — `mkcsv <이름> [cw|ccw]`
```bash
mkcsv my_track0727 ccw    # 기본 CCW. 시계방향 트랙만 cw
```
맵은 `~/IFAC_ws/maps` → `planning_pkg/maps` 순으로 찾는다. 출력은
`config/global_trajectory/<맵>_iqp_*.csv`(+`_sp_`), `config/centerline/<맵>_centerline.csv`,
그리고 확인용 `scripts/global_traj_sim/outputs/<맵>_trajectory.png`.

(alias 없이 직접: `PYTHONNOUSERSITE=1 ~/IFAC_ws/.venv/bin/python build_map.py my_track0727 --dir ccw`)

### 3. 빌드 + source
```bash
cd ~/IFAC_ws && ./build_all.sh
sa                    # zsh에서 setup.bash는 깨진다 — setup.zsh/sa 를 쓸 것
```
CSV는 심링크가 아니라 **빌드 시 복사**된다. 이 단계를 건너뛰면 런타임 노드가
CSV를 못 찾아 launch가 멈춘다.

### 4. 속도 튜너 (선택, 웹)
```bash
tune my_track0727     # = python3 tools/racelinetuner.py my_track0727
```
브라우저 도구이고 **속도 프로파일만** 편집한다 (라인 기하는 `mkcsv`/build_map.py).
CSV는 `<맵>_iqp_*.csv` 로 자동 탐색하므로 맵 이름 말고는 넘길 게 없다.
원본은 건드리지 않고 `<맵>_tuned_iqp_*.csv` 로 저장하니,
편집 후에는 `./build_all.sh` 다시 하고 `rc <맵>_tuned` 로 띄운다.

### 5. 실차 주행
```bash
rc my_track0727       # localization 맵 + planner CSV 3종이 모두 이 이름으로 잡힘
```

---

## 파라미터 (build_map.py 상단 `설정` 블록)

| 상수 | 기본 | 의미 |
|------|------|------|
| `V_MAX` | 6.0 | 최대 속도 [m/s] |
| `MU` | 0.75 | 노면 마찰 |
| `SAFE_DIST` | 0.40 | 라인-벽 최소 이격 [m] |
| `MARGIN` | 0.08 | 벽 여유 [m] |
| `INFIELD_FILL_M` | 0.8 | infield 오목한 곳 메우기 [m] (0=끄기) |
| `RESOLUTION_OVERRIDE` | None | yaml resolution 덮어쓰기 |

> CSV 파일명은 `<맵>_iqp_max<V_MAX>ms_margin<MARGIN>m_mu<MU>.csv` 규칙.
> 이름 앞부분(`<맵>_iqp_`)만 맞으면 launch는 glob으로 찾으므로 V_MAX/MARGIN/MU를
> 바꿔도 `map:=` 는 그대로 동작한다. 다만 **같은 맵에 옛 CSV가 남아 있으면 둘 다
> 매칭돼 사전순 첫 번째가 잡히므로**(launch가 경고를 찍는다) 옛 파일은 지울 것.
> `params.yaml` 의 하드코딩 기본값(`rc` 를 맵 이름 없이 실행할 때만 쓰임)은
> 파일명 전체가 적혀 있어 자동으로 따라가지 않는다.
