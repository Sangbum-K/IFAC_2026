# 맵 폴더 (워크스페이스 공용) — 새 맵으로 레이스라인 만들기

이 폴더가 **맵의 단일 위치**다. `mapp`이 여기에 저장하고,
sim / localization / raceline_tuner / build_map 이 전부 여기서 읽는다.
맵은 패키지에 설치되지 않으므로 사본을 만들지 말 것 — 같은 이름 맵이
두 벌이면 어느 쪽이 쓰였는지 조용히 갈린다.

## 🚀 시뮬이면 이거 한 줄이면 끝 — `simm <맵이름>`

`maps/`에 `<맵>.pgm`(또는 `.png`) + `<맵>.yaml`을 넣고:

```bash
simm my_track0807          # CSV 생성 -> ./build_all.sh sim -> 시뮬 실행
simm my_track0807 --regen  # CSV 가 있어도 지금 상수로 다시 생성
simm my_track0807 --dir cw # 시계방향 트랙
simm --help                # 옵션 + 사용 가능한 맵 목록
```

`~/IFAC_ws/simm.sh` 가 아래 2~3단계(mkcsv + 빌드 + 실행)를 순서대로 한다.
**어떤 launch에도 기본 맵이 없다** — 맵 이름은 항상 호출자가 준다.
실차(`rc`)나 튜너(`tune`)는 아래 절차를 그대로 쓴다.

---

`maps/`에 `<맵>.png`(또는 `.pgm`) + `<맵>.yaml`을 넣은 뒤, 아래를 순서대로 실행.
(예시는 맵 이름이 `my_map`)

> 이미지 포맷은 png가 아니어도 된다. `build_map.py`는 yaml의 `image:` 필드를
> PIL로 열기 때문에 `.pgm`도 그대로 쓴다 (변환 불필요).

### 0. 파이썬 (2026-08-07 갱신 — venv 불필요)

`build_map.py`는 그냥 `/usr/bin/python3`로 돈다. 시뮬 데스크탑의 user-site
(`~/.local`)에 numpy 1.26.4 / scipy 1.15.3 / scikit-image 0.25.2 /
matplotlib 3.10.9 가 서로 맞게 깔려 있고, **skimage는 거기에만 있다.**

> ⚠️ **`PYTHONNOUSERSITE=1`을 붙이지 말 것.** 예전 문서(user-site numpy 2.2.6
> 시절)의 지시인데, 지금 그걸 붙이면 apt numpy 1.21로 내려가면서 skimage를
> 통째로 잃어 `ModuleNotFoundError: No module named 'skimage'`로 죽는다.
> `.venv` / `make_raceline.sh` 도 더 이상 필요 없다.

같은 이유로 `./build_all.sh sim`은 user-site numpy를 **경고만 하고 통과**한다
(gym의 numba도 그 스택에 맞춰져 있다). 실차 빌드(`./build_all.sh`)에서는
여전히 fail이다 — particle_filter/range_libc C확장이 실제로 깨지기 때문.

### 1. 맵 만들기 (SLAM) — `mapp <이름>`
```bash
mapp my_track0727     # 조이스틱으로 한 바퀴 -> Ctrl+C
                      # -> ~/IFAC_ws/maps/my_track0727.{pgm,yaml}
```
이미 맵 파일이 있으면 **이 폴더(`~/IFAC_ws/maps/`)** 에 두고
yaml의 `image:` 줄이 실제 이미지 파일명과 같은지 확인:
```bash
grep image ~/IFAC_ws/maps/my_track0727.yaml
```

### 2. 레이스라인 생성 — `mkcsv <이름> [cw|ccw]`
```bash
mkcsv my_track0727 ccw    # 기본 CCW. 시계방향 트랙만 cw
```
맵은 이 폴더(`~/IFAC_ws/maps`) 한 곳에서 찾는다. 출력은
`config/global_trajectory/<맵>_iqp_*.csv`(+`_sp_`), `config/centerline/<맵>_centerline.csv`,
그리고 확인용 `scripts/global_traj_sim/outputs/<맵>_trajectory.png`.

(alias 없이 직접: `cd ~/IFAC_ws/planning_ws/src/planning_pkg/scripts/global_traj_sim &&
python3 build_map.py my_track0727 --dir ccw` — §0 참고, venv/PYTHONNOUSERSITE 쓰지 말 것)

### 3. 빌드 + source
```bash
cd ~/IFAC_ws && ./build_all.sh
sa                    # zsh에서 setup.bash는 깨진다 — setup.zsh/sa 를 쓸 것
```
CSV는 빌드 때 `install/` 로 들어간다(복사 또는 심링크). **새로 만든 CSV는 이
단계를 거쳐야** 런타임 노드가 찾는다 — 건너뛰면 launch가 "raceline 없음"으로 멈춘다.
그래서 `simm` 도 "CSV 생성 → 빌드 → 실행" 순서다 (빌드를 먼저 하면 방금 만든
CSV가 설치되지 않는다).

**맵 자체는 빌드가 필요 없다.** 맵은 패키지에 설치되지 않고 launch가 이 폴더를
직접 읽는다 (sim / localization / tuner / build_map 전부 동일). 맵을 추가하거나
고쳤을 때 재빌드해야 하는 건 raceline·centerline CSV 쪽뿐이다.

### 4. RViz 튜너 (선택)
```bash
tune my_track0727     # = ros2 launch planning_pkg raceline_tuner.launch.py map:=my_track0727
```
CSV는 `<맵>_iqp_*.csv` 로 자동 탐색, 배경 맵도 이 폴더에서 바로 읽는다.
`v_max`/`mu` 는 CSV 파일명에서, `stride`/`falloff`
는 트랙 전장에서 자동으로 정해지므로 맵 이름 말고는 넘길 게 없다.
편집 후 저장했으면 `./build_all.sh` 를 다시.

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
