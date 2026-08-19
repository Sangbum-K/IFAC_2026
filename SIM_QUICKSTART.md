# 시뮬레이터 실행 (심 전용)

gym 위에서 perception → planning → control 스택을 돌린다.
ROS 2 Humble·`range_libc`·`f1tenth_gym` 등 의존성은 이미 깔려 있다고 가정한다
(처음 세팅하는 머신이면 `README.md` 의 빌드 절, 안 깔린 게 있으면 8장).

쉘은 **zsh**. alias 없이 저장소 스크립트를 그대로 쓴다. 실차는 `HOW_TO_RUN.md`.

---

## 1. 빌드

```zsh
cd ~/IFAC_ws
./setup_deps.sh sim     # 머신당 1회
./build_all.sh sim      # 매번
```

`build_all.sh sim` 이 gym(`sim_ws`)까지 같이 빌드한다.

---

## 2. 실행

```zsh
./run_sim.sh my_track0817
```

source·프로세스 정리는 `run_sim.sh` 가 알아서 한다. **맵 이름은 필수다.**

바로 쓸 수 있는 맵 (레이스라인이 있는 것):

```zsh
ls planning_ws/src/planning_pkg/config/global_trajectory/*_iqp_*.csv \
  | sed 's#.*/##; s/_iqp_.*//' | sort -u
```

---

## 3. 옵션

```zsh
./run_sim.sh my_track0817 --plain      # gym 기본 물리 + GT odom (빠른 확인)
./run_sim.sh my_track0817 --gt         # 위치추정만 GT
./run_sim.sh my_track0817 --no-rviz    # RViz 없이
./run_sim.sh my_track0817 map_filter:=false   # 브릿지 벽필터 끄고 A/B
```

그 외 인자는 `ros2 launch` 로 그대로 넘어간다 (`suffix:=…` 등).

### 동적 장애물 인지 확인 (RViz2)

`--plain`으로 빠르게 띄운 뒤 RViz 상단의 **2D Goal Pose**로 레이스라인 위,
ego 차량 전방 3–8 m 지점에 상대 차량을 스폰한다.

```zsh
./run_sim.sh my_track0817 --plain
```

`DetectedObstacles (LiDAR)` 디스플레이가 실제 C++ 인지 결과를 보여 준다.
상대 차량의 LiDAR 반사가 `/scan_overlay → obstacle_detector_node`를 통과한 뒤
약 0.8초 동안 추적되면, `is_dynamic=true`로 확정되어 **빨간 원기둥**, 노란 속도
화살표, `#<id> D <speed>m/s` 텍스트가 나타난다. 이 마커가 동적 인지 확인 대상이다.

`Publish Point`로 추가한 `VirtualStaticObstacles (GT)`의 초록 원기둥은 정적
ground truth라서 동적 인지 결과와 구분된다. 입력 스캔도 `ScanOverlay`에서 함께
볼 수 있다.

터미널에서 판별값을 직접 확인하려면 다음을 실행한다.

```zsh
ros2 topic echo /perception/obstacles_detected
```

---

## 4. 새 맵

`maps/` 에 `<맵>.pgm` + `<맵>.yaml` 을 넣고 (yaml 의 `image:` 가 실제 파일명과 같아야 함):

```zsh
./simm.sh my_track0813              # 레이스라인 생성 → 빌드 → 실행
./simm.sh my_track0813 --regen      # 지금 상수로 레이스라인 다시 생성
./simm.sh my_track0813 --dir cw     # 시계방향 트랙 (기본 ccw)
```

레이스라인 CSV 는 `colcon build` 가 `install/` 로 복사하므로 순서가
"생성 → 빌드 → 실행"이다. 맵 파일만 고쳤으면 재빌드는 필요 없다.

---

## 5. 두 프로파일

파라미터는 **둘이 같고**(전부 `real_car.yaml` 값), 차이는 위치추정과 차량물리다.

| | 기본 (`sim_real_car`) | `--plain` (`sim`) |
|---|---|---|
| 위치추정 | particle_filter (실차와 동일) | gym GT odom |
| 차량물리 | mu 0.90 / a_max 4.5 / 조향 좌 0.3800·우 -0.3578 | mu 1.0489 / a_max 9.51 / s_max ±0.4189 |
| LiDAR | 40 Hz / 라이다 x 0.28 m (실차 실측) | gym 기본 250 Hz / 0.275 m |

빔 수(1080)와 FOV(4.7 rad)는 두 프로파일 모두 gym 고정이다 — 실차는 1081빔 /
4.712389 rad 이지만 f110_gym 이 하드코딩이라 못 바꾼다. 남는 각분해능 차이는
0.17% 라 인지의 점수 문턱은 그대로 옮겨 쓸 수 있다.
| 용도 | **실차 판정 근거로 쓸 수 있는 쪽** | 빠른 확인 |

`--plain` 은 PF 가 없어서 `localization_gate` 가 꺼진다(켜면 출발 못 함).

---

## 6. 파라미터를 고쳤을 때

config 는 `install/` 로 복사되므로 재빌드해야 반영된다.

```zsh
./build_all.sh sim
```

sim 프로파일이 실차와 어긋나지 않는지 확인:

```zsh
python3 -m pytest planning_ws/src/planning_pkg/test -q
```

---

## 7. 문제 해결

| 증상 | 조치 |
|---|---|
| `맵 이름이 필요합니다` | 맵 이름을 줄 것 (2장) |
| `raceline 없음` 으로 죽음 | 그 맵의 CSV 가 없다 → `./simm.sh <맵>` |
| iqp CSV 가 여러 벌이라 거부 | `suffix:=max7.00ms_margin0.08m_mu0.90` 처럼 명시 |
| `<ws>_ws가 빌드되지 않았습니다` | `./build_all.sh sim` |
| 차가 출발 안 함 (기본 프로파일) | PF 미수렴 → `/pf/localization_quality` 확인 |
| 파라미터를 고쳤는데 그대로 | 6장 (재빌드) |
| `⚠ 저장소 밖의 gym 사본을 씁니다` | 아래 참조 |

> ⚠️ **gym 사본 경고가 뜨면 무시하지 말 것.**
> 저장소의 `sim_ws/src/f1tenth_gym_ros` 는 upstream 이 아니라 로컬 수정본이라
> `vehicle.*` 와 `sensor_publish_rate_hz` 를 받는다. 홈의 옛 사본(`~/sim_ws`)이
> 잡히면 그 파라미터가 **조용히 무시되어** 실차 물리가 안 걸린 채 돈다.
> `./build_all.sh sim` 을 돌리면 저장소 사본이 잡힌다.
> 실행 로그의 `차량 물리: mu=0.9 a_max=4.5 …` 줄로 확인할 수 있다.

---

## 8. 의존성이 빠졌을 때

| 증상 | 조치 |
|---|---|
| `range_libc 를 찾을 수 없습니다` | `git clone https://github.com/f1tenth/range_libc ~/range_libc`<br>다른 곳에 있으면 `RANGE_LIBC=<경로> ./build_all.sh sim` |
| `ModuleNotFoundError: f110_gym` 또는 `No module named 'gym'` | `git clone https://github.com/f1tenth/f1tenth_gym ~/f1tenth_gym`<br>`cd ~/f1tenth_gym && python3 -m pip install --user -e .`<br>(`gym`·`numba`·`pyglet` 이 여기 딸려 온다. 설치 후 `python3 -c 'import numpy; print(numpy.__file__)'` 가 `dist-packages` 인지 확인) |
| launch 가 `file not found: [Errno 2] ... 'xacro'` 로 즉사 | `sudo apt install ros-humble-xacro`<br>`robot_description` 을 `Command(['xacro ', ...])` 로 만드므로 **실행파일**이 있어야 한다. 없으면 노드 하나가 죽는 게 아니라 launch 가 통째로 죽는다. |
| `build_map.py 의존성 부족` | `python3 -m pip install --user numpy scipy pillow pyyaml matplotlib scikit-image` |
| 랩타임·CTE 오버레이가 안 뜸 | `sudo apt install ros-humble-rviz-2d-overlay-msgs ros-humble-rviz-2d-overlay-plugins` 후 재빌드 |
| `source setup.bash` 오류 | zsh 에서는 `setup.zsh` |

> ⚠️ `PYTHONNOUSERSITE=1` 을 걸면 안 된다. gym 의 numba 와 `build_map.py` 의
> skimage 가 user-site 에 있어서, 걸면 apt numpy 1.21 로 내려가며 둘 다 잃는다.
