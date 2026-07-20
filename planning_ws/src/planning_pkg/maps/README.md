# 새 맵으로 레이스라인 만들기

`maps/`에 `<맵>.png` + `<맵>.yaml`을 넣은 뒤, 아래를 순서대로 실행.
(예시는 맵 이름이 `my_map`)

### 1. yaml의 image 파일명이 실제 png와 같은지 확인
```bash
grep image ~/IFAC_ws/planning_ws/src/planning_pkg/maps/my_map.yaml
```

### 2. 레이스라인 생성
```bash
cd ~/IFAC_ws/planning_ws/src/planning_pkg/scripts/global_traj_sim
~/IFAC_ws/.venv/bin/python build_map.py my_map
```

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
ros2 launch planning_pkg raceline_tuner.launch.py \
    csv:=my_map_iqp_max6.00ms_margin0.15m_mu0.75.csv map:=my_map stride:=10 falloff:=3.0
```

### 6. 튜너로 편집했으면 다시 빌드
```bash
cd ~/IFAC_ws/planning_ws && colcon build --packages-select planning_pkg
```

---

## 파라미터 (build_map.py 상단 `설정` 블록)

| 상수 | 기본 | 의미 |
|------|------|------|
| `V_MAX` | 6.0 | 최대 속도 [m/s] |
| `MU` | 0.75 | 노면 마찰 |
| `SAFE_DIST` | 0.25 | 라인-벽 최소 이격 [m] |
| `MARGIN` | 0.15 | 벽 여유 [m] |
| `INFIELD_FILL_M` | 0.8 | infield 오목한 곳 메우기 [m] (0=끄기) |
| `RESOLUTION_OVERRIDE` | None | yaml resolution 덮어쓰기 |

> CSV 파일명은 `<맵>_iqp_max<V_MAX>ms_margin<MARGIN>m_mu<MU>.csv` 규칙.
> V_MAX/MARGIN/MU를 바꾸면 5번의 `csv:=` 이름도 그에 맞게 바꿀 것.
