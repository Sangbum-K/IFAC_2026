# Raceline Tuner

RViz에서 레이스라인(라인 + 속도)을 인터랙티브하게 튜닝하는 도구.
(HMCL unicorn `raceline_tuner`의 ROS2 구현)

- 🟣 **보라 구체 드래그** → 라인을 좌우로 이동 (주변이 8m 반경 코사인 falloff로 부드럽게 따라옴, 벽 여유 0.35m 자동 클램프)
- 🟢 **초록 큐브 위/아래 드래그** → 그 구간 속도 조절 (핸들 높이 = 속도)
- **핸들 우클릭 메뉴**: 속도 재계산(마찰한계) / 실행취소(undo) / 전체 리셋 / CSV 저장

---

## ⚠️ 켜기 전에 알아야 할 것

1. **launch 기본 CSV 이름이 실제 파일과 다름.**
   [`launch/raceline_tuner.launch.py`](../../launch/raceline_tuner.launch.py) 기본값은 `..._max7.00ms_...csv` 인데
   실제 `config/global_trajectory/` 엔 `max15.00ms` 파일만 있음.
   → **`csv:=` 로 파일명을 직접 넘겨야 함.** 안 그러면 로드 에러.

2. **맵 이름이 `Spielberg_map` 으로 하드코딩됨.**
   launch가 맵을 `install/share` 의 `Spielberg_map` 으로 고정해서 읽고, 맵은 심링크가 아니라
   **빌드 시 복사**됨. → 새 맵을 쓰려면 `src` 에 넣고 **colcon build** 해야 RViz에 뜸.

---

## 새 맵으로 튜닝하기 (스텝별)

### 1. 새 맵 파일 넣기
받은 맵 `.png` + `.yaml` 을 복사:
```bash
cp /받은경로/새맵.png /받은경로/새맵.yaml \
   ~/IFAC_ws/planning_ws/src/planning_pkg/maps/
```
> `.yaml` 안의 `image:` 줄이 실제 png 파일명과 같은지 확인.

### 2. 맵 이름 맞추기 (둘 중 하나)
- **추천:** 받은 맵을 `Spielberg_map.png` / `Spielberg_map.yaml` 로 **이름만 변경**. launch 수정 불필요.
- **원래 이름 유지:** `launch/raceline_tuner.launch.py` 의 `map_stem` 에 있는 `'Spielberg_map'` 을 새 stem으로 변경.

### 3. raceline CSV 넣기
9열(`;` 구분) raceline CSV를 복사:
```bash
cp /받은경로/새raceline.csv \
   ~/IFAC_ws/planning_ws/src/planning_pkg/config/global_trajectory/
```

### 4. 빌드 (맵 반영 필수)
```bash
cd ~/IFAC_ws/planning_ws
colcon build --packages-select planning_pkg
source install/setup.bash
```

### 5. 튜너 실행 (CSV 파일명 필수)
```bash
ros2 launch planning_pkg raceline_tuner.launch.py csv:=새raceline.csv
```
예: `csv:=Spielberg_map_iqp_max15.00ms_margin0.15m_mu0.85.csv`

### 6. 저장 & 반영
RViz에서 우클릭 → **CSV 저장**.
- 원본은 `*.bak` 으로 자동 백업 후 같은 위치에 덮어씀.
- 저장한 라인을 **런타임 노드가 읽게** 하려면 다시 빌드:
  ```bash
  cd ~/IFAC_ws/planning_ws && colcon build --packages-select planning_pkg
  ```

---

## CSV 포맷 (9열, `;` 구분)

```
s_m ; x_m ; y_m ; d_right ; d_left ; psi_rad ; kappa_radpm ; vx_mps ; ax_mps2
```

| 열 | 의미 |
|----|------|
| `s_m` | 누적 거리 |
| `x_m`, `y_m` | 좌표 |
| `d_right`, `d_left` | 현재 라인 기준 좌/우 남은 트랙 폭 |
| `psi_rad` | 헤딩 |
| `kappa_radpm` | 곡률 |
| `vx_mps` | 속도 |
| `ax_mps2` | 종방향 가속도 |

라인을 옮기면 `s / psi / kappa` 는 자동 재계산되고, **속도 재계산** 메뉴는 마찰한계 +
가감속 한계로 forward-backward 속도 프로파일을 다시 계산함.

---

## 주요 파라미터

launch에서 `parameters` 로 오버라이드 가능 (`raceline_tuner_node.py` 참고):

| 파라미터 | 기본값 | 의미 |
|----------|--------|------|
| `tuner/handle_stride` | 25 | 핸들 간격 (wpt 수) |
| `tuner/falloff_window_m` | 8.0 | 드래그 영향 반경 [m] |
| `tuner/min_wall_dist_m` | 0.35 | 라인–벽 최소 여유 [m] |
| `tuner/v_max_mps` | 12.0 | 최대 속도 |
| `tuner/mu` | 0.85 | 마찰계수 |
| `tuner/a_acc_max` | 5.5 | 최대 가속 |
| `tuner/a_dec_max` | 7.0 | 최대 감속 |
| `tuner/vel_handle_scale` | 0.3 | 속도 핸들 높이 스케일 (높이 = v × scale) |
