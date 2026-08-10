# IFAC_ws

F1TENTH 자율주행 스택. ROS 2 Humble, 쉘은 **zsh**.

## 워크스페이스

| 경로 | 내용 |
|---|---|
| `hardware_ws/` | f1tenth_stack(VESC·joy teleop·mux·LiDAR bringup), ackermann_mux, vesc, urg_node. (`vehicle_bringup`은 legacy) |
| `localization_ws/` | localization_bringup, particle_filter — 위치추정(`backend:=particle_filter`가 `/pf/pose/odom`+map→odom TF) |
| `perception_ws/` | perception_pkg — LiDAR 장애물 탐지 |
| `planning_ws/` | planning_pkg(레이스라인 추종+회피), monitor_pkg, pitwall_panel, **메시지 정의**(f1tenth_planning_msgs, f1tenth_perception_msgs) |
| `control_ws/` | control_pkg — pure pursuit(ETH MAP) 경로 추종 제어기 |
| `integration_ws/` | integration_pkg — 전체 스택 통합 launch + 헬스 모니터 |

전체 실행 절차는 `README.md` / `HOW_TO_RUN.md` 참고.

## 차량 캘리브레이션

**`TUNING_STATUS.md`가 조향/속도 캘리브레이션의 단일 기준 문서.**
캘리브레이션 관련 작업을 할 때는 먼저 읽을 것.

현재 상태:
- **조향 — 2026-08-10 타이어 교체 후 재적합, 실차 확인됨.**
  `gain -0.8320` / `offset 0.3941`, 대칭 최대 조향 ±21.07°(0.3677 rad).
  플래닝·컨트롤은 0.36으로 내림해서 쓴다 (네 곳이 같아야 함, `TUNING_STATUS.md` §1).
  ⚠️ 옛 값 `-0.8206`/`0.403`이 문서 여기저기 남아 있으니 값은 vesc.yaml을 볼 것.
  ⚠️ **미해결**: 조향 중립이 런 사이 최대 2.5° 튄다 — 기계 슬립이라 yaml로 못 잡음
  (`TUNING_STATUS.md` §11). 소프트 보정 시도 금지.
- **속도 — 마진 소멸이 최우선 과제.** 타이어 교체로 유효 구름둘레 +3.9%,
  `speed_to_erpm_gain: 4255.1`은 그대로 → **08-09까지의 모든 실차 검증이 서 있던
  "명령보다 4% 느림" 여유가 사라졌다.** 플래닝이 `velocity_scale 0.96`으로 임시
  상쇄 중. 근본 해결은 08-10 이후 백으로 A_ACC/A_DEC를 **odom 아닌 PF 기준**으로
  재측정 → `deployment_profiles.py` 갱신 → 궤적 재생성 (`TUNING_STATUS.md` §11).
- **속도 캘리브레이션 (`speed_to_erpm_gain` 자체)** — 그다음. 함정 3가지가 §5 ⑤에 있음

`hardware_ws/src/f1tenth_stack/config/vesc.yaml`이 hardware_ws 전체의
single source of truth. **launch 인자로 덮어쓰지 말 것** — yaml과 어긋나서
측정을 통째로 날린 적이 있음.

## 캘리브레이션 도구

`hardware_ws/tools/` (홈 디렉터리에 `~/steer_calib.py` 등으로 심볼릭 링크됨).
vesc.yaml을 직접 읽으므로 상수를 손으로 맞출 필요 없음.

```bash
python3 ~/steer_calib.py limits    # 포화 없는 조향각 범위
python3 ~/steer_calib.py plan      # 각도별 필요 원 지름
python3 ~/steer_calib.py fit ...   # 좌우 원으로 gain+offset 피팅
python3 ~/steer_stand.py sweep ... # 스탠드 테스트 (speed 0 고정)
```

## 실차 작업 시 주의

- **측정 전 프로세스 중복 확인.** ROS 2는 같은 이름 노드가 중복 실행돼도 에러를
  안 냄. `ackermann_to_vesc_node`가 둘이면 서로 다른 값을 같은 토픽에 써서
  서보가 떨림. `TUNING_STATUS.md` §0에 절차와 실제 사고 기록이 있음.
- **`ros2 param set`은 안 먹음.** `ackermann_to_vesc.cpp`가 생성자에서만 읽음
  → launch 재시작 필요.
- **`/odom`만으로 속도를 검증하면 안 됨.** 같은 eRPM gain을 사용하므로 줄자 같은
  외부 기준으로 측정할 것. 조향 방향은 현재 `use_imu_yaw: true`에서 IMU 상대 yaw를
  사용하므로 서보 명령 모델과 독립적으로 확인할 수 있다.
- **zsh에서 `source /opt/ros/humble/setup.bash`는 깨짐.** `setup.zsh`를 쓸 것.
- launch 전 `ls -l /dev/ttyACM0` 확인. 포트가 없으면 `vesc_driver_node`가
  조용히 죽고 자동 복구되지 않음.
