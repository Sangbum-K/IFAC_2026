# F1TENTH 조향 캘리브레이션 — 인수인계

최종 갱신: 2026-07-22
작업 위치: `~/IFAC_ws/hardware_ws` (ROS 2 Humble, 쉘은 zsh)

**조향 캘리브레이션 완료 · 최종 검증까지 끝남.** 값은 §1, 검증 근거는 §4.
gain·offset 모두 측정 정밀도 한계에 도달했으므로 **더 손대지 말 것.**
다음 작업은 속도 캘리브레이션(§5 ⑤).

---

## 0. ★ 매 세션 시작 절차 — 프로세스 중복 확인

**측정 전에 반드시 할 것.** 이 절차를 건너뛰어서 하루치 측정을 날린 적이 있음.

### 왜 필요한가 — 실제 사고 기록

직진 검증용으로 `steering_angle_to_servo_offset:=0.407` launch를 띄웠는데,
그걸 죽이지 않고 새 launch를 띄웠음. **ROS 2는 같은 이름의 노드가 중복되어도
막지 않음.** 결과적으로 `ackermann_to_vesc_node`가 2개, `/teleop` 발행자가 2개가
되어 `/commands/servo/position`에 네 값이 번갈아 나왔음:

```
0.403   = 0.403 − 0.8206 × 0.00   (새 노드, δ=0)
0.15682 = 0.403 − 0.8206 × 0.30   (새 노드, δ=0.30)
0.407   = 0.407 − 1.2135 × 0.00   (옛 노드, δ=0)
0.04295 = 0.407 − 1.2135 × 0.30   (옛 노드, δ=0.30)
```

서보가 이 네 위치를 초당 수십 번 왕복해서 **앞바퀴가 심하게 떨렸음.**
서보 고장이 아니라 순전히 프로세스 중복 문제였음.

정리하려고 `pkill -f ackermann_to_vesc`를 실행했는데 **그 노드만 죽고 나머지는
고아 프로세스로 살아남음** (`pkill -f bringup_launch`는 런처만 죽이고 자식 노드는
안 죽임). 세션 종료 시점의 `ros2 node list`:

```
/ackermann_mux  /joy  /joy_toggle_teleop  /static_baselink_to_laser
/urg_node  /vesc_driver_node  /vesc_to_odom_node
        ↑ /ackermann_to_vesc_node 가 빠져 있음 = 조향 명령이 서보까지 못 감
```

### 정리 절차

launch로 뜬 노드는 전부 명령줄에 `--ros-args`가 붙음. 이걸로 한 번에 잡는다.

```bash
pgrep -af -- "--ros-args"        # 먼저 확인
pkill -f -- "--ros-args"
pkill -f "ros2 launch"
pkill -f "ros2 topic"

ros2 node list                   # 진짜로 비어야 함
```

> ⚠️ `perception_ws` 등 다른 ROS 노드가 떠 있으면 그것도 같이 죽음. 확인하고 쓸 것.

비어있지 않으면 남은 것을 이름으로 직접 죽일 것 (`pkill -f vesc_driver_node` 등).

### 재시작 후 반드시 검증

```bash
ls -l /dev/ttyACM0
ros2 launch f1tenth_stack bringup_launch.py     # 인자 없이!

ros2 node list                                       # 8개, ackermann_to_vesc_node 포함
ros2 topic info /commands/servo/position --verbose   # Publisher count: 1
ros2 topic info /teleop --verbose                    # Publisher count: 1
```

δ=0을 명령했을 때 `/commands/servo/position`이 **0.403 하나만** 찍히면 준비 완료.
값이 두 종류 이상 나오면 아직 중복이 남은 것.

---

## 1. 확정값

`hardware_ws/src/f1tenth_stack/config/vesc.yaml` — hardware_ws 전체의 single source of truth.

```yaml
steering_angle_to_servo_gain:   -0.8206
steering_angle_to_servo_offset:  0.403
servo_min: 0.05
servo_max: 0.70
```

```
servo = -0.8206 * delta(rad) + 0.403
```

- `delta` = `steering_angle`, **라디안**. 직진 = 0. **양수 = 좌회전(반시계).**
- `servo` = **0~1 정규화 서보 위치**. 각도가 아님. VESC가 PWM 펄스폭으로 변환
  (0.0→1.0ms, 0.5→1.5ms, 1.0→2.0ms).
- 직진 명령 시 `servo == offset`이 찍히는 것이 정상.

**사용 가능한 조향 범위** (`steer_calib.py limits` 출력):

```
포화 없는 조향각 범위 : -0.3619 ~ +0.4302 rad (-20.7° ~ +24.6°)
좌우 대칭 최대 조향각 : ±0.3619 rad (±20.7°)
```

우측이 좁아서 대칭 한계를 결정함. 최소 선회반경 약 0.87 m.

---

## 2. 신호 흐름

```
조이스틱 → /teleop ─┐
                     ├→ ackermann_mux → /ackermann_cmd
자율주행  → /drive  ─┘   (teleop 우선순위 100 > drive 10, timeout 0.2s)
                              │
                              ▼
                  ackermann_to_vesc_node   ← gain/offset 여기서 적용
                              │
                   /commands/servo/position     ★ clip 전
                              │
                              ▼
                        vesc_driver
                  servo_min/max 로 clip
                              │
                /sensors/servo_position_command  ★ clip 후
                              │
                              ▼
                          실제 서보
```

`throttle_interpolator`는 `bringup_launch.py:172`에서 주석 처리되어 **체인에 없음.**
따라서 명령과 서보 사이에 스무딩/레이트 리밋이 없고, 위 두 토픽은 clip 여부만 다름.

---

## 3. 서보의 물리적 한계 (스탠드 실측)

앞바퀴를 띄우고 조이스틱을 아주 천천히 밀며 스톨 부저음("삐")이 나기 시작하는
지점을 측정. 스톨음이 멎는 지점에서 값을 읽음.

| 조건 | 결과 |
|---|---|
| 무부하 · 우측 | **servo 0.7196부터 스톨.** 이후 계속 남 |
| 무부하 · 좌측 | servo 0.0424(조이스틱 최대)까지 **무소음** |
| 부하(바닥) · 전 구간 | 중립(0.455) 포함 **모든 위치에서 스톨음** |

**중요한 발견:**

- **포화는 우측이다.** 구버전 문서가 전제했던 "좌측이 servo 0.155에서 포화"는
  **반증됨.**
- 좌측 실제 한계는 **아직 모름.** `servo_min: 0.05`에 막혀서 그 아래를 못 봤을 뿐.
  0.05는 측정 근거가 없는 값임.
- **부하 시 전 구간 스톨음**은 정지 상태의 타이어 스크럽 토크가 서보 토크를 넘는
  것. 주행 중에는 스크럽 토크가 훨씬 작아 원 측정 자체는 유효했음.
  다만 **지면 정지 상태에서 잰 조향각은 신뢰 불가.**

---

## 4. 어떻게 확정했나 — 채택한 데이터

세 측정이 잔차 0.003 servo(약 0.2°) 이내로 **한 직선에 놓임.**

| # | 측정 | servo | 실제 조향각 |
|---|---|---|---|
| B | 좌 원, δ=+0.18, 지름 **3.12 m** | 0.2366 | +0.2084 rad (+11.9°) |
| C | 우 원, δ=−0.18, 지름 **1.96 m** | 0.6734 | −0.3248 rad (−18.6°) |
| E | 직진, offset 0.407, 4.5 m에 **15 cm 우측** | 0.4070 | −0.0049 rad (−0.28°) |

- **gain**은 B·C가 결정 (servo 간격이 넓어 조건수가 좋음)
- **offset**은 E가 결정 (δ=0을 직접 재므로 gain과 무관)

B·C는 같은 속도(0.8 m/s), 2바퀴 이상 돌린 뒤 뒷차축 중심 원의 지름을 측정.
두 원의 크기가 크게 다른 것(3.12 vs 1.96)이 곧 offset이 0.455에서 멀리
떨어져 있다는 증거였음.

### 검증 — 독립 재측정으로 확정 (2026-07-22)

B·C가 방전된 배터리로 측정됐고 원 중심이 드리프트해서 gain이 의심스러웠음.
배터리 교체 + 프로세스 중복 정리 후, **다른 조향각(δ=±0.30)으로 재측정:**

| | 예상 지름 | 실측 |
|---|---|---|
| 좌 (servo 0.1568) | 2.133 m | **2.13 m** (±2~3 cm) |
| 우 (servo 0.6492) | 2.133 m | **2.13 m** (±2~3 cm) |

- **gain 확정.** 역산하면 −0.8198, 현재 값 −0.8206과 0.1% 차이. 측정 정밀도 안
- **offset 확정.** 좌우 지름이 3 cm 이내로 일치 → 중립 오차 **±0.002 rad(0.11°) 이내**.
  offset이 틀렸다면 두 원의 크기가 눈에 띄게 달라졌을 것
- **선형성 확인.** δ=±0.18과 δ=±0.30, 두 서로 다른 각도에서 같은 모델이 성립
- 결과적으로 방전 배터리와 중심 드리프트는 B·C를 오염시키지 않았음

### 최종 검증 — 측정 한계 도달, 값 확정

미검증이던 두 영역(장거리 직진 / 조향 극단)을 서로 다른 코드 경로로 확인:

| 테스트 | 결과 | 판정 |
|---|---|---|
| 직진 9 m (`topic pub`, δ=0) | 우측 **35 cm** = 0.16° 편차 | 통과 |
| 풀스틱 원 (**조이스틱**, δ=±0.34) | 시계 **1.86 m** / 반시계 **1.84 m** (예상 1.87) | 통과 |

풀스틱에서 **스톨음 없음** → `servo_max: 0.70`이 서보를 제대로 보호함을 확인.
조이스틱 경로(`joy_toggle_teleop` → mux)도 이때 처음 검증됨.

**두 테스트는 offset을 서로 반대 방향으로 밀었음** (직진 → 0.4007, 원 → 0.4044,
차이 0.26°). 각 측정의 불확실도(±0.1~0.15°)와 같은 크기이므로 **노이즈 바닥에
도달한 것.** 지금까지의 모든 독립 추정을 모으면:

```
gain   : -0.8192 / -0.8198 / -0.8142   (산포 0.8%)   → 현재값 -0.8206
offset :  0.4007 /  0.4030 /  0.4044   (평균 0.4027) → 현재값  0.4030
```

**현재 값이 모든 추정의 중심에 있음. 더 이상 손대지 말 것** — 어느 한 측정을
골라 반영하면 없는 정밀도를 만들어내는 것이 됨.

> 남은 0.16° 편차는 실용상 무시 가능(제어기 피드백이 흡수). 다만 offset 변경에
> 대한 응답이 계속 예측보다 작게 나옴(이번에도 예측 0.28° 중 0.12°만 반영).
> 조향과 무관한 미세한 기계적 쏠림(토우 등)이 섞여 있다는 뜻이나, 이 규모에서는
> 손댈 가치가 없음.

### 폐기한 데이터

| 데이터 | 폐기 이유 |
|---|---|
| 옛 우회전 원 (servo 0.8876, 지름 1.51 m) | 스톨 지점 0.7196을 **0.168 초과**. 전 구간 스톨 상태로 측정됨 |
| 옛 좌회전 원 (servo 0.0624, 지름 2.25 m) | 다른 세션 · 좌우 속도 불일치 의심. 새 모델은 이 servo에서 23.9°를 예측하나 측정은 16.35° |
| 2.25 m 직진 3회 (1.09° / 1.12° / 1.49°) | **재현 안 됨.** offset을 0.02 바꿔도 예측된 1.09° 변화가 나타나지 않음. 노이즈가 신호와 같은 크기 |

구버전 문서의 gain 후보 −1.0498은 **위 무효 우회전 원으로 피팅한 값**이라 폐기.

### 원인 규명: 왜 서보가 스톨했나

구버전 문서 §2(b)가 `servo_min/max`를 `0.15/0.85 → 0.05/0.95`로 넓힌 것이
직접 원인. 근거는 "손으로 돌리니 24°까지 가더라"였는데, **손으로는 서보의
구동 범위를 넘어 강제로 돌릴 수 있어서** 판단 근거가 될 수 없었음.
원래의 0.15/0.85는 이 서보를 스톨에서 보호하는 값이었음.

---

## 5. 남은 작업

> **완료된 항목**
> - gain·offset 확정 및 독립 재측정 검증 (§4)
> - `control_pkg/config/params.yaml`의 `max_steering_angle_rad` 0.4189 → **0.36** 반영

### ① 좌측 스톨 지점 측정 (미완, 선택)

`servo_min: 0.05`는 근거 없는 값. 실제 좌측 한계를 알면 비대칭 정도를 확정할 수 있고,
드래그링크를 얼마나 조절해야 중립이 가동범위 중앙으로 오는지 계산 가능.

임시로 `servo_min: 0.0`으로 낮추고 스탠드에서:

```bash
python3 ~/steer_stand.py sweep 0.40 0.52 0.01 --dwell 1.5
```

소리 나면 즉시 Ctrl+C. **측정 후 `servo_min`을 반드시 되돌릴 것.**

### ② 다른 스택의 파라미터 사본 동기화 (미완)

`unicorn-racing-stack-jazzy/stack_master/config/CAR/vehicle_config.yaml`에
gain/offset/조향한계 사본이 있음. **그 스택을 쓸 거면 같이 수정할 것.**
(`control_pkg` 쪽은 이미 0.36으로 반영 완료.)

### ③ VESC 시리얼 에러 원인 규명

작업 말미에 `Invalid end-of-frame character` / `Out-of-sync with VESC` 발생.
**배터리 전압 저하가 1순위 용의자** (서보 반복 스톨로 BEC에서 1~2 A씩 소모).

```bash
ros2 topic echo /sensors/core --field state.voltage_input
```

2S 7 V / 3S 10.5 V 아래면 충전. 포트 중복 점유도 확인 (`lsof /dev/ttyACM0`).

### ④ 서보 상태 점검

부하 시 전 구간에서 스톨음이 나는 것이 정상 범위인지 확인 필요.
그동안 반복 스톨했으므로 **기어·포텐셔미터 마모로 헌팅**하는 상태일 가능성 있음.
무부하 조용 + 부하 시 전 구간 소음은 그 징후와도 일치함.

### ⑤ ★ 속도 캘리브레이션 (`speed_to_erpm_gain`) — 다음 주 작업

- **VESC Tool에서 걸어둔 max ERPM 제한을 반드시 해제할 것.** 응답이 포화되어
  gain이 엉뚱하게 나옴.
- **`/odom`으로 검증하면 안 됨.** `vesc_to_odom`이 같은 gain으로 나눠서 오차가
  상쇄됨. 줄자 + 스톱워치 같은 **외부 기준**으로 측정.
- `speed_to_erpm_offset`은 0으로 유지 (정지 명령이 정지여야 함).

---

## 6. 알아둘 함정 (모두 실제로 겪음)

- **`ros2 param set`이 안 먹는다.** `ackermann_to_vesc.cpp:51-56`이 생성자에서
  파라미터를 한 번 읽고 멤버에 저장. 변경 콜백 없음. → **launch 재시작.**

- **vesc.yaml과 launch 인자가 어긋날 수 있다.** 실제로 발생했음 — yaml에는
  0.5304가 있는데 launch 인자로 0.455를 계속 덮어쓰고 있었고, 그동안 계산 도구는
  yaml 값을 읽고 있어서 결과가 전부 틀렸음. **가급적 yaml만 고치고 launch 인자는 쓰지 말 것.**

- **★ launch를 두 번 띄우면 조용히 망가진다.** ROS 2는 같은 이름의 노드가 중복
  실행돼도 **에러를 내지 않음.** `ackermann_to_vesc_node`가 둘이 되면 각자 다른
  gain/offset으로 **같은 `/commands/servo/position`에 쓰고**, 서보가 두 위치를
  왕복하며 떨림. §0에 실제 사례와 정리 절차가 있음.
  → 새 launch 전에 항상 `ros2 node list`로 이전 것이 죽었는지 확인할 것.

- **`pkill -f bringup_launch`는 자식 노드를 안 죽인다.** 런처만 죽고 노드들은
  고아로 남아 계속 동작함. 터미널이 없는데 `ros2 node list`에 노드가 뜨면 이 상태.
  → `pkill -f -- "--ros-args"` 로 잡을 것 (§0).

- **짧은 직진 테스트는 쓸모없다.** 2.25 m에서는 출발 헤딩 1° 오차가 3.9 cm를
  만들어 신호와 노이즈가 같은 크기가 됨. **4 m 이상**에서 재야 의미가 있음
  (이탈은 거리²에 비례).

- **조이스틱으로는 캘리브레이션 못 한다.** 아날로그 스틱은 특정 δ에 고정할 수도,
  재현할 수도 없고, 30 Hz로 스크롤되는 토픽 값을 눈으로 따라가기도 어려움.
  → 캘리브레이션 측정은 반드시 `ros2 topic pub`으로 정확한 δ를 명령할 것.

- **클립되면 `fit` 결과가 통째로 틀린다.** `fit`은 명령 δ로부터 servo를 계산하는데,
  실제로는 클립된 값이 나가므로 둘이 어긋남. 구버전의 gain −1.0498이 이렇게 나온 값.
  측정 중 `/commands/servo/position`과 `/sensors/servo_position_command`가
  같은지 항상 확인할 것. (`steer_calib.py fit`도 클립 범위를 벗어나면 경고하고 중단함.)

  참고로 **현재 캘리브레이션에서는 조이스틱 최대 δ=0.34가 클립되지 않음**
  (좌 servo 0.124, 우 0.682 — 둘 다 [0.05, 0.70] 안, 우측 스톨 0.7196에서도 여유).
  단, gain/offset을 다시 바꾸면 이 여유가 사라질 수 있으니 재확인할 것.

- **`topic pub`과 조이스틱은 같은 `/teleop`을 쓴다.** MANUAL 모드에서는
  `joy_toggle_teleop`이 30 Hz로 0을 쏘고 있어서 20 Hz `topic pub`을 덮어씀
  → 차가 안 움직임. **반드시 버튼 5로 AUTONOMOUS 전환** 후 로그에
  `--> AUTONOMOUS (joystick ignored)` 확인.

- **IMU yaw odometry 적용 전에는 `/odom`으로 조향 오차를 못 쟀다.** 현재는
  `vesc.yaml`의 `use_imu_yaw: true`로 VESC AHRS 상대 yaw를 사용하므로 `/odom` yaw가
  서보 명령 모델과 독립적이다. 단, 속도·거리 스케일은 여전히 같은 eRPM gain을
  사용하므로 반드시 **줄자로 물리 측정**해 검증할 것. `use_imu_yaw: false`로 되돌리면
  예전처럼 서보 명령값을 같은 gain/offset으로 복원하므로 이 독립성이 사라진다.

- **`vesc_driver_node`는 포트가 없으면 조용히 죽는다.** launch 로그에
  `process has finished cleanly`가 뜨고 끝. 죽은 뒤 VESC를 연결해도 자동 복구
  안 됨. → **launch 전에 `ls -l /dev/ttyACM0` 확인.**
  VESC는 `lsusb`에 `0483:5740 STMicroelectronics Virtual COM Port`로 보임.
  USB만 꽂고 배터리가 없으면 서보 전원(5V BEC)이 안 들어와 서보가 안 움직임.

- **저속에서 차가 안 움직일 수 있다.** 0.4 m/s(=1845 erpm)는 브러시리스 모터가
  코깅 때문에 정지 마찰을 못 이기는 영역. 0.8 m/s를 쓰거나 손으로 살짝 밀어줄 것.

---

## 7. 빌드 — yaml만 고치면 재빌드 불필요

`--symlink-install`로 빌드되어 있고 링크 체인이 살아있음:

```
install/.../config/vesc.yaml → build/.../config/vesc.yaml → src/.../config/vesc.yaml
```

`.py` 파일을 새로 추가하거나 entry point를 바꿀 때만 재빌드:

```bash
cd ~/IFAC_ws/hardware_ws
colcon build --base-paths src --symlink-install --packages-select f1tenth_stack
```

> zsh에서 `source /opt/ros/humble/setup.bash`는 깨짐. `setup.zsh`를 쓸 것.

---

## 8. 도구

원본은 **`hardware_ws/tools/`** 에 있고 (저장소에 커밋됨),
홈 디렉터리에 심볼릭 링크가 걸려 있어 `~/steer_calib.py` 로도 그대로 실행된다.

| 경로 | 용도 |
|---|---|
| `~/steer_calib.py` | `limits` / `plan` / `fit` / `trim` / `arc` — 조향 계산기 |
| `~/steer_stand.py` | `sweep` / `hold` — 스탠드 테스트용 명령 발행 (speed 0 고정) |
| `~/servo_sweep.py` | 스틱 스윕하며 clip 전/후 servo 최소·최대 기록 |

**`steer_calib.py`는 이제 vesc.yaml을 직접 읽음.** 상수를 손으로 맞출 필요 없음
(예전에는 상단 상수가 낡아서 계속 틀린 결과가 나왔음).

```bash
python3 ~/steer_calib.py limits                    # 포화 없는 조향각 범위
python3 ~/steer_calib.py plan                      # 각도별 필요 원 지름
python3 ~/steer_calib.py fit 0.18 3.12 -0.18 1.96  # 좌우 원으로 gain+offset 피팅
python3 ~/steer_calib.py trim 4.5 -0.15            # 직진 이탈로 offset 보정
python3 ~/steer_calib.py arc 1.60 1.60 2.30        # 원 못 그릴 때: 경로 3점으로 반경 역산
```

`steer_stand.py`는 **speed가 0.0으로 고정**되어 스탠드에서 안전.
실행 전 **버튼 5로 AUTONOMOUS 전환** 필수.

```bash
python3 ~/steer_stand.py sweep 0.20 0.36 0.02 --dwell 1.5   # 스톨 지점 탐색
python3 ~/steer_stand.py hold 0.0                           # 한 값 고정 (정렬 작업용)
```

---

## 9. 표준 터미널 세팅

```bash
# 모든 터미널 공통
source /opt/ros/humble/setup.zsh
source ~/IFAC_ws/hardware_ws/install/setup.zsh
```

```bash
# T1 — bringup (포트 먼저 확인!)
ls -l /dev/ttyACM0
ros2 launch f1tenth_stack bringup_launch.py
#   launch 인자 없이. yaml이 정답을 갖고 있음.
#   조이스틱 버튼 5 → 로그에 "--> AUTONOMOUS" 확인

# T2 — 명령
ros2 topic pub -r 20 /teleop ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.8, steering_angle: 0.0}}"

# T3 — clip 전 servo
ros2 topic echo /commands/servo/position --field data

# T4 — clip 후 servo (T3와 값이 다르면 clip 걸린 것 = 측정 무효)
ros2 topic echo /sensors/servo_position_command --field data
```

건강 확인 — **측정 전에 매번 할 것:**
```bash
ros2 node list                                       # 8개, ackermann_to_vesc_node 포함
ros2 topic info /commands/servo/position --verbose   # Publisher count: 1  ← 중복 검출
ros2 topic info /teleop --verbose                    # Publisher count: 1
ros2 topic hz /sensors/core                          # VESC 시리얼 통신 확인
ros2 topic echo /sensors/core --field state.voltage_input   # 배터리
```

`Publisher count`가 2 이상이면 **그 상태로 측정한 데이터는 전부 무효** (§0).
`/commands/servo/position`에 값이 두 종류 이상 번갈아 나오는 것도 같은 신호.

---

## 10. 커밋 안 된 변경사항

```
 M hardware_ws/src/f1tenth_stack/config/joy_teleop.yaml
 M hardware_ws/src/f1tenth_stack/config/vesc.yaml
 M hardware_ws/src/f1tenth_stack/launch/bringup_launch.py
 M hardware_ws/src/f1tenth_stack/setup.py
?? hardware_ws/src/f1tenth_stack/f1tenth_stack/joy_toggle_teleop.py
```

### `joy_toggle_teleop.py` — 토글식 teleop (신규)

upstream `joy_teleop`은 두 문제가 있어 교체:

1. `joy_teleop.yaml`의 `default:` 블록에 `deadman_buttons`가 없어 **노드가 기동
   즉시 크래시**했음 (`JoyTeleopException: No buttons or axes configured for
   command 'default'`). 조이스틱이 아예 안 먹던 원인.
2. deadman **홀드** 방식만 지원. 토글 불가.

동작:
- **버튼 5 = 수동/자율 토글** (엣지 트리거, 홀드해도 1회만 전환)
- 수동: 스틱만으로 조작
- 자율: 0.5초간 0을 발행해 차를 세운 뒤 침묵 → mux가 `/drive`로 넘어감
- 강제정지 버튼 없음 (요청대로)
- 기능 검증 완료

### `vesc.yaml`

- `servo_max: 0.95 → 0.70` (우측 스톨 0.7196 아래로)
- `gain: -1.2135 → -0.8206`, `offset: 0.5304 → 0.403` (본 문서 §4)

### `bringup_launch.py`, `setup.py`

`joy_teleop` 대신 `f1tenth_stack/joy_toggle_teleop` 실행하도록 배선 + entry point 등록.
calibration 4종을 launch 인자로도 노출 (기본값은 vesc.yaml에서 읽어옴).

### `joy_teleop.yaml`

`max_steering_angle: 0.34` — 그대로 두어도 됨. 새 캘리브레이션에서 스틱을 끝까지
밀어도 servo가 0.124~0.682라 클립·스톨 모두 걸리지 않음. 대칭 한계 0.3619보다도
안쪽임.
