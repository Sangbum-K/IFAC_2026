# F1TENTH 조향 캘리브레이션 — 인수인계

최종 갱신: 2026-08-10
작업 위치: `~/IFAC_ws/hardware_ws` (ROS 2 Humble, 쉘은 zsh)

**현재 확정값은 `gain -0.8320` / `offset 0.3941`** (§1, 이력 §11).
2026-08-10 타이어 교체 후 재적합했고 같은 날 저녁 실차 주행으로 확인했다.
**이 문서 곳곳에 남아 있는 `-0.8206` / `0.403`은 07-22 옛 값이다** — 유도 과정
설명에만 쓰고, 값 자체는 §1과 vesc.yaml을 볼 것.

> ⚠️ **미해결: 조향 중립이 런마다 최대 2.5° 튄다** (§11).
> 지금 값으로 잘 달린다고 이 문제가 사라진 게 아니다. 한 런 안에서는 안정,
> 런 사이에는 점프 — 소프트 보정으로 덮을 수 있는 종류가 아니라서
> **기계적 점검이 먼저다.** "고쳤다"고 말하려면 §11 ★4의 연속 3런 기준을
> 통과해야 한다.

> ⚠️ **속도 마진 소멸** (§11 ①). 유효 구름둘레 +3.9%로 `speed_to_erpm_gain`이
> 이제 정확해졌다 = **08-09까지의 모든 실차 검증이 서 있던 4% 여유가 없다.**
> 플래닝은 `global_trajectory/velocity_scale 0.96`으로 임시 상쇄 중이고,
> 근본 해결은 08-10 이후 백으로 A_ACC/A_DEC를 **odom이 아닌 PF 기준**으로
> 재측정 → `deployment_profiles.py` 갱신 → 궤적 재생성이다.

다음 작업은 위 A_ACC/A_DEC 재측정, 그다음이 속도 캘리브레이션(§5 ⑤).

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
steering_angle_to_servo_gain:   -0.8320     # 2026-08-10 타이어 교체 후 재적합
steering_angle_to_servo_offset:  0.3941     # 2026-08-10 (0.4319 시도 → 기각, §11)
servo_min: 0.05
servo_max: 0.70
```

```
servo = -0.8320 * delta(rad) + 0.3941
```

> 07-22 ~ 08-10 오전에는 `-0.8206` / `0.403`이었다. 아래 §2~§10의 서술과 숫자
> 예시 상당수가 그 시절 것이니, **값이 필요하면 위 블록이나 vesc.yaml을 보고
> 본문 예시를 베끼지 말 것.** gain이 1.4% 바뀌었으므로 옛 백에서 뽑은 조향
> 수치는 전부 명령 대비 98.6% 상태의 값이다.

- `delta` = `steering_angle`, **라디안**. 직진 = 0. **양수 = 좌회전(반시계).**
- `servo` = **0~1 정규화 서보 위치**. 각도가 아님. VESC가 PWM 펄스폭으로 변환
  (0.0→1.0ms, 0.5→1.5ms, 1.0→2.0ms).
- 직진 명령 시 `servo == offset`이 찍히는 것이 정상.

**사용 가능한 조향 범위** (`steer_calib.py limits` 출력):

```
포화 없는 조향각 범위 : -0.3677 ~ +0.4136 rad (-21.07° ~ +23.70°)
좌우 대칭 최대 조향각 : ±0.3677 rad (±21.07°)
```

우측이 좁아서 대칭 한계를 결정함. 최소 선회반경 약 0.857 m, kappa_max 1.167 1/m.
(07-22 값 기준으로는 ±0.3619 rad / ±20.7° / 0.87 m였다.)

**플래닝·컨트롤은 이 값을 0.36으로 내림해서 쓴다** — 0.3677 대비 2.1% 안쪽.
네 곳이 반드시 같아야 하고 어긋나면 `deployment_profiles.resolve()`가 죽는다:
`spliner/vehicle_max_steering_rad`, `tracker/max_steer_rad`,
`deployment_profiles.MAX_STEER`, control `vehicle/max_steering_angle_rad`.

> ⚠️ 여유가 0.5% → 2.1%로 늘었다고 0.3677로 올리지 말 것. 이건 중립이
> 0.3941일 때의 값이고, §11처럼 실제 중립이 0.43 근처로 밀리면 우측 한계가
> **0.321 rad(18.4°, 선회반경 0.97 m)** 까지 줄어 0.36조차 낙관값이 된다.
> 기계적 재중심이 조향 범위를 되찾는 일이기도 하다는 뜻.

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

> ⚠️ offset 0.4319 기준으로는 스틱 최대(δ=±0.34)에서 servo가 0.153~0.711이 되어
> **우측이 servo_max 0.70을 살짝 넘긴다**(0.711 → 0.70으로 clip). 수동 조작
> 최대 우조향이 δ=-0.327에서 잘린다는 뜻. 자율주행에는 영향 없음
> (control의 `max_steering_angle_rad`는 0.4189이고 실제 명령은 0.28 이하).
> 거슬리면 `joy_teleop.yaml`의 `max_steering_angle`을 0.32로 내릴 것.

---

## 11. 2026-08-10 타이어 교체 — offset 재측정

### 무슨 일이 있었나

타이어를 갈고(지름 약 +2 mm) 나서 레이스라인 이탈이 눈에 띄게 커졌다.
코드는 08-09에 잘 달리던 그대로였다 (`git diff`에서 코드/설정 변경 0건).

| | 랩별 Mean CTE | 코너 바깥 편향 |
|---|---|---|
| 08-09 (`rosbag/run_0809_*`) | 0.122~0.162 m | +0.07 ~ +0.13 m |
| 08-10 (`rosbag0810/run_0810_*`) | 0.181~0.208 m | **+0.19 ~ +0.21 m** |

### 배제한 원인

- **접지력 저하 아님** — 달성 횡가속당 필요 조향각이 모든 구간에서 동일(1.35~1.52).
- **더 몰아붙인 것 아님** — 코너 요구 횡가속이 오히려 낮음(4.85 vs 4.98).
- **맵/측위 아님** — 스캔-맵 잔차 0.110 vs 0.114 m. 맵 해상도 셋 다 0.05 m/px.
  레이스라인 3일치를 겹쳐보면 같은 레이아웃.
- **회피 개입 아님** — `/pitwall/spliced_wpnts` 0%, e-stop 0%.

### 확인된 물리 변화 2가지

**① 유효 구름둘레 +3.9%.** PF 실측 경로길이 ÷ odom 적분:

```
08-09:  0.957 / 0.960        08-10:  0.994 / 0.996
```

`speed_to_erpm_gain: 4255.1`이 이제 거의 정확해졌다. 뒤집으면 **전에는 명령보다
4% 느리게 달렸고 그 마진이 사라진 것.** A_ACC/A_DEC, 6 m/s 승격, LUT 검증이 전부
그 4% 마진 위에서 이뤄졌음을 기억할 것. 지름 2 mm면 ~100 mm 휠에서 2%가 나와야
하는데 실측은 두 배 — 공기압/변형에 의한 유효 반경 차이나 슬립 감소가 섞였을 수 있다.

**② 조향 중립이 런마다 튄다 — 이게 진짜 문제다.**

각 런에서 요레이트가 0을 지나는 조향 명령 δ₀(Theil-Sen, wz/v 대 δ, |δ|<0.12,
v>2.5)를 구하고, 그 런에 **실제로 돌고 있던** offset을 백에서 역산해서
(`servo = gain*δ + offset` 회귀, 잔차 rms 0.0005) 하드웨어가 실제로 직진하는
서보 위치 **S\* = gain·δ₀ + offset_실행** 을 계산했다:

| 런 | 실행 offset | δ₀ | **S\*** | \|e\| p50 | 코너 바깥 | 랩타임 |
|---|---|---|---|---|---|---|
| `run_0809_224544` (36랩) | 0.403 | -0.0038 | **0.4061** | 0.141 | +0.050 | — |
| `run_0810_181449` | 0.403 | -0.0366 | **0.4330** | 0.173 | +0.215 | 9.4 s |
| `run_0810_182709` | 0.403 | -0.0340 | **0.4309** | 0.199 | +0.225 | — |
| `run_0810_192916` (t>75s) | **0.4319** | +0.0426 | **0.3969** | 0.253 | +0.275 | 11.0 s |

**한 런 안에서는 S\*가 매우 안정적인데(15초 슬라이딩 윈도 표준편차 0.0014~0.0047),
런 사이에서는 최대 0.036 servo = 2.5°씩 튄다.** 이 대비가 핵심이다.

- 추종이 나쁜 구간이 추정을 오염시킨 게 아니다 — `|lat_err|<0.15` 구간만
  써도 S\*가 0.4094 / 0.4340 / 0.4017로 전체값과 거의 같다.
- 부트스트랩 불확실도는 ±0.0009 이하. 즉 **런 사이 변동이 측정 정밀도의 40배.**

### 그래서: 소프트 보정은 안 된다

18:14/18:27 백에 맞춰 offset을 0.4319로 올렸더니, 19:29 런에서는 하드웨어가
이미 그 지점을 지나가 있었다. 결과는 **개선이 아니라 악화**:

```
|e| p50   0.173 -> 0.253      코너 바깥  +0.215 -> +0.275
랩타임     9.4 s -> 11.0 s     Mean CTE   0.20 -> 0.25~0.38
```

0.4319는 기각했다. 08-09의 실측 S\*(0.4061)가 원 캘리브레이션 0.403과 0.4° 이내로
일치하므로, "하드웨어가 정상일 때의 정답"은 0.43 쪽이 아니라 0.40 쪽이다.

**최종 채택: `gain -0.8320` / `offset 0.3941`** (08-10 저녁 재적합).
gain까지 함께 손댄 이유는 offset만으로는 설명 안 되는 잔차가 남았기 때문이고,
결과적으로 옛 gain은 명령 대비 1.4% 언더스티어였다(-0.8206/-0.8320 = 0.986).
**이 조합으로 실차 주행이 뚜렷하게 좋아졌다** — 다만 정량 재측정은 아직이다.

> ⚠️ 이게 "고쳤다"는 뜻은 **아니다.** 아래 진단(런 사이 점프)은 그대로 살아
> 있고, 지금 값은 "현재 하드웨어 상태에서 잘 맞는 값"일 뿐이다. 다음 충격
> 한 번에 다시 밀릴 수 있다. 판정 기준은 ★ 할 일 4번이다.

### 진단: 이산적 기계 미끄러짐

**런 안에서는 안정 / 런 사이에는 점프** 는 서서히 풀리는 헐거움이 아니라
**한 번에 툭 어긋나는** 고장의 신호다. 후보:

- 서보 세이버(servo saver)가 충격에 돌아감
- 서보 혼 스플라인 한 칸 밀림 / 혼 고정나사 풀림
- 타이로드·너클이 벽 충돌 때 재안착
- 휠 너트 토크 부족

08-10 런들은 코너에서 크게 벌어져 벽에 붙는 주행이었으므로 충격 기회가 충분했다.
백래시 추정(조향을 좌로 감을 때 vs 우로 감을 때의 S\* 차이)도 1.2~2.9°로 나오는데,
이건 동역학 지연이 섞여 있어 그대로 믿을 값은 아니지만 **손으로 유격을 확인해 볼
근거는 된다.**

### ★ 할 일 — 순서대로

1. **차를 들고 손으로 확인.** 앞바퀴를 좌우로 흔들어 유격, 서보 혼 고정나사,
   타이로드 엔드, 서보 세이버, 휠 너트 토크. 이게 1순위다.
2. 기계적으로 **재중심**을 잡고 offset은 현행 0.3941 유지.
3. §4의 E 방식 **4.5 m 직진 주행**으로 확인. 좌우 편차 5 cm 이내면 정상
   (4.5 m에 15 cm ≈ 0.005 rad ≈ servo 0.004).
4. **연속 3런**을 돌리고 매번 위 표의 S\*를 다시 계산할 것. 세 번 모두
   0.3941±0.005 안에 들어오면 그때 비로소 "고쳤다"고 말할 수 있다.
   여전히 튀면 부품(서보 세이버/혼) 교체.
   (08-10 저녁 주행은 정성적으로 좋았을 뿐 이 계산을 아직 안 돌렸다.)

> ⚠️ 교훈: 백에서 뽑은 조향 중립은 **한 런 안에서만 정밀하다.** 그 값을
> yaml 상수로 굳히기 전에 반드시 여러 런에서 재현되는지부터 볼 것.

### 속도 마진은 별개 문제로 남아 있음

①의 4%는 offset 수정으로 해결되지 않는다. 08-09와 같은 *실제* 속도로 돌려보려면
`planning_ws/src/planning_pkg/config/environments/real_car.yaml`의
`global_trajectory/velocity_scale`을 1.0 → **0.96**으로 두고 한 런 돌릴 것.
오차가 0.14 m대로 돌아오면 속도 탓, 그대로면 조향 탓 — 두 원인을 분리하는 A/B가 된다.
