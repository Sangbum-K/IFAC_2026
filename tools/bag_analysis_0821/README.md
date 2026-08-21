# run_0820_235246 분석 (11분 저속 RC 상대 동적장애물 연습런)

```bash
python3 extract.py /home/sangbum/IFAC_ws/run_0820_235246/run_0820_235246_0.db3 d.pkl
python3 relog.py       # /rosout 을 메시지 자체 stamp 로 재타임스탬프 (필수)
python3 ov.py          # 랩타임 + reject/outcome 최종 카운터
python3 tr.py          # [trace] 라인 -> trace.pkl (모드/static/dist/사유)
python3 det.py         # 트랙별 수명·최초탐지거리·분류 flip
python3 ep.py          # trace 연속구간 = 조우 에피소드 요약 (113개)
python3 tl.py t0 t1 [step]   # 프레임 타임라인
python3 log.py t0 t1 [pat]   # 재타임스탬프 rosout
python3 mech.py        # committed_break 사유, id 전환, 분류 지연, 예측 커버리지
python3 gap.py         # 동적 트레일링 간격/속도, otwpnts 진폭
python3 clr.py         # 요구 장애물간격(3σ) vs 트랙 폭
python3 vd.py          # 브릿지 vs/vd 품질(0.3s 차분 대조) + 상대 횡이동량
python3 pass.py        # |Δs|<0.45 순간의 실제 차체 횡여유 = 접촉 판정
python3 brake.py       # 에피소드별 탐지거리 / 제동시작 / 통과 성공
python3 why.py         # 탐지 후 1초 이내 기각 사유 분포
python3 side.py        # 회피 좌우 뒤집힘, 계획 apex 변화율
python3 line.py        # 상대 주행선 재현성(s별 d 산포) + 속도 손실
python3 chk.py         # 분류 출생 패턴 + 트레일링 제동 포락선
python3 froz.py        # otwpnts 비트동결 구간
python3 hit.py         # 급감속/yaw 반전 후보
```

시간축은 bag 첫 메시지 기준 상대초 (사용자 영상 시각과 일치).

## 핵심 결과
- 랩 63회. 자유주행 랩 7.45~7.7 s, 상대 조우 랩 최대 18.55 s.
- **동적 상대가 5 m 안에 있던 시간 166 s (전체의 25%), 그 구간 속도는 레이스라인의 46%**
  (자유주행 85%).
- **나란히 선(|Δs|<0.45 m) 순간의 실제 차체 횡여유: 동적 21회 중앙 0.11 m,
  9회(43%)가 0 이하 = 접촉.** 정적은 64회 중앙 0.31 m, 5회(8%)만 0 이하.
- 분류 로그 141 트랙 중 **139개(99%)가 STATIC 으로 태어난다**. 그중 37개가
  나중에 DYNAMIC 이 된다 — 지연 p50 0.23 s / p90 0.57 s / max 1.13 s.
- **최초 탐지거리 중앙 4.1 m** (코너 s20~24 는 2.85 m, 직선 s28~36 은 7.9 m).
  range_max 12 m 이므로 센서 한계가 아니라 시선 차단이다.
- 트레일링 제동 포락선(decel_dyn 1.5, lag 0.3, gap 1.5)이 요구하는 여유는
  v=4 에서 6.7~7.7 m, v=5 에서 10.0~11.0 m. **탐지 4.1 m 는 항상 그 안쪽이라
  첫 관측 순간 이미 "지금 최대로 밟아야" 하는 상태다.**
- 탐지 후 1초 이내 기각 1위는 `lane_wall_no_room` 30%(전체 34%),
  `lane_geom_floor` 는 7%(전체 16%). 즉 1차 원인은 **시간이 아니라 폭**이다.
  - "벽 때문에 오프셋 축소" 39건: 목표 apex 0.56 -> 클램프 0.22, 필요 0.27 (5 cm 부족).
  - "어떤 속도로도 불가" 61건: 횡 0.50 m 에 기하하한 1.94 m 필요, 가용 1.22 m.
- 상대 횡이동 |Δd(1 s)| p50 0.24 / p90 0.79 m — **37% 의 경우 1초 안에
  evasion_dist(0.35) 보다 크게 움직인다.** `fixed_pred_time` 은 0.15 s(=0.04 m).
- 브릿지 속도 추정은 문제없다: vd 보고/실측비 0.92·부호일치 92%·상관 0.76,
  vs 비 0.99·상관 0.87. **속도가 아니라 지평(horizon)이 없는 것이 문제.**
- opponent_prediction 은 6149 발행 중 787(12.8%)만 비어있지 않고,
  동적 프레임 기준 커버리지 41%.
- 계획 apex 변화율 p90: 동적 4.07 m/s vs 정적 0.74 m/s. 회피 좌우 뒤집힘
  동적 에피소드당 2.2회.
- 접촉 2건의 기전이 동일하다 — **커밋된 회피선 위로 상대가 옆에서 들어온다**:
  - t=228.6~231.0 (id 932): ot `d -0.22..0.25` 로 2.4 s 유지되는 동안 상대가
    d +0.16 -> -0.46 이동, 자차 d -0.38 이 상대 박스 [-0.49,-0.32] 안.
  - t=647.1~648.6 (id 1698): 정적 콘 #1708 기준으로 좌측 d=+0.66 커밋,
    그 좌측을 상대가 d[0.68,1.03] 로 점유. 커밋 해제는 dist 0.87 m 에서야 났다.

## P0 계측 (2026-08-21) — 거동 변경 없음

`spliner_node` 에 카운터 3종을 붙였다 (PLAN.md §1). 전부 `NoteOutcome` 이라
`/pitwall/outcome_*` 와 `[trace]` 문자열에만 나간다.

| 카운터 | 위치 | 목적 |
|---|---|---|
| `blocked_both_sides` | `MoreSpace` 의 `!left_clears && !right_clears` | "트랙이 실제로 막혔다"의 유일한 사전 신호. 지금은 아무도 소비하지 않는다 |
| `escape_attempted` / `escape_gate_not_geom` | `BuildEscapeArc` 진입 / Tier2c 게이트 | escape 활성 0% 의 분모 |
| `trailing_retry_clamp_raised` / `_stop_denied_ever_dynamic` / `trailing_no_return_*` | `BuildTrailing` 의 `!allow_stop` 두 곳 | 정지 권한이 박탈된 사이클, 그중 "달리다 멈춘 상대차" |

```bash
./replay_spliner.zsh <bag_dir> <out_prefix>   # spliner 단독 개루프 재생 (1배속 고정)
python3 p0.py <out_prefix>                    # 카운터 + 지속길이 + 원본 대조
```

⚠️ `replay_spliner.zsh` 주의
- **배속 인자를 두지 않았다.** 루프가 `create_wall_timer`(벽시계)인데 staleness 는
  `use_sim_time`(재생시계)이라, 배속을 주면 sim 초당 사이클 수가 그만큼 줄어
  카운터가 통째로 어긋난다.
- 중복 노드 가드가 `pgrep -x`(프로세스명 정확매칭)다. `pgrep -f` 는 **호출한
  쉘의 명령줄까지 훑어 자기 자신을 잡는다** — 실제로 잡아서 한 번 안 돌았다.
- 그 전 판은 `set -e` 때문에 `kill` 의 비영 반환에서 스크립트가 끊겨 노드가
  살아남았고, 다음 실행과 겹쳐 spliner 가 2개 떴다 (CLAUDE.md §0 의 그 사고).
  지금은 `trap` + `cleanup()` 으로 처리한다.

## 이 백의 맵/레이스라인은 저장소에 없다

맵 36개 중 origin/크기가 맞는 것 0 (백은 375x150, origin −12.217/−1.627),
iqp CSV 24개 중 x/y 범위가 맞는 것 0. `map-run0815-not-in-repo` 와 같은 사고다.
둘 다 백에서 복원했다 — 시뮬에서 같은 트랙을 쓰려면 이 두 파일이 필요하다.

```bash
python3 dump_map.py      d.pkl my_track0820-235246   # -> <ws>/maps/*.pgm + *.yaml
python3 dump_raceline.py d.pkl my_track0820-235246   # -> config/global_trajectory/*_iqp_*.csv
python3 s1_geom.py                                   # S1 콘 배치 계산
```

복원한 레이스라인은 검증을 통과한다: `|kappa|` max 0.650 < 조향한계 1.133,
a_lat max 9.94 < `validation_mu`(1.10)x g = 10.79.

## S1 (봉쇄 정지) 배치

s=31.03 이 직선(|kappa| 0.01)이면서 폭 1.51 m 로 가장 좁다. 상대가 레이스라인
(d=0)에 서면 **우측 여유가 0.27 m 로 이미 통과 불가**(필요 0.55)이므로,
콘 하나로 좌측만 막으면 양쪽 봉쇄가 완성된다.

```bash
python3 scenario_s1.py --s 31.03 --hold 30
```

콘 자동 배치: `d = d_left − 0.55 − 0.17 + 0.10` (좌측 잔여 0.45 m < 필요 0.55).

## 시뮬 시나리오 (2026-08-21)

```bash
# 공통 — 반드시 저장소의 ./run_sim.sh(= simm 이 쓰는 경로)를 통해야 한다
DUR=<초> MAP=my_track0820-4 [POST=<명령>] ./run_sim.zsh <out_prefix>
```

⚠️ **`sim_race.launch.py` 를 직접 띄우지 말 것.** 한 번 그렇게 돌려서 측정을 통째로
날렸다. 그쪽은 detector 도 control 도 안 띄우고(launch 주석: "integration_pkg
stack.launch.py 가 별도로 띄운다"), 파이썬 트래커의 `tracker/speed_scale 0.30`
때문에 랩이 23 s(실차 7.5 s)였으며 `/perception/obstacles` 가 전 프레임 비어 있었다.
`./run_sim.sh` 는 `stack.launch.py` 를 띄우고 워크스페이스 5개 source, 고아 노드
정리, gym 경로 해석(저장소 사본 우선)까지 대신한다.

정식 경로로 돌린 기준선 (my_track0820-4, 콘 3개, 상대 없음):
랩 최속 7.20 s / 자유주행 속도비 93% / 정적 통과 63회 접촉 0%.

| 스크립트 | 용도 |
|---|---|
| `run_sim.zsh` | `./run_sim.sh` 얇은 껍데기 + 기록 + POST 훅 |
| `place_cones.sh` | S4 콘 3개 배치 (`s4_cones.py` 산출 좌표) |
| `scenario_s1.py` | S1 봉쇄 정지 자동 진행 |
| `s1_verdict.py` | S1 판정 — **봉쇄 물체 전부**에 대한 차체 여유 |
| `metrics.py` | 회귀 감시 지표 (실차/시뮬 공통) |
| `ab3.py` | 3-way 판정 (잡음 바닥 대비 신호) |

### S1 시나리오를 만들 때 걸린 함정 4개

1. **상대가 정지 명령 후 2.2 m 타행한다.** 콘을 미리 놓으면 상대가 콘을 지나쳐 서서
   봉쇄가 성립하지 않는다(자차가 4.85 m/s 로 옆을 통과했다). 순서를 "정지 → 타행
   종료 대기 → **실제 정지 위치 확인** → 그 옆에 콘" 으로 바꿔야 한다.
2. **명령 토픽은 `/drive`** 다. `/pitwall/cmd_speed` 는 파이썬 트래커 전용이라
   정식 스택에서는 오지 않는다.
3. `dec_odom` 은 `(ht, x, y, yaw, v)` 를 준다. frenet odom 에서 x=s, y=d 이고
   **속도는 5번째**다. yaw 를 속도로 읽으면 "자차 속도 0.00" 이 계속 나온다.
4. **여유를 상대에게만 재면 안 된다.** 봉쇄는 상대+콘으로 만들어지므로 상대를 피해
   콘을 밟고 지나가도 "여유 +0.37" 로 보인다 — 실제로 1회차가 그랬다.
   `s1_verdict.py` 는 나란히 선 프레임의 **모든** 장애물에 대해 최소값을 쓴다.

### S4 (정적 회귀) — 통과

`ever_moved_min_disp_m` 를 0(기능 OFF) / 0.5(ON) 로만 바꾼 파라미터 A/B.
상대가 없으므로 `ever_moved` 트랙이 생길 수 없고, 실제로 두 런 다 **0개**였다
(콘 max_disp 최대 0.199 / 0.422 m). 정적 접촉률 **0% / 0%**.
→ 기능이 발동할 수 없는 조건에서의 대조이므로 "정적 무영향"이 런 단위로 확인됐다.
남은 차이(랩 p50 7.56→8.27, ok_lane_switch 103→229)는 전부 시뮬 런-간 분산이다.

⚠️ 시뮬 콘이 실차보다 거칠다: max_disp 최대 0.422 m (실차 0.244 m). 문턱 0.5 는
그 위에 있지만 여유가 실차만큼 크지 않다.
