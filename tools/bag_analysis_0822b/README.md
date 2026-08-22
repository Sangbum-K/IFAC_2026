# run_0822_172617 정적 회피 병목 분석

```bash
BAG=/home/miru/IFAC_ws/rosbag0822/run_0822_172617/run_0822_172617_0.db3
python3 extract.py $BAG d17.pkl && python3 addbool.py d17.pkl $BAG
python3 relog.py $BAG log17.pkl
python3 laps.py   d17.pkl              # 랩 분할 + 랩별 장애물 노출
python3 tr17.py   log17.pkl            # [trace] 모드/기각/거리별 성공률
python3 cost.py   d17.pkl              # gb -> 회피경로 -> 명령 -> 실제 속도 손실
python3 prof.py                        # 모드별 경로 속도 프로파일 형태  <-- 핵심
python3 head.py                        # 경로 머리 속도 vs cur_vs_       <-- 핵심
python3 attrib.py                      # 엔벨로프 seed 반사실
python3 caps.py                        # 그립/조향률/gb 캡 재구성
python3 corr.py   d17.pkl              # 회랑 기하 성립성 + 반사실
python3 rej.py                         # 모드별 기각 사유, 조우->커밋 지연
python3 epi.py                         # 조우별 손실 + 최악 조우 타임라인
python3 split3.py                      # 손실의 모드별 배분
python3 gates.py                       # 신규 게이트 3종 발화
python3 hold.py / holdwhy.py / holdwhy2.py   # 회피상실 홀드
python3 evermoved.py                   # ever_moved 트랙 <-> 저속 트레일링  <-- 핵심
```

시간축은 bag 첫 메시지 기준 상대초. 주행 구간 4~170 s (이후 정차).

## 요약

18랩. 장애물 적은 랩 7.37/7.69/8.08 s, 장애물 랩 평균 9.50 s -> **랩당 약 1.8 s (23 %) 손실**.
회피 경로가 발행된 시간이 주행의 52.5 %.

### 병목이 아닌 것 (측정으로 배제)

| 항목 | 측정 |
|---|---|
| 인지 거리 / 결정 지연 | 최초 감지 6.99 m -> 커밋 6.24 m, 지연 0.23 s. 조우 30회 중 26회(87%) 커밋 도달 |
| 회랑 폭 | 좌/우 모두 불가한 장애물-사이클 **3.8 %**. 넓은 쪽 회랑 중앙 1.20 m |
| 벽 유령 | planner 장애물 중 벽거리 <0.30 m = 1.2 % (detector 8.6 %) |
| commit_break_lat_dev (신규) | 최대 0.292 < 문턱 0.40 -> **한 번도 발화 안 함** |
| corridor_block_stop (신규) | 주행 중 래치 0회 (167.8 s 정차 후 1회) |
| 조향률 캡 | 웨이포인트의 4.6 %만 6 m/s 아래 |

### 병목 1 — committed_hold 의 속도가 자차 **s-속도**에 앵커된다 (67 % 사이클)

`ApplySpeedProfile` 의 전방 가속 엔벨로프 seed 가 `cur_vs_` (frenet ṡ) 다.

- 회피 중 ṡ/|v| = **0.917** (p10 0.756), 회피 없을 때 0.991. |d| 중앙 0.43 m.
- 경로 머리 vx 가 cur_vs_ 의 ±5 % 안: **65.5 %** 사이클.
- committed_hold 경로: 머리 3.23 (gb 의 0.59), 1 m 앞 3.59 (0.63), 5 m 앞에서야 0.87.
  컨트롤러는 speed_lookahead 0.25 s x v ~ 1 m 지점을 읽는다.
- 즉 **횡으로 크게 피할수록 플래너가 자차 속도를 더 낮게 보고 더 세게 제동을 지시한다.**
- 반사실(seed 를 실제 속도로): 1 m 지점 계획속도 3.59 -> 4.18 (+16 %).

### 병목 2 — ever_moved 오발동 (`spliner/ever_moved_min_disp_m: 0.5`)

정적 콘의 centroid 가 **자차가 지나친 뒤** 최대 1.07 m 흐른다 (id 92/107/130/134/143/151, 67개 중 6개).
장애물이 앞에 있는 동안은 0.00~0.43 m 로 안정적이고, 옆/뒤로 가는 순간 옆면만 보여 미끄러진다.

| closest 트랙 | trailing 명령속도 p50 | <1.5 m/s | <0.5 m/s |
|---|---|---|---|
| ever_moved 발동 (6개) | **0.74** | 75 % | 34 % |
| 일반 정적 | 2.85 | 13 % | 6 % |

`allow_stop = !is_static \|\| WasEverMoved(id)` 가 열려 `trailing_retry_speed`(2.5) 바닥이 사라지고,
`span=0`(이미 gap 안쪽) + 정적 목표속도 0 이 결합해 **이미 지나친 콘 옆에서 정지까지 감속**한다.
최악 조우 t=121.2~124.9: 커밋 파기(0.66 m) 후 2.49 -> 0.28 m/s, 콘은 0.47 m **뒤**에 있었다.
ever_moved 가 낀 조우 6회의 손실 조우당 +2.34 s vs 나머지 +1.34 s.

### 병목 3 — 회피상실 홀드가 이미 지나친 장애물에도 1 s 걸린다

`local_trajectory/avoidance_loss_hold_sec: 1.0`, 캡 = 마지막 회피경로 머리속도(= 위 ṡ 앵커값).

- 활성 29.0 s (주행의 17.5 %), 캡이 gb 의 78 %, 캡 < 실제속도(= 실제 감속 강제) 33 %.
- 그중 **64.7 %** 는 마지막 회피 대상 뒤끝을 이미 중앙 4.63 m 지난 상태였다.
- 전방 장애물이 실제로 있는 "진짜 드롭아웃 방어" 는 15.5 % 뿐.

### 손실 배분 (s별 무장애물 페이스 기준, 근사)

| 구간 | 시간 | 평균 v | 비고 |
|---|---|---|---|
| committed_hold | 52.0 s | 4.41 | gb 의 71 % |
| hold(회피상실) | 16.4 s | 4.52 | 78 % |
| trailing(장애물 앞) | 11.1 s | 4.36 | |
| trailing(장애물 뒤) | 3.8 s | **2.42** | 562 trailing 사이클의 28 % |
| lane_switch | 3.5 s | 4.75 | gb 의 77 % |
| free_spline | 0.8 s | 5.09 | gb 의 92 % |
| 무장애물 | 41.5 s | 5.81 | |
