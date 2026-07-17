# Pitwall 패널 + 텔레메트리 구현 TODO (목표: 오늘 완성)

> 참고: [Pitwall Panel Usage](https://hmcl-unist.github.io/unicorn-racing/posts/pitwall-panel-usage/),
> [Pitwall Telemetry](https://hmcl-unist.github.io/unicorn-racing/posts/pitwall-telemetry/)
> 작성: planning 팀, 2026-07-17

## 무엇을 만드는가

1. **Pitwall 텔레메트리** — 코드 내부 변수를 토픽 정의 없이 한 줄로 노출하는
   초경량 라이브러리. `pitwall.log("speed", v)` → `/pitwall/speed` 발행,
   `pitwall.event("벽 충돌")` → `/pitwall/events` 로그. rqt_plot으로 즉시 그래프.
2. **Pitwall 패널** — RViz 도킹 패널(Qt 플러그인). 주행 STATE 배너,
   이벤트 로그, 상대 속도 ±/제거 버튼, 장애물 일괄 삭제 버튼.

## 현재 보유 vs 필요 (요약)

| 튜토리얼 기능 | 우리 스택 현황 | 오늘 할 일 |
|---|---|---|
| Publish Point 정적 장애물 | ✅ virtual_obstacle_node (재클릭 삭제까지) | Clear-all 토픽만 추가 |
| 2D Goal Pose 상대 스폰/제거 | ✅ virtual_opponent_node | 유지 |
| 상대 속도 ±0.5 버튼 | ⚠️ speed_scale 파라미터(기동 시 고정) | 런타임 변경 토픽 추가 |
| 상대 주행 모드 (Manual/Path/FTG) | ⚠️ Path(레이스라인 추종)만 | **오늘 범위 제외** (Path 고정) |
| STATE 배너 (GB_TRACK/TRAILING/OVERTAKE/RECOVERY) | ❌ 상태머신 없음 | 기존 토픽에서 상태 유도 (아래 T2.2) |
| ego Joy/Keyboard 모드 | ❌ | **오늘 범위 제외** |
| LiDAR 토글 / Perception Injection | ❌ (가상 상대가 ghost라 개념 없음) | **오늘 범위 제외** |
| pitwall.log / event | ❌ | **신규 구현 (Phase 1)** |
| RViz 패널 플러그인 | ❌ | **신규 구현 (Phase 2)** |

---

## Phase 1 — pitwall 텔레메트리 라이브러리 (예상 2~3h, Python 담당)

### T1.1 Python 모듈 `planning_pkg/scripts/lib/pitwall.py`
- API (튜토리얼과 동일):
  ```python
  import pitwall
  pitwall.init(self)                    # 노드 __init__에서 1회
  pitwall.log("speed", value)           # -> /pitwall/speed (std_msgs/Float32)
  pitwall.event("벽 충돌 감지")           # -> /pitwall/events (std_msgs/String)
  ```
- 구현 규칙: publisher는 이름별 lazy 생성 + 캐시. event 문자열 포맷
  `"[<sec>.<ns>][<node_name>] <text>"`. import 실패나 init 누락 시
  **조용히 no-op** (계측이 주행 코드를 죽이면 안 됨).
- 설치: `CMakeLists.txt`에서 lib/ 폴더째 lib/planning_pkg로 설치,
  각 노드에서 `sys.path` 추가 없이 import 되도록 배치 확인.

### T1.2 C++ 헤더 `planning_pkg/include/planning_pkg/pitwall.hpp`
- 동일 API: `pitwall::Init(node)`, `pitwall::Log("name", v)`, `pitwall::Event("...")`
- header-only, `std::unordered_map<std::string, Publisher>` 캐시.
- control_ws에서도 쓸 수 있게 planning_pkg install/include로 내보내기.

### T1.3 계측 지점 심기 (최소 세트 — 각 1줄)
| 노드 | log | event |
|---|---|---|
| waypoint_tracker / control_node | `cmd_speed`, `cmd_steer`, `lookahead` | 스턱 감지, 복구 완료 |
| opponent_predictor | `opp_gap`(상대와 s 간격), `pred_catch_s` | "GP 학습 완료", "예측 발동" |
| spliner_node (C++) | `evasion_d_apex` | "회피 포기(트랙경계)", "사이드 스위치 거부" |
| frenet_frame_node (C++) | `lat_err`(d) | pose-jump 재탐색 |
| local_trajectory_node (C++) | `spliced_wpnts` | — |

### T1.4 검증 & 사용법 문서
- `ros2 run rqt_plot rqt_plot /pitwall/lat_err/data /pitwall/cmd_speed/data`
- `ros2 topic echo /pitwall/events`
- 기록: `ros2 bag record -s mcap -e '/pitwall/.*' -o pitwall_$(date +%m%d)`
- README에 위 3줄 추가.

**완료 기준**: sim_race 주행 중 rqt_plot에 lat_err/cmd_speed 곡선이 그려지고,
장애물 클릭 시 events에 회피 관련 이벤트가 찍힌다.

---

## Phase 2 — RViz Pitwall 패널 (예상 4~6h, C++/Qt 담당)

### T2.1 패키지 골격 `planning_ws/src/pitwall_panel/`
- `rviz_common::Panel` 상속 Qt 위젯. 의존: rviz_common, pluginlib, Qt5::Widgets.
- `plugin_description.xml` + `CMakeLists.txt`(pluginlib export) —
  rviz2 Panels 메뉴 → "Add New Panel" → PitwallPanel 뜨면 OK.
- 참고 예시: `rviz_common` 튜토리얼 패널 (humble 문서의 DockPanel 예제).

### T2.2 STATE 배너 (상태머신 나오기 전 임시 유도 규칙)
- 구독: `/planner/avoidance/otwpnts`, `/opponent_prediction/obstacles`, `/pitwall/events`
- 표시 규칙 (0.5s 주기 평가):
  - otwpnts 비어있지 않음 → **OVERTAKE** (보라)
  - 예측 장애물 존재(추격 중) → **TRAILING** (노랑)
  - events에 "복구 시작" 최근 수신 → **RECOVERY** (빨강, 복구 완료 시 해제)
  - 그 외 → **GB_TRACK** (초록)
- ※ H2H 상태머신(BehaviorState 발행 노드)이 생기면 이 유도 규칙을
  `/behavior_state` 구독으로 교체한다 — 배너 위젯은 그대로.

### T2.3 이벤트 로그 뷰
- `/pitwall/events` 최근 30줄 QListWidget, 자동 스크롤, Clear 버튼.

### T2.4 상대 차량 컨트롤 (버튼 4개)
- `[- 0.5]` `[+ 0.5]` → `/sim/opp_speed_delta` (std_msgs/Float32)
- `[Remove opponent]` → `/sim/remove_opponent` (std_msgs/Empty)
- 현재 상대 속도 라벨: `/perception/opponent`의 vs 표시
- **연동 수정 (Python 담당과 협업)**: `virtual_opponent_node.py`에
  두 토픽 구독 추가 — speed_delta는 `speed_scale`을 raceline 속도 기준으로
  환산해 가감 (`speed_scale += delta / raceline_v_mean`), remove는 `active=False`.

### T2.5 정적 장애물 Clear
- `[Clear static obstacles]` → `/sim/clear_obstacles` (std_msgs/Empty)
- **연동 수정**: `virtual_obstacle_node.py`에 구독 추가 → `obstacles.clear()`.

### T2.6 rviz 설정 반영
- `sim_race.rviz`의 Panels 목록에 PitwallPanel 추가 (기본 표시).
- sim_race.launch.py는 수정 불필요 (패널은 rviz가 로드).

**완료 기준**: sim_race 실행 → 패널이 우측에 표시되고,
① 주행 중 GB_TRACK(초록), 상대 스폰 후 추격 시 TRAILING, 회피 스플라인
생성 중 OVERTAKE로 배너가 바뀐다 ② +/- 버튼으로 상대 속도가 실시간
변한다 ③ Remove/Clear 버튼이 동작한다 ④ 이벤트 로그에 pitwall.event가 찍힌다.

---

## 토픽 계약 (신규 — 다른 팀도 이 이름 그대로 사용)

| 토픽 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/pitwall/<name>` | std_msgs/Float32 | 각 노드 → | `pitwall.log(name, v)` 값 |
| `/pitwall/events` | std_msgs/String | 각 노드 → 패널 | `pitwall.event(text)` 로그 |
| `/sim/opp_speed_delta` | std_msgs/Float32 | 패널 → virtual_opponent | 상대 속도 증감 [m/s] |
| `/sim/remove_opponent` | std_msgs/Empty | 패널 → virtual_opponent | 상대 제거 |
| `/sim/clear_obstacles` | std_msgs/Empty | 패널 → virtual_obstacle | 정적 장애물 전체 삭제 |

## 분담 제안

| 담당 | 작업 | 예상 |
|---|---|---|
| A (Python) | T1.1 → T1.3(Python 노드) → T1.4 → T2.4/T2.5 노드측 구독 | 3~4h |
| B (C++) | T2.1 골격 → T2.3 → T2.4/T2.5 버튼 → T2.2 배너 → T2.6 | 4~6h |
| 공통 | T1.2(C++ pitwall.hpp) + T1.3(C++ 노드 계측)은 A/B 중 먼저 끝나는 쪽 | 1h |

병렬 진행 가능: Phase 1과 T2.1~T2.3은 서로 독립.
통합 지점은 T2.4/T2.5(토픽 계약)와 T2.2(events 포맷) 두 곳뿐이므로
**위 토픽 계약 표만 지키면 충돌 없음**.

## 오늘 범위에서 뺀 것 (다음 이터레이션)

- 상대 FTG/Manual 주행 모드 (지금은 Path=레이스라인 추종 고정)
- ego Joy/Keyboard 전환 (gym kb_teleop로 대체 가능)
- LiDAR 토글, Perception Injection 모드 (가상 상대가 ghost라 현재 무의미)
- STATE 배너의 정식 BehaviorState 연동 (H2H 상태머신 구현 후)
