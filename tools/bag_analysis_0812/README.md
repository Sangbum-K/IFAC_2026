# run_0812_013630 회피 실패 분석

`rosbag_0811/run_0812_013630` 전량 분석 스크립트. ROS 런타임 불필요 —
`tools/bagviz.py` 의 CDR 디코더를 재사용하고 sqlite3 + numpy 만 쓴다.

리포트: `rosbag_0811/run_0812_analysis.html`

```bash
python3 extract.py     # 92개 토픽 -> data.pkl (~5s, 약 300MB, gitignore됨)
python3 incidents.py   # 사고 4건(75/140/157/194s)의 판단 흐름·조향·서보
python3 grip.py        # 그립 포락선, 조향 응답비, 조향 사용 분포
python3 quant.py       # 사이드락 오발화율, 반사실, 인지 통계
```

다른 백을 보려면 `extract.py` 의 `BAG` 상수만 바꾼다. `incidents.py` 의
`INCIDENTS` 는 (초, 라벨) 목록이다.

`validate_palette.py` 는 리포트 차트 색상 검증용 (dataviz 스킬의
`validate_palette.js` 를 파이썬으로 포팅 — 이 환경에 node 가 없다).

## 핵심 수치 요약

- 회피 판단 275회 중 236회(86%)가 trailing(감속만)으로 떨어짐
- 실측 횡가속도 p99 **8.76 m/s²**, 9.5까지 조향 응답비 저하 없음
  vs `escape_arc_lat_acc` 5.5 / `blend_lat_acc_max` 5.0 (실측의 57~63%)
- `escape_active` 7597 샘플 전부 false — 최후 기동이 한 번도 안 뜸
- 장애물 없는 구간에서도 `|d| > 0.20`(= `side_switch_d_tresh`)인 시간 36.4%
- 3:14 은 인지 0.25s 드롭아웃 -> 레이스라인 속도 복귀(fail-open)가 원인
