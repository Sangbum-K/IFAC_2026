# perception_pkg/test

`run_0809_231809_61_2_radius_gate.json` 은 다시 뽑을 수 없는 실차 회귀 픽스처다.
`test_obstacle_radius_gate.cpp`가 production의 wall-distance predicate, mask 적용,
adaptive clustering, post-filter radius gate를 이 데이터로 검증한다.

출처: 2026-08-09 실차 런(`run_0809_231809`)에서 반경 게이트가 오작동한 순간의
스캔/클러스터 캡처. localization_upgrade 브랜치의 파이썬 `test_obstacle_radius_gate.py`
가 이 데이터를 회귀 케이스로 썼는데, main 이 perception 을 C++ 로 재작성하면서
그 테스트가 임포트할 모듈이 사라졌다 (2026-08-11 머지). 테스트만 지우고
데이터는 남겼다 — 다시 뽑을 수 없는 실차 캡처이기 때문이다.

파이썬 원본 테스트는 `git show origin/localization_upgrade:perception_ws/src/perception_pkg/test/test_obstacle_radius_gate.py`.
fixture는 gtest가 source tree에서 직접 읽는 test-only asset이며 runtime install에는
포함하지 않는다.
