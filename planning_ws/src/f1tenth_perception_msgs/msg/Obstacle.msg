int32 id                          # 트랙 ID (프레임 간 동일 물체 유지)
geometry_msgs/Point position       # x, y (laser/base_link frame 기준)
geometry_msgs/Vector3 velocity     # vx, vy [m/s]
bool is_dynamic                    # true: 동적 장애물, false: 정적 장애물
float32 confidence                 # 분류 확신도 (0.0 ~ 1.0)
float32 radius                     # 클러스터 반경 추정치 [m] (충돌 반경 계산용)
# 이번 프레임에 **실제 관측**됐는가. false = KF 예측만으로 외삽(coasting) 중.
# 하류(spliner)가 측정값과 외삽값을 구분할 수 있어야 한다 — 예전에는 이 정보가
# confidence 감쇠에만 담겨 있었고 브릿지가 is_visible=True 로 덮어써서 통째로
# 사라졌다. 그 결과 근거 없는 coasting 트랙이 유일한 통로를 막아 회피가
# 정지로 떨어졌다 (rosbag_0811). 필드를 끝에 추가하므로 기존 발행자와 호환된다
# (미설정 시 false 이므로, 발행자를 반드시 같이 갱신할 것).
bool is_visible                    # true: 이번 프레임 측정됨, false: 외삽(coasting)
