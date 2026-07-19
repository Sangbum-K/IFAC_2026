int32 id                          # 트랙 ID (프레임 간 동일 물체 유지)
geometry_msgs/Point position       # x, y (laser/base_link frame 기준)
geometry_msgs/Vector3 velocity     # vx, vy [m/s]
bool is_dynamic                    # true: 동적 장애물, false: 정적 장애물
float32 confidence                 # 분류 확신도 (0.0 ~ 1.0)
float32 radius                     # 클러스터 반경 추정치 [m] (충돌 반경 계산용)
