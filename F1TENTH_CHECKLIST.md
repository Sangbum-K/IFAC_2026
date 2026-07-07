# ROS 2 파이프라인 구조를 검증하는 최소 골격

## 1. Interface

- `std_msgs/String` 대신 실제 메시지 타입으로 교체
- 후보:
- `sensor_msgs/msg/LaserScan`
- `sensor_msgs/msg/Image`
- `nav_msgs/msg/Odometry`
- `ackermann_msgs/msg/AckermannDriveStamped`
- 필요 시 커스텀 메시지 패키지 추가

## 2. Perception

- LiDAR 기반 obstacle detection
- free-space extraction 또는 gap detection
- 카메라 사용 시 cone/track marker detection
- 센서 토픽 이름과 frame 정리

## 3. Planning

- local waypoint 생성
- pure pursuit / gap follow / lattice / MPC 중 전략 선택
- track map 또는 centerline 입력 방식 정의
- planning 주기와 latency budget 설정

## 4. Control

- `AckermannDriveStamped` publish로 변경
- steering, speed saturation 추가
- emergency stop 조건 추가
- planner output과 vehicle command 사이 단위 통일

## 5. Integration

- launch 인자 추가
- 예:
- `use_sim_time`
- topic remap
- namespace
- vehicle parameters
- state machine 또는 mission manager 추가 검토

## 6. Configuration

- 패키지별 `config/*.yaml` 추가
- simulator / real car / test track 설정 분리
- vehicle wheelbase, max steer, max speed 파라미터화

## 7. Simulator / Hardware

- `f1tenth_gym_ros` 또는 사용하는 시뮬레이터 연동
- VESC, LiDAR, camera driver 연결
- TF tree 문서화
- 실제 차량 bringup 패키지 분리

## 8. Testing

- `ament_flake8`, `ament_pep257` 등 lint 추가
- basic node smoke test 추가
- launch test 추가
- bag replay 기반 회귀 테스트 추가

## 9. Ops

- 루트 README에 개발 환경 버전 명시
- ROS 2 distro 명시
- 의존 패키지 설치 방법 문서화
- race day 실행 절차 문서화
