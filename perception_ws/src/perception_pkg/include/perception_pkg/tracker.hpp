/*
 * 매칭(Association) + 상태추정(Kalman Filter) + 재식별(Re-ID) 모듈
 *
 * - 상태벡터: [x, y, vx, vy] (등속도 Constant Velocity 모델), map 프레임.
 * - 매칭: 이전 프레임 트랙 예측 위치 vs 현재 프레임 클러스터 중심을
 *   최근접 이웃(Greedy Nearest Neighbor) 방식으로 매칭.
 *   (장애물 수가 많지 않은 F1TENTH 트랙 특성상 헝가리안 없이도 충분)
 * - 재식별(Re-ID): timeout으로 소멸한 트랙을 잠깐 보관했다가, 곧 근처에서
 *   다시 나타난 클러스터에 같은 id를 부활시킨다. H2H에선 상대가 딱 1대라
 *   occlusion(코너 뒤, 자차 사각) 후에도 같은 id를 유지해야 GP 예측 버퍼와
 *   상대 정체성(ever_dynamic)이 끊기지 않는다.
 * - 위치 이력(pos_hist): 정적/동적 판별의 "위치 분산" 근거 (classifier.hpp).
 *
 * 트랙 컨테이너는 **삽입 순서**를 유지한다 (파이썬 dict와 동일). 그리디 매칭의
 * 동점 처리와 신규 트랙 id 부여 순서가 이 순서에 의존한다.
 */
#ifndef PERCEPTION_PKG__TRACKER_HPP_
#define PERCEPTION_PKG__TRACKER_HPP_

#include <algorithm>
#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "perception_pkg/clustering.hpp"
#include "perception_pkg/types.hpp"

namespace perception_pkg
{

class TrackBounds;

/// 등속도(Constant Velocity) 모델 2D 칼만필터. 상태: [x, y, vx, vy]
class KalmanFilterCV
{
public:
  KalmanFilterCV(double x0, double y0, double process_var = 0.5, double meas_var = 0.05);

  void predict(double dt);
  /// z: 관측된 [x, y] 위치
  void update(double zx, double zy);
  /// 재획득 때는 코너 전의 직선 속도를 승계하지 않는다. 현재 측정 위치에서
  /// 속도를 다시 추정하도록 상태를 초기화한다.
  void reset_to_measurement(double zx, double zy);

  double x() const {return state_(0);}
  double y() const {return state_(1);}
  double vx() const {return state_(2);}
  double vy() const {return state_(3);}
  double speed() const;

private:
  Eigen::Vector4d state_;
  Eigen::Matrix4d cov_;
  double q_base_;
  Eigen::Matrix2d meas_cov_;
};

struct Track
{
  Track(int track_id, const Cluster & cluster, double stamp, double pos_hist_sec);

  void predict(double stamp);
  void update(const Cluster & cluster, double stamp);
  /// 정상 association gate 밖에서 짧은 동적 재획득을 성공했을 때 사용한다.
  /// 위치는 새 측정으로 고정하고, 이전 코너 진입 속도는 버린다.
  void reacquire(const Cluster & cluster, double stamp);
  void mark_missed() {++misses;}

  int id;
  KalmanFilterCV kf;
  double last_stamp;
  /// 마지막으로 실제 cluster가 매칭된 시각. last_stamp는 predict마다 갱신되므로
  /// 재획득 시간 게이트에는 쓰면 안 된다.
  double last_measurement_stamp;
  double radius;
  int hits{1};              ///< 총 관측된 횟수
  int misses{0};            ///< 연속 미관측 횟수
  /// 연속으로 '동적'/'정적' 조건을 만족한 **누적 시간** [s].
  ///
  /// ⚠️ 2026-08-19: 프레임 수(dynamic_streak/static_streak)에서 시간으로 바꿨다.
  /// streak 은 classify_track 이 **실측 프레임에서만** 전진시키는데(코스팅이면
  /// 즉시 반환), 문턱 n_confirm 은 노드가 **발행 주기**로 환산해 넣고 있었다.
  /// run_0818 실측: 발행 39.8 Hz 인데 이동 장애물의 실측 프레임은 33% 뿐이라
  /// 실효 13 Hz —> n_confirm 0.78 s 로 의도한 창이 실제로는 2.33 s 였다.
  /// 조우가 1.5~3 s 라 확정 전에 끝나서, 이동 트랙 50 개 중 23 개만 동적으로
  /// 확정됐다(232744 는 11 개 중 3 개). 시간 적립은 실측률과 무관하게 계약을
  /// 지킨다. 같은 이유로 pos_hist 도 개수가 아니라 시간으로 자른다.
  double dynamic_time{0.0};
  double static_time{0.0};
  /// 직전 측정과 이번 측정 사이의 간격 [s]. classify_track 이 시간을 적립할 때
  /// 쓴다. update()/reacquire() 가 last_measurement_stamp 를 덮기 **전에**
  /// 계산해 둔다 — classifier 는 tracker->step() 이 끝난 뒤에 불리므로
  /// last_measurement_stamp 만으로는 간격을 복원할 수 없다.
  /// 새 트랙과 재획득 직후에는 0 (근거 없는 시간을 적립하지 않는다).
  double last_meas_dt{0.0};
  bool is_dynamic{false};
  /// 한 번이라도 동적 확정된 적이 있나 = "상대 차량" 정체성. 이후 정차/코너
  /// 감속으로 잠깐 느려져도 이 플래그로 dynamic을 오래 유지한다(classifier).
  bool ever_dynamic{false};
  double confidence{0.0};
  /// map 프레임 "측정" 위치 이력 (정적/동적 위치분산 판별용).
  /// 측정된 프레임만 push — 코스팅(외삽) 위치는 분산을 왜곡하므로 넣지 않는다.
  std::deque<Point2> pos_hist;
  /// pos_hist 와 1:1 로 대응하는 측정 시각 [s]. 창을 개수가 아니라 시간으로
  /// 자르기 위한 것 (위 dynamic_time 주석 참조). 두 deque 는 항상 같은
  /// 길이로 유지된다 — push/pop 을 반드시 짝지어 할 것.
  std::deque<double> pos_hist_t;
  /// 위치이력 창 [s]. 이 시간보다 오래된 측정은 버린다.
  double pos_hist_sec;
  /// 메모리 가드. 시각이 깨져도 deque 가 무한정 자라지 않게 하는 상한이며,
  /// 정상 동작에서는 시간 창이 먼저 걸리므로 도달하지 않는다.
  static constexpr std::size_t kPosHistMax{600};
};

using TrackPtr = std::shared_ptr<Track>;

/// 다중 장애물 트래킹 관리자.
/// step(): predict -> associate -> update/spawn(+re-ID) -> prune(-> retire)
class Tracker
{
public:
  Tracker(
    double gating_distance, int track_timeout, double reid_distance,
    double reid_timeout, double pos_hist_sec, double radius_ratio_max,
    double dynamic_reacquire_gate_multiplier,
    double dynamic_reacquire_timeout_sec,
    double dynamic_frenet_reid_timeout_sec,
    double dynamic_frenet_reid_lateral_gate_m,
    double dynamic_frenet_reid_progress_slack_m,
    double dynamic_frenet_reid_backtrack_m);

  /// 한 프레임(스캔) 처리. track_bounds가 있으면 retired 동적 트랙의 긴
  /// 코너/FOV 이탈 재식별만 Frenet s/d 조건으로 보조한다.
  std::vector<TrackPtr> step(
    const std::vector<Cluster> & clusters, double stamp,
    const TrackBounds * track_bounds = nullptr);

  int track_timeout() const {return track_timeout_;}

  /// 프레임 창을 실측 처리율로부터 다시 계산해 넣기 위한 setter.
  /// (초 단위 파라미터 -> 프레임 변환은 노드가 담당한다. tracker 는 프레임
  ///  도메인에서만 동작한다 — 스캔이 균일 간격이 아니어도 '연속 미관측 N회'
  ///  라는 의미가 흔들리지 않기 때문이다.)
  /// pos_hist_sec 변경은 **새로 생성되는 트랙**부터 적용된다.
  void set_track_timeout(int frames) {track_timeout_ = std::max(1, frames);}
  void set_pos_hist_sec(double sec) {pos_hist_sec_ = std::max(0.05, sec);}

private:
  struct Retired
  {
    int id;
    double x, y, vx, vy, radius, t;
    /// 마지막 실제 cluster 관측. x/y는 retire 시점의 KF coast 위치이므로,
    /// 코너/FOV 이탈 재식별의 Frenet 기준점으로 쓰면 안 된다.
    double last_measurement_x, last_measurement_y, last_measurement_stamp;
    double dynamic_time, static_time;
    double confidence;
    bool ever_dynamic;
    bool is_dynamic;
  };

  bool radius_matches(const Track & track, const Cluster & cluster) const;
  bool radius_matches(const Retired & retired, const Cluster & cluster) const;
  TrackPtr revive(const Retired & retired, const Cluster & cluster, double stamp) const;
  void retire(const Track & track, double stamp);
  void spawn(const Cluster & cluster, double stamp);

  std::vector<TrackPtr> tracks_;   ///< 삽입 순서 유지 (파이썬 dict 동등)
  std::vector<Retired> retired_;   ///< 최근 소멸 트랙 (재식별 후보)
  int next_id_{0};
  double gating_distance_;         ///< 매칭 허용 최대 거리 [m]
  int track_timeout_;              ///< 연속 미관측 허용 프레임 수
  double reid_distance_;           ///< 재식별 허용 거리 [m]
  double reid_timeout_;            ///< 소멸 트랙 보관 시간 [s]
  double pos_hist_sec_;
  /// 크기 일관성 게이트: 트랙이 보던 반경과 후보 클러스터 반경의 비가
  /// 이 배수를 넘으면 같은 물체로 보지 않는다. 1 이하면 비활성.
  double radius_ratio_max_;
  /// 표준 association이 한 번 실패한 동적 트랙에만 적용하는 별도 재획득 gate.
  /// 일반 gate를 키우지 않아 정적 클러스터의 ID 탈취를 피한다.
  double dynamic_reacquire_gate_multiplier_;
  /// 마지막 실측 후 이 시간 안에서만 active dynamic 재획득을 허용한다 [s].
  double dynamic_reacquire_timeout_sec_;
  /// retired 동적 트랙을 Frenet 조건으로 보조 재식별할 cache 수명 [s].
  double dynamic_frenet_reid_timeout_sec_;
  /// Frenet 보조 재식별 때 허용하는 횡방향(d) 차이 [m].
  double dynamic_frenet_reid_lateral_gate_m_;
  /// 직선 CV 대신 트랙 진행량으로 재식별할 때 속도 적분에 더하는 여유 [m].
  double dynamic_frenet_reid_progress_slack_m_;
  /// Frenet 보조 재식별에서 허용하는 역방향 진행량 [m].
  double dynamic_frenet_reid_backtrack_m_;
};

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__TRACKER_HPP_
