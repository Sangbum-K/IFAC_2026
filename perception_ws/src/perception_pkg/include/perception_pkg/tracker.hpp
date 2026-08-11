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

/// 등속도(Constant Velocity) 모델 2D 칼만필터. 상태: [x, y, vx, vy]
class KalmanFilterCV
{
public:
  KalmanFilterCV(double x0, double y0, double process_var = 0.5, double meas_var = 0.05);

  void predict(double dt);
  /// z: 관측된 [x, y] 위치
  void update(double zx, double zy);

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
  Track(int track_id, const Cluster & cluster, double stamp, std::size_t pos_hist_len);

  void predict(double stamp);
  void update(const Cluster & cluster, double stamp);
  void mark_missed() {++misses;}

  int id;
  KalmanFilterCV kf;
  double last_stamp;
  double radius;
  int hits{1};              ///< 총 관측된 횟수
  int misses{0};            ///< 연속 미관측 횟수
  int dynamic_streak{0};    ///< 연속으로 '동적' 조건을 만족한 프레임 수
  int static_streak{0};     ///< 연속으로 '정적' 조건을 만족한 프레임 수
  bool is_dynamic{false};
  /// 한 번이라도 동적 확정된 적이 있나 = "상대 차량" 정체성. 이후 정차/코너
  /// 감속으로 잠깐 느려져도 이 플래그로 dynamic을 오래 유지한다(classifier).
  bool ever_dynamic{false};
  double confidence{0.0};
  /// map 프레임 "측정" 위치 이력 (정적/동적 위치분산 판별용).
  /// 측정된 프레임만 push — 코스팅(외삽) 위치는 분산을 왜곡하므로 넣지 않는다.
  std::deque<Point2> pos_hist;
  std::size_t pos_hist_len;
};

using TrackPtr = std::shared_ptr<Track>;

/// 다중 장애물 트래킹 관리자.
/// step(): predict -> associate -> update/spawn(+re-ID) -> prune(-> retire)
class Tracker
{
public:
  Tracker(
    double gating_distance, int track_timeout, double reid_distance,
    double reid_timeout, int pos_hist_len, double radius_ratio_max);

  /// 한 프레임(스캔) 처리. 반환값은 현재 활성 트랙들.
  std::vector<TrackPtr> step(const std::vector<Cluster> & clusters, double stamp);

  int track_timeout() const {return track_timeout_;}

  /// 프레임 창을 실측 처리율로부터 다시 계산해 넣기 위한 setter.
  /// (초 단위 파라미터 -> 프레임 변환은 노드가 담당한다. tracker 는 프레임
  ///  도메인에서만 동작한다 — 스캔이 균일 간격이 아니어도 '연속 미관측 N회'
  ///  라는 의미가 흔들리지 않기 때문이다.)
  /// pos_hist_len 변경은 **새로 생성되는 트랙**부터 적용된다.
  void set_track_timeout(int frames) {track_timeout_ = std::max(1, frames);}
  void set_pos_hist_len(int frames)
  {
    pos_hist_len_ = static_cast<std::size_t>(std::max(2, frames));
  }

private:
  struct Retired
  {
    int id;
    double x, y, vx, vy, t;
    bool ever_dynamic;
    bool is_dynamic;
  };

  void retire(const Track & track, double stamp);
  void spawn(const Cluster & cluster, double stamp);

  std::vector<TrackPtr> tracks_;   ///< 삽입 순서 유지 (파이썬 dict 동등)
  std::vector<Retired> retired_;   ///< 최근 소멸 트랙 (재식별 후보)
  int next_id_{0};
  double gating_distance_;         ///< 매칭 허용 최대 거리 [m]
  int track_timeout_;              ///< 연속 미관측 허용 프레임 수
  double reid_distance_;           ///< 재식별 허용 거리 [m]
  double reid_timeout_;            ///< 소멸 트랙 보관 시간 [s]
  std::size_t pos_hist_len_;
  /// 크기 일관성 게이트: 트랙이 보던 반경과 후보 클러스터 반경의 비가
  /// 이 배수를 넘으면 같은 물체로 보지 않는다. 1 이하면 비활성.
  double radius_ratio_max_;
};

}  // namespace perception_pkg

#endif  // PERCEPTION_PKG__TRACKER_HPP_
