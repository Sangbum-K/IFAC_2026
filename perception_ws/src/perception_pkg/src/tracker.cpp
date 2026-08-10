#include "perception_pkg/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <tuple>

namespace perception_pkg
{

// ── KalmanFilterCV ────────────────────────────────────────────────────────
KalmanFilterCV::KalmanFilterCV(double x0, double y0, double process_var, double meas_var)
: state_(x0, y0, 0.0, 0.0),
  cov_(Eigen::Matrix4d::Identity()),
  q_base_(process_var),
  meas_cov_(Eigen::Matrix2d::Identity() * meas_var)
{
}

void KalmanFilterCV::predict(double dt)
{
  Eigen::Matrix4d f = Eigen::Matrix4d::Identity();
  f(0, 2) = dt;
  f(1, 3) = dt;

  // 등속도 모델용 프로세스 노이즈 (표준적인 discretized white noise acceleration)
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  Eigen::Matrix4d q = Eigen::Matrix4d::Zero();
  q(0, 0) = q(1, 1) = dt4 / 4.0;
  q(2, 2) = q(3, 3) = dt2;
  q(0, 2) = q(2, 0) = q(1, 3) = q(3, 1) = dt3 / 2.0;
  q *= q_base_;

  state_ = f * state_;
  cov_ = f * cov_ * f.transpose() + q;
}

void KalmanFilterCV::update(double zx, double zy)
{
  // H = [[1,0,0,0],[0,1,0,0]] 이므로 H와의 곱은 부분행렬 접근으로 대체한다.
  const Eigen::Vector2d innovation(zx - state_(0), zy - state_(1));
  const Eigen::Matrix2d s = cov_.topLeftCorner<2, 2>() + meas_cov_;
  const Eigen::Matrix<double, 4, 2> k = cov_.leftCols<2>() * s.inverse();

  state_ += k * innovation;

  Eigen::Matrix4d kh = Eigen::Matrix4d::Zero();
  kh.leftCols<2>() = k;
  cov_ = (Eigen::Matrix4d::Identity() - kh) * cov_;
}

double KalmanFilterCV::speed() const
{
  return std::hypot(state_(2), state_(3));
}

// ── Track ─────────────────────────────────────────────────────────────────
Track::Track(int track_id, const Cluster & cluster, double stamp, std::size_t hist_len)
: id(track_id),
  kf(cluster.centroid[0], cluster.centroid[1]),
  last_stamp(stamp),
  radius(cluster.radius),
  pos_hist_len(hist_len)
{
  pos_hist.push_back(cluster.centroid);
}

void Track::predict(double stamp)
{
  const double dt = std::max(stamp - last_stamp, 1e-3);
  kf.predict(dt);
  last_stamp = stamp;
}

void Track::update(const Cluster & cluster, double stamp)
{
  kf.update(cluster.centroid[0], cluster.centroid[1]);
  radius = cluster.radius;
  ++hits;
  misses = 0;
  last_stamp = stamp;
  pos_hist.push_back(cluster.centroid);
  while (pos_hist.size() > pos_hist_len) {
    pos_hist.pop_front();
  }
}

// ── Tracker ───────────────────────────────────────────────────────────────
Tracker::Tracker(
  double gating_distance, int track_timeout, double reid_distance,
  double reid_timeout, int pos_hist_len, double radius_ratio_max)
: gating_distance_(gating_distance),
  track_timeout_(track_timeout),
  reid_distance_(reid_distance),
  reid_timeout_(reid_timeout),
  pos_hist_len_(static_cast<std::size_t>(std::max(1, pos_hist_len))),
  radius_ratio_max_(radius_ratio_max)
{
}

std::vector<TrackPtr> Tracker::step(const std::vector<Cluster> & clusters, double stamp)
{
  // 1) 모든 트랙 예측 (dt 반영)
  for (auto & track : tracks_) {
    track->predict(stamp);
  }

  // 2) Greedy Nearest Neighbor 매칭
  //    (트랙, 클러스터) 거리 쌍을 모두 계산 후 가까운 순으로 정렬해 그리디 매칭.
  struct Pair
  {
    double d;
    std::size_t ti;
    std::size_t ci;
  };
  std::vector<Pair> pairs;
  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
    const Track & track = *tracks_[ti];
    const double tx = track.kf.x();
    const double ty = track.kf.y();
    for (std::size_t ci = 0; ci < clusters.size(); ++ci) {
      const double d = std::hypot(tx - clusters[ci].centroid[0], ty - clusters[ci].centroid[1]);
      if (d > gating_distance_) {
        continue;
      }
      // ── 크기 일관성 게이트 ────────────────────────────────────
      // 거리만 보면 근처의 작은 노이즈 클러스터가 진짜 물체보다 가깝다는
      // 이유로 트랙 ID를 가져간다. rosbag_0806 run_234734 #53 실측: 반경
      // 0.157m 실물과 반경 0.005m 파편 사이를 같은 ID가 왕복해 KF 속도가
      // 0.47 m/s로 튀었고 정지 물체가 "동적"으로 분류됐다.
      // 같은 물체라면 반경이 프레임 간 몇 배씩 변하지 않는다.
      if (radius_ratio_max_ > 1.0) {
        const double r_t = std::max(track.radius, 1e-3);
        const double r_c = std::max(clusters[ci].radius, 1e-3);
        if (std::max(r_t / r_c, r_c / r_t) > radius_ratio_max_) {
          continue;
        }
      }
      pairs.push_back({d, ti, ci});
    }
  }
  // 동점은 생성 순서(트랙 삽입 순서 -> 클러스터 순서)로 깨진다.
  std::stable_sort(
    pairs.begin(), pairs.end(),
    [](const Pair & a, const Pair & b) {return a.d < b.d;});

  std::vector<uint8_t> track_matched(tracks_.size(), 0);
  std::vector<uint8_t> cluster_used(clusters.size(), 0);
  for (const auto & p : pairs) {
    if (track_matched[p.ti] || cluster_used[p.ci]) {
      continue;
    }
    tracks_[p.ti]->update(clusters[p.ci], stamp);
    track_matched[p.ti] = 1;
    cluster_used[p.ci] = 1;
  }

  // 3) 매칭 안 된 트랙: miss 처리
  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
    if (!track_matched[ti]) {
      tracks_[ti]->mark_missed();
    }
  }

  // 4) 매칭 안 된 클러스터: 재식별 시도 후 신규 트랙 생성
  for (std::size_t ci = 0; ci < clusters.size(); ++ci) {
    if (!cluster_used[ci]) {
      spawn(clusters[ci], stamp);
    }
  }

  // 5) 오래 미관측된 트랙 제거 -> 재식별용으로 잠깐 보관(retire)
  std::vector<TrackPtr> survivors;
  survivors.reserve(tracks_.size());
  for (auto & track : tracks_) {
    if (track->misses > track_timeout_) {
      retire(*track, stamp);
    } else {
      survivors.push_back(track);
    }
  }
  tracks_.swap(survivors);

  // 6) 보관 기한 지난 재식별 후보 폐기
  retired_.erase(
    std::remove_if(
      retired_.begin(), retired_.end(),
      [&](const Retired & r) {return stamp - r.t > reid_timeout_;}),
    retired_.end());

  return tracks_;
}

void Tracker::retire(const Track & track, double stamp)
{
  retired_.push_back(
    Retired{
      track.id, track.kf.x(), track.kf.y(), track.kf.vx(), track.kf.vy(), stamp,
      track.ever_dynamic, track.is_dynamic});
}

void Tracker::spawn(const Cluster & cluster, double stamp)
{
  const double cx = cluster.centroid[0];
  const double cy = cluster.centroid[1];

  // 재식별: 최근 소멸 트랙을 등속 외삽한 예측 위치가 이 클러스터와
  // reid_distance 이내이면 같은 id를 부활 (정체성 유지). 자차 사각/코너
  // 뒤 occlusion 후 재등장하는 상대가 새 id를 받지 않게 한다.
  bool found = false;
  std::size_t best_k = 0;
  double best_d = reid_distance_;
  for (std::size_t k = 0; k < retired_.size(); ++k) {
    const Retired & r = retired_[k];
    const double dt = stamp - r.t;
    const double px = r.x + r.vx * dt;
    const double py = r.y + r.vy * dt;
    const double d = std::hypot(px - cx, py - cy);
    if (d <= best_d) {
      best_d = d;
      best_k = k;
      found = true;
    }
  }
  if (found) {
    const Retired r = retired_[best_k];
    retired_.erase(retired_.begin() + static_cast<long>(best_k));
    auto track = std::make_shared<Track>(r.id, cluster, stamp, pos_hist_len_);
    track->ever_dynamic = r.ever_dynamic;
    track->is_dynamic = r.is_dynamic;   // 재등장 즉시 정체성 복원
    tracks_.push_back(track);
    return;
  }

  tracks_.push_back(std::make_shared<Track>(next_id_, cluster, stamp, pos_hist_len_));
  ++next_id_;
}

}  // namespace perception_pkg
