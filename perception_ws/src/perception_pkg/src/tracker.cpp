#include "perception_pkg/tracker.hpp"

#include "perception_pkg/track_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <tuple>

namespace perception_pkg
{

// ── KalmanFilterCV ────────────────────────────────────────────────────────
KalmanFilterCV::KalmanFilterCV(
  double x0,
  double y0,
  double process_var,
  double meas_var)
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

  // 등속도(Constant Velocity) 모델용 process noise
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;

  Eigen::Matrix4d q = Eigen::Matrix4d::Zero();

  q(0, 0) = dt4 / 4.0;
  q(1, 1) = dt4 / 4.0;

  q(2, 2) = dt2;
  q(3, 3) = dt2;

  q(0, 2) = dt3 / 2.0;
  q(2, 0) = dt3 / 2.0;

  q(1, 3) = dt3 / 2.0;
  q(3, 1) = dt3 / 2.0;

  q *= q_base_;

  state_ = f * state_;
  cov_ = f * cov_ * f.transpose() + q;
}

void KalmanFilterCV::update(double zx, double zy)
{
  // H =
  // [1 0 0 0]
  // [0 1 0 0]

  const Eigen::Vector2d innovation(
    zx - state_(0),
    zy - state_(1));

  const Eigen::Matrix2d s =
    cov_.topLeftCorner<2, 2>() + meas_cov_;

  const Eigen::Matrix<double, 4, 2> k =
    cov_.leftCols<2>() * s.inverse();

  state_ += k * innovation;

  Eigen::Matrix4d kh = Eigen::Matrix4d::Zero();
  kh.leftCols<2>() = k;

  cov_ =
    (Eigen::Matrix4d::Identity() - kh) *
    cov_;
}

void KalmanFilterCV::reset_to_measurement(double zx, double zy)
{
  // 코너에서 CV 예측이 크게 벗어난 뒤에는 이전 속도를 그대로 이어 받으면
  // 다음 predict도 같은 접선 방향으로 다시 튄다. 재획득은 새 측정에서
  // 시작하고, 속도는 다음 연속 관측으로 다시 추정한다.
  state_ << zx, zy, 0.0, 0.0;
  cov_.setIdentity();
}

double KalmanFilterCV::speed() const
{
  return std::hypot(state_(2), state_(3));
}


// ── Track ─────────────────────────────────────────────────────────────────
Track::Track(
  int track_id,
  const Cluster & cluster,
  double stamp,
  std::size_t hist_len)
: id(track_id),
  kf(cluster.centroid[0], cluster.centroid[1]),
  last_stamp(stamp),
  last_measurement_stamp(stamp),
  radius(cluster.radius),
  pos_hist_len(hist_len)
{
  pos_hist.push_back(cluster.centroid);
}

void Track::predict(double stamp)
{
  const double dt =
    std::max(stamp - last_stamp, 1e-3);

  kf.predict(dt);

  last_stamp = stamp;
}

void Track::update(
  const Cluster & cluster,
  double stamp)
{
  kf.update(
    cluster.centroid[0],
    cluster.centroid[1]);

  radius = cluster.radius;

  ++hits;

  // 다시 관측되었으므로 miss 초기화
  misses = 0;

  last_stamp = stamp;
  last_measurement_stamp = stamp;

  pos_hist.push_back(cluster.centroid);

  while (pos_hist.size() > pos_hist_len) {
    pos_hist.pop_front();
  }
}

void Track::reacquire(
  const Cluster & cluster,
  double stamp)
{
  // 동적 ID/분류 이력은 유지하되, 코너 전 CV 속도와 오래된 위치 이력은
  // 새 물체에 붙였을 때의 오차를 키울 수 있으므로 측정 기준으로 다시 시작한다.
  kf.reset_to_measurement(cluster.centroid[0], cluster.centroid[1]);
  radius = cluster.radius;
  ++hits;
  misses = 0;
  last_stamp = stamp;
  last_measurement_stamp = stamp;
  pos_hist.clear();
  pos_hist.push_back(cluster.centroid);
}


// ── Tracker ───────────────────────────────────────────────────────────────
Tracker::Tracker(
  double gating_distance,
  int track_timeout,
  double reid_distance,
  double reid_timeout,
  int pos_hist_len,
  double radius_ratio_max,
  double dynamic_reacquire_gate_multiplier,
  double dynamic_reacquire_timeout_sec,
  double dynamic_frenet_reid_timeout_sec,
  double dynamic_frenet_reid_lateral_gate_m,
  double dynamic_frenet_reid_progress_slack_m,
  double dynamic_frenet_reid_backtrack_m)
: gating_distance_(gating_distance),
  track_timeout_(track_timeout),
  reid_distance_(reid_distance),
  reid_timeout_(reid_timeout),
  pos_hist_len_(
    static_cast<std::size_t>(
      std::max(1, pos_hist_len))),
  radius_ratio_max_(radius_ratio_max),
  dynamic_reacquire_gate_multiplier_(
    std::max(dynamic_reacquire_gate_multiplier, 1.0)),
  dynamic_reacquire_timeout_sec_(
    std::max(dynamic_reacquire_timeout_sec, 0.0)),
  dynamic_frenet_reid_timeout_sec_(
    std::max(dynamic_frenet_reid_timeout_sec, 0.0)),
  dynamic_frenet_reid_lateral_gate_m_(
    std::max(dynamic_frenet_reid_lateral_gate_m, 0.0)),
  dynamic_frenet_reid_progress_slack_m_(
    std::max(dynamic_frenet_reid_progress_slack_m, 0.0)),
  dynamic_frenet_reid_backtrack_m_(
    std::max(dynamic_frenet_reid_backtrack_m, 0.0))
{
}

bool Tracker::radius_matches(const Track & track, const Cluster & cluster) const
{
  if (radius_ratio_max_ <= 1.0) {
    return true;
  }
  const double r_track = std::max(track.radius, 1e-3);
  const double r_cluster = std::max(cluster.radius, 1e-3);
  const double ratio = std::max(r_track / r_cluster, r_cluster / r_track);
  return ratio <= radius_ratio_max_;
}

bool Tracker::radius_matches(const Retired & retired, const Cluster & cluster) const
{
  if (radius_ratio_max_ <= 1.0) {
    return true;
  }
  const double r_retired = std::max(retired.radius, 1e-3);
  const double r_cluster = std::max(cluster.radius, 1e-3);
  const double ratio = std::max(r_retired / r_cluster, r_cluster / r_retired);
  return ratio <= radius_ratio_max_;
}

TrackPtr Tracker::revive(
  const Retired & retired, const Cluster & cluster, double stamp) const
{
  auto track = std::make_shared<Track>(
    retired.id, cluster, stamp, pos_hist_len_);
  track->ever_dynamic = retired.ever_dynamic;
  track->is_dynamic = retired.is_dynamic;
  track->dynamic_streak = retired.dynamic_streak;
  track->static_streak = retired.static_streak;
  track->confidence = retired.confidence;

  // 현 관측 위치에서 다시 시작하고, 코너 전 접선 속도는 재사용하지 않는다.
  track->reacquire(cluster, stamp);
  return track;
}

std::vector<TrackPtr> Tracker::step(
  const std::vector<Cluster> & clusters,
  double stamp,
  const TrackBounds * track_bounds)
{
  // ================================================================
  // 1. 모든 살아있는 Track에 대해 prediction
  //
  // 관측 여부와 관계없이 먼저 predict한다.
  //
  // 즉 dynamic obstacle이 잠시 안 보여도:
  //
  // last measurement
  //       ↓
  // KF.predict
  //       ↓
  // predicted position
  //
  // 상태를 유지한다.
  // ================================================================
  for (auto & track : tracks_) {
    track->predict(stamp);
  }


  // ================================================================
  // 2. Association candidate 생성
  // ================================================================
  struct Pair
  {
    double d;
    std::size_t ti;
    std::size_t ci;
  };

  std::vector<Pair> pairs;

  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {

    const Track & track = *tracks_[ti];

    // ★ 이미 predict() 된 위치
    const double tx = track.kf.x();
    const double ty = track.kf.y();


    for (std::size_t ci = 0; ci < clusters.size(); ++ci) {

      const double cx = clusters[ci].centroid[0];
      const double cy = clusters[ci].centroid[1];

      const double d =
        std::hypot(
        tx - cx,
        ty - cy);


      // 일반 association은 동적/정적에 같은 엄격한 gate를 쓴다. 동적만
      // gate를 넓히면 코너 근처 정적 cluster가 상대 ID를 탈취할 수 있다.
      if (d > gating_distance_) {
        continue;
      }

      // 근처 작은 noise cluster가 기존 obstacle ID를 탈취하는 현상 방지.
      if (!radius_matches(track, clusters[ci])) {
        continue;
      }


      pairs.push_back(
        Pair{d, ti, ci});
    }
  }


  // ================================================================
  // 3. Greedy nearest-neighbor
  // ================================================================
  std::stable_sort(
    pairs.begin(),
    pairs.end(),
    [](const Pair & a, const Pair & b)
    {
      return a.d < b.d;
    });


  std::vector<uint8_t>
  track_matched(
    tracks_.size(), 0);

  std::vector<uint8_t>
  cluster_used(
    clusters.size(), 0);


  for (const auto & p : pairs) {

    if (
      track_matched[p.ti] ||
      cluster_used[p.ci])
    {
      continue;
    }


    tracks_[p.ti]->update(
      clusters[p.ci],
      stamp);

    track_matched[p.ti] = 1;
    cluster_used[p.ci] = 1;
  }

  // ================================================================
  // 4. Active dynamic re-acquisition
  //
  // 코너에서 CV 예측이 직선 접선 방향으로 벗어나면 primary gate를 살짝
  // 넘는 순간에도 기존 트랙은 active인 채 남고, 이전 구현은 같은 cluster를
  // 곧바로 새 ID로 spawn했다. 여기서는 "이번 primary match에 실패한"
  // ever_dynamic 트랙만 마지막 실측 위치를 기준으로 한 번 더 확인한다.
  //
  // 일반 gate는 절대 넓히지 않는다. 짧은 관측 공백, 반경 일치, prediction
  // 이탈 상한, cluster당 단일 후보를 모두 만족할 때만 이 좁은 경로를 쓴다.
  // ================================================================
  const double recovery_gate = std::min(
    reid_distance_, gating_distance_ * dynamic_reacquire_gate_multiplier_);
  std::vector<Pair> recovery_pairs;
  std::vector<std::size_t> recovery_candidates(clusters.size(), 0);

  if (recovery_gate > 0.0 && dynamic_reacquire_timeout_sec_ > 0.0) {
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
      const Track & track = *tracks_[ti];
      if (track_matched[ti] || !track.ever_dynamic || track.pos_hist.empty() ||
        stamp - track.last_measurement_stamp > dynamic_reacquire_timeout_sec_)
      {
        continue;
      }

      const Point2 & last_measurement = track.pos_hist.back();
      for (std::size_t ci = 0; ci < clusters.size(); ++ci) {
        if (cluster_used[ci] || !radius_matches(track, clusters[ci])) {
          continue;
        }

        const double cx = clusters[ci].centroid[0];
        const double cy = clusters[ci].centroid[1];
        const double d_last = std::hypot(
          last_measurement[0] - cx, last_measurement[1] - cy);
        if (d_last > recovery_gate) {
          continue;
        }

        // 오래된 CV ghost가 전혀 관계없는 cluster에 재연결되는 것을 막는다.
        const double d_prediction = std::hypot(track.kf.x() - cx, track.kf.y() - cy);
        if (d_prediction > 2.0 * recovery_gate) {
          continue;
        }

        recovery_pairs.push_back(Pair{d_last, ti, ci});
        ++recovery_candidates[ci];
      }
    }

    std::stable_sort(
      recovery_pairs.begin(), recovery_pairs.end(),
      [](const Pair & a, const Pair & b) {return a.d < b.d;});

    for (const auto & p : recovery_pairs) {
      // 두 active dynamic ID가 같은 cluster를 후보로 내면 안전하게 보류한다.
      if (recovery_candidates[p.ci] != 1 || track_matched[p.ti] || cluster_used[p.ci]) {
        continue;
      }
      tracks_[p.ti]->reacquire(clusters[p.ci], stamp);
      track_matched[p.ti] = 1;
      cluster_used[p.ci] = 1;
    }
  }

  // ================================================================
  // 5. unmatched Track
  //
  // ★ 여기서 삭제하지 않는다.
  //
  // KF prediction 상태 그대로 유지하면서 misses만 증가시킨다.
  //
  // ================================================================
  for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {

    if (!track_matched[ti]) {
      tracks_[ti]->mark_missed();
    }
  }


  // ================================================================
  // 6. Track lifecycle
  //
  // lifecycle을 spawn보다 먼저 처리한다. timeout이 난 바로 그 프레임의
  // cluster도 retired cache에서 같은 ID로 되살릴 수 있어야 한다.
  // ================================================================
  std::vector<TrackPtr> survivors;

  survivors.reserve(tracks_.size());


  for (auto & track : tracks_) {
    if (track->misses > track_timeout_) {

      // 완전히 버리지 않고 re-ID pool로 이동
      retire(
        *track,
        stamp);

    } else {
      survivors.push_back(track);
    }
  }


  tracks_.swap(survivors);


  // ================================================================
  // 7. retired cache timeout
  // ================================================================
  retired_.erase(
    std::remove_if(
      retired_.begin(),
      retired_.end(),
      [&](const Retired & r)
      {
        // 동적으로 확정된 상대의 identity cache만 조금 더 오래 남긴다.
        // active coast 수명(track_timeout)과 분리되어 있으므로 여기만 늘려도
        // RViz/플래너에 오래된 KF ghost를 더 오래 발행하지는 않는다.
        const double timeout = r.ever_dynamic ?
          std::max(reid_timeout_, dynamic_frenet_reid_timeout_sec_) : reid_timeout_;
        return stamp - r.t > timeout;
      }),
    retired_.end());

  // ================================================================
  // 8. Frenet dynamic Re-ID
  //
  // strict CV re-ID(.7m)는 spawn()에 그대로 남긴다. 다만 코너/FOV 이탈 뒤에는
  // 지도 좌표의 직선 예측이 실제 트랙 진행과 크게 어긋날 수 있다. 그런 경우에만
  // retired dynamic track을 마지막 실측점의 Frenet s/d로 한 번 더 대조한다.
  //
  // - 일반/정적 트랙에는 절대 적용하지 않는다.
  // - 같은 retired ID 또는 cluster에 후보가 둘 이상이면 복구하지 않는다.
  // - 진행방향, 진행량, 횡방향, 반경을 모두 검사한다.
  //
  // 즉 Euclidean gate를 넓혀 정적 장애물이 상대 ID를 탈취하는 대신, 레이스
  // 트랙 위에서 물리적으로 가능한 재등장만 좁게 허용한다.
  // ================================================================
  if (track_bounds != nullptr && dynamic_frenet_reid_timeout_sec_ > 0.0 &&
    dynamic_frenet_reid_lateral_gate_m_ > 0.0)
  {
    struct FrenetPair
    {
      double score;
      std::size_t ri;
      std::size_t ci;
    };
    std::vector<FrenetPair> frenet_pairs;
    std::vector<std::size_t> retired_candidates(retired_.size(), 0);
    std::vector<std::size_t> cluster_candidates(clusters.size(), 0);

    for (std::size_t ri = 0; ri < retired_.size(); ++ri) {
      const Retired & retired = retired_[ri];
      if (!retired.ever_dynamic || !retired.is_dynamic ||
        stamp - retired.t > dynamic_frenet_reid_timeout_sec_)
      {
        continue;
      }

      const double measurement_age = std::max(
        0.0, stamp - retired.last_measurement_stamp);
      const double expected_progress = std::hypot(retired.vx, retired.vy) * measurement_age;
      const double max_progress =
        expected_progress + dynamic_frenet_reid_progress_slack_m_;

      for (std::size_t ci = 0; ci < clusters.size(); ++ci) {
        if (cluster_used[ci] || !radius_matches(retired, clusters[ci])) {
          continue;
        }

        const std::vector<Point2> points{
          Point2{{retired.last_measurement_x, retired.last_measurement_y}},
          clusters[ci].centroid};
        std::vector<std::size_t> indices;
        std::vector<double> s;
        std::vector<double> d;
        track_bounds->project(points, indices, s, d);
        const double ds = track_bounds->normalize_s(s[1] - s[0]);
        const double dd = std::abs(d[1] - d[0]);
        if (ds < -dynamic_frenet_reid_backtrack_m_ || ds > max_progress ||
          dd > dynamic_frenet_reid_lateral_gate_m_)
        {
          continue;
        }

        // 두 제약 모두 작게 위반할수록 좋은 후보지만, 최종 복구는 아래의
        // one-to-one 조건을 만족할 때만 한다.
        const double score = std::abs(ds - expected_progress) + dd;
        frenet_pairs.push_back(FrenetPair{score, ri, ci});
        ++retired_candidates[ri];
        ++cluster_candidates[ci];
      }
    }

    std::stable_sort(
      frenet_pairs.begin(), frenet_pairs.end(),
      [](const FrenetPair & a, const FrenetPair & b) {return a.score < b.score;});

    std::vector<uint8_t> retired_used(retired_.size(), 0);
    for (const auto & pair : frenet_pairs) {
      if (retired_candidates[pair.ri] != 1 || cluster_candidates[pair.ci] != 1 ||
        retired_used[pair.ri] || cluster_used[pair.ci])
      {
        continue;
      }
      tracks_.push_back(revive(retired_[pair.ri], clusters[pair.ci], stamp));
      retired_used[pair.ri] = 1;
      cluster_used[pair.ci] = 1;
    }

    if (std::any_of(retired_used.begin(), retired_used.end(), [](uint8_t used) {return used != 0;})) {
      std::vector<Retired> remaining;
      remaining.reserve(retired_.size());
      for (std::size_t ri = 0; ri < retired_.size(); ++ri) {
        if (!retired_used[ri]) {
          remaining.push_back(retired_[ri]);
        }
      }
      retired_.swap(remaining);
    }
  }

  // ================================================================
  // 9. unmatched cluster
  //
  // 현재 프레임에 retire된 트랙까지 포함해 re-ID를 먼저 시도한다.
  // 실패했을 때만 신규 ID를 만든다.
  //
  // ── 파편 스폰 억제 (2026-08-18, run_0817_194156) ────────────────────────
  // 이미 이번 프레임에 소비된 cluster 로부터 gating_distance_ 안에 남아 있는
  // cluster 는 **같은 물체가 쪼개진 파편**으로 보고 새 ID 를 만들지 않는다.
  //
  // 근거: 한 물체의 연속 표면이 adaptive breakpoint 문턱 근처에서 프레임마다
  // 붙었다 갈라진다 (clustering.hpp 의 kSafetyFactor 주석). 갈라지면 그리디
  // 매칭은 트랙당 cluster 하나만 주므로 파편이 새 ID 로 태어나고, 다음 프레임에
  // 다시 합쳐지면 짝을 잃은 쪽이 0.80s 코스팅으로 계속 발행된다. 실측:
  //   · 신규 트랙 229개 중 19%(44개)가 기존 트랙 0.45m 안에서 태어났고
  //     그중 82%가 반경 <0.10m 파편
  //   · 발행 인스턴스 16,475개 중 21.5%가 0.45m 안에 짝을 가진 중복
  //     (쌍 간격 p50 0.23m, 큰 쪽 r 0.16 / 작은 쪽 r 0.04)
  //
  // gating_distance_ 를 그대로 쓴다 — 그보다 멀면 애초에 같은 트랙으로 매칭될
  // 수 없으니 "별개 물체"라는 판정과 기준이 하나로 유지된다. 억제해도 그 자리는
  // 이웃 트랙이 이미 점유하고 있으므로 플래너가 보는 가용공간은 줄지 않는다.
  //
  // 남은 cluster 끼리도 같은 규칙을 적용해야 **첫 검출부터 쪼개진** 물체가
  // 두 ID 로 태어나는 것을 막는다. 그래서 큰 것부터 스폰하고(본체가 ID 를
  // 갖는다) 스폰한 cluster 를 곧바로 소비 처리한다.
  // ================================================================
  std::vector<std::size_t> leftovers;
  leftovers.reserve(clusters.size());
  for (std::size_t ci = 0; ci < clusters.size(); ++ci) {
    if (!cluster_used[ci]) {
      leftovers.push_back(ci);
    }
  }
  // 점 수 내림차순. 동점은 원래 각도 순서를 유지한다(삽입 순서 계약).
  std::stable_sort(
    leftovers.begin(), leftovers.end(),
    [&clusters](std::size_t a, std::size_t b) {
      return clusters[a].points.size() > clusters[b].points.size();
    });

  for (const std::size_t ci : leftovers) {
    bool fragment = false;
    if (gating_distance_ > 0.0) {
      for (std::size_t cj = 0; cj < clusters.size(); ++cj) {
        if (cj == ci || !cluster_used[cj]) {
          continue;
        }
        const double d = std::hypot(
          clusters[ci].centroid[0] - clusters[cj].centroid[0],
          clusters[ci].centroid[1] - clusters[cj].centroid[1]);
        if (d < gating_distance_) {
          fragment = true;
          break;
        }
      }
    }
    if (fragment) {
      continue;
    }
    spawn(clusters[ci], stamp);
    cluster_used[ci] = 1;
  }

  return tracks_;
}


// ── Retire ────────────────────────────────────────────────────────────────
void Tracker::retire(
  const Track & track,
  double stamp)
{
  // retired 동안의 예측에는 현재 속도를 쓰되, 재연결 뒤 새 KF에 그 속도를
  // 그대로 주입하지는 않는다. 코너 전 접선 속도가 다시 외삽 ghost를 만들기
  // 때문이다.
  const Point2 last_measurement = track.pos_hist.empty() ?
    Point2{{track.kf.x(), track.kf.y()}} : track.pos_hist.back();
  retired_.push_back(
    Retired{
      track.id,

      track.kf.x(),
      track.kf.y(),

      track.kf.vx(),
      track.kf.vy(),

      track.radius,

      stamp,

      last_measurement[0],
      last_measurement[1],
      track.last_measurement_stamp,

      track.dynamic_streak,
      track.static_streak,
      track.confidence,

      track.ever_dynamic,
      track.is_dynamic
    });
}


// ── Spawn / Re-ID ─────────────────────────────────────────────────────────
void Tracker::spawn(
  const Cluster & cluster,
  double stamp)
{
  const double cx =
    cluster.centroid[0];

  const double cy =
    cluster.centroid[1];


  // ================================================================
  // 최근 사라진 track에 대해 re-ID
  //
  // retired 상태에서도 CV extrapolation:
  //
  // x_pred = x + vx * dt
  // y_pred = y + vy * dt
  // ================================================================
  bool found = false;

  std::size_t best_k = 0;

  double best_d =
    reid_distance_;


  for (std::size_t k = 0; k < retired_.size(); ++k) {

    const Retired & r =
      retired_[k];

    const double dt =
      stamp - r.t;


    const double px =
      r.x +
      r.vx * dt;

    const double py =
      r.y +
      r.vy * dt;


    const double d =
      std::hypot(
      px - cx,
      py - cy);

    if (d > reid_distance_) {
      continue;
    }

    if (!radius_matches(r, cluster)) {
      continue;
    }

    if (!found || d < best_d) {
      best_d = d;
      best_k = k;
      found = true;
    }
  }


  if (found) {

    const Retired r =
      retired_[best_k];

    retired_.erase(
      retired_.begin() +
      static_cast<long>(best_k));


    tracks_.push_back(revive(r, cluster, stamp));

    return;
  }


  // ================================================================
  // 완전히 새로운 obstacle
  // ================================================================
  tracks_.push_back(
    std::make_shared<Track>(
    next_id_,
    cluster,
    stamp,
    pos_hist_len_));

  ++next_id_;
}

}  // namespace perception_pkg
