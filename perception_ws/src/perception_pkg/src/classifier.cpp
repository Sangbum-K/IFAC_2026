#include "perception_pkg/classifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace perception_pkg
{
namespace
{

/// 측정 위치 이력의 공간 퍼짐 [m] = x/y 표준편차 중 큰 값.
/// 표본 부족(3점 미만)이면 false를 돌려준다.
bool pos_spread(const std::deque<Point2> & pos_hist, double & spread_out)
{
  if (pos_hist.size() < 3) {
    return false;
  }
  const double n = static_cast<double>(pos_hist.size());
  double mx = 0.0;
  double my = 0.0;
  for (const auto & p : pos_hist) {
    mx += p[0];
    my += p[1];
  }
  mx /= n;
  my /= n;
  double vx = 0.0;
  double vy = 0.0;
  for (const auto & p : pos_hist) {
    vx += (p[0] - mx) * (p[0] - mx);
    vy += (p[1] - my) * (p[1] - my);
  }
  // numpy std 기본값과 동일한 모분산(ddof=0)
  spread_out = std::max(std::sqrt(vx / n), std::sqrt(vy / n));
  return true;
}

}  // namespace

bool classify_track(
  Track & track, bool has_measurement,
  double v_thresh, int n_confirm, int n_release,
  double static_pos_std, double dynamic_pos_std, int dynamic_release)
{
  // KF coast는 기존 속도·위치이력만으로 만들어진 예측이다. 이를 한 프레임의
  // 관측 증거로 세면 실제 3회 관측 확인이 아니라 miss 프레임만으로 확정/해제된다.
  if (!has_measurement) {
    return track.is_dynamic;
  }

  const double speed = track.kf.speed();   // 칼만필터가 추정한 현재 속도 크기
  double spread = 0.0;
  const bool has_spread = pos_spread(track.pos_hist, spread);

  // 프레임 투표:
  //  moving = 속도가 임계 이상 '또는' 위치 분산이 큼 (느리지만 꾸준히 이동)
  //  still  = 저속 '그리고' 위치 분산이 작음 (표본 부족이면 분산 조건은 통과로 봄)
  const bool moving = (speed >= v_thresh) || (has_spread && spread >= dynamic_pos_std);
  const bool still = (speed < v_thresh) && (!has_spread || spread < static_pos_std);

  if (moving) {               // 움직임 근거 프레임
    ++track.dynamic_streak;
    track.static_streak = 0;
  } else if (still) {         // 정지 근거 프레임
    ++track.static_streak;
    track.dynamic_streak = 0;
  }
  // 모호(저속인데 분산이 중간)한 프레임은 두 streak 모두 유지 -> 깜빡임 방지

  // 정체성: 한 번 상대(동적)로 확정된 트랙은 정차/코너 감속에 쉽게 안 뒤집히게
  // release를 크게 잡는다. 일반 트랙(노이즈 클러스터 등)은 기존 n_release.
  const int release = track.ever_dynamic ? dynamic_release : n_release;

  if (!track.is_dynamic && track.dynamic_streak >= n_confirm) {
    track.is_dynamic = true;      // 동적으로 확정
    track.ever_dynamic = true;    // 상대 정체성 각인 (이후 잘 안 풀림)
  } else if (track.is_dynamic && track.static_streak >= release) {
    track.is_dynamic = false;     // 충분히 오래 정지 -> 정적으로 복귀
  }

  // confidence: 확정 조건 대비 진행률 (0~1). 로깅/디버깅 및 msg.confidence용
  if (track.is_dynamic) {
    track.confidence = std::min(
      1.0, static_cast<double>(track.dynamic_streak) / static_cast<double>(std::max(n_confirm, 1)));
  } else if (track.hits > 1) {    // 최소 2번 이상 관측된 트랙만 신뢰도 부여
    track.confidence = std::min(
      1.0, static_cast<double>(track.static_streak) / static_cast<double>(std::max(release, 1)));
  } else {
    track.confidence = 0.0;
  }

  return track.is_dynamic;
}

}  // namespace perception_pkg
