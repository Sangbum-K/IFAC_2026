#ifndef PLANNING_PKG__STEER_GRIP_MODEL_HPP_
#define PLANNING_PKG__STEER_GRIP_MODEL_HPP_

// ── 속도별 조향 능력 모델 (2026-08-23) ────────────────────────────────────
//
// 플래닝은 지금까지 차의 선회 능력을 kinematic 자전거모델 하나로 봤다:
//     kappa_max = tan(delta_max) / wheelbase          (속도 무관, 1.13 1/m)
// 그런데 control 이 실제로 쓰는 조향 LUT(ETH Pacejka 표, 실차 11 백 피팅)를
// 뒤집어 보면 최대 조향 0.3578 rad 에서 낼 수 있는 곡률은 속도의 함수다:
//
//     v [m/s]   a_lat_max [m/s^2]   kappa=a/v^2   R [m]    자전거모델 대비
//       2.0           3.72             0.93       1.07        82 %
//       3.0           6.84             0.76       1.32        67 %
//       4.0          10.21             0.64       1.57        56 %
//       4.5           9.54             0.47       2.12        42 %
//       5.0           8.44             0.34       2.96        30 %
//       5.5           6.46             0.21       4.68        19 %
//       6.0           6.02             0.17       5.98        15 %
//
// (4 m/s 위에서 a_lat 가 도로 줄어드는 것은 표 자체의 그립 천장이다 —
//  거기서는 조향을 더 넣어도 앞이 밀린다.)
//
// 근거 (run_0823_043254 / run_0823_044748 재구성):
//   · 레이스라인 344 점 중 **167 점(48.5 %)** 이 계획 속도에서 조향으로 낼 수
//     없는 곡률을 요구한다 (요구/가능 최대 2.02 배). 속도 0.90 배면 4.1 %,
//     0.80 배면 0 %.
//   · 그 결과 s=0~4.5 코너에서는 **장애물이 하나도 없을 때도** |d| p90 0.63 m,
//     최대 0.81 m 로 밀려난다 (다른 구간은 p90 0.13~0.31 m).
//   · 두 백의 정적 충돌 4 건이 전부 그 코너(s 0.8~1.6)에서 났고, 그중 3 건은
//     **회피할 쪽이 첫 관측부터 열려 있었다** (좌측 개방률 78~92 %).
//     즉 경로가 없어서가 아니라 그린 경로를 차가 못 따라가서 난 사고다.
//
// 이 모델은 그 표를 구간선형으로 담아, 계획이 "차가 실제로 낼 수 있는 것"만
// 약속하게 한다. 표를 비우면(빈 배열) enabled()==false 가 되고 모든 질의가
// 종전 자전거모델 값을 그대로 돌려준다 — 즉 **되돌리기는 yaml 한 줄이다.**

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace planning_pkg {

class SteerGripModel {
 public:
  // 표를 싣는다. 유효하지 않으면 false 를 돌리고 모델은 꺼진 채로 남는다
  // (호출부는 파라미터 검증 실패로 처리할 것).
  // speed_brk: 단조증가 양수 속도 [m/s]
  // lat_acc  : 그 속도에서 최대 조향으로 낼 수 있는 횡가속 [m/s^2]
  // margin   : 표에 곱하는 안전계수 (0 < margin <= 1.0). 1.0 = 표 그대로.
  bool Set(const std::vector<double> &speed_brk,
           const std::vector<double> &lat_acc, double margin) {
    speed_.clear();
    acc_.clear();
    margin_ = 1.0;
    if (speed_brk.empty() && lat_acc.empty()) {
      return true;                       // 의도적 비활성 — 종전 동작
    }
    if (speed_brk.size() != lat_acc.size() || speed_brk.size() < 2) {
      return false;
    }
    if (!(margin > 0.0) || margin > 1.0 || !std::isfinite(margin)) {
      return false;
    }
    for (std::size_t i = 0; i < speed_brk.size(); ++i) {
      if (!std::isfinite(speed_brk[i]) || !std::isfinite(lat_acc[i]) ||
          speed_brk[i] <= 0.0 || lat_acc[i] <= 0.0) {
        return false;
      }
      if (i > 0 && !(speed_brk[i] > speed_brk[i - 1])) {
        return false;
      }
    }
    speed_ = speed_brk;
    acc_ = lat_acc;
    margin_ = margin;
    return true;
  }

  bool enabled() const { return speed_.size() >= 2; }

  // 그 속도에서 조향으로 낼 수 있는 횡가속 [m/s^2].
  // 표 범위 밖은 양끝 값으로 클램프한다 — 저속(표 아래)에서는 어차피
  // 자전거모델 기하 한계가 먼저 물리고, 고속(표 위)에서는 마지막 값이
  // 가장 보수적이다.
  double LatAccAt(double speed) const {
    if (!enabled()) {
      return std::numeric_limits<double>::infinity();
    }
    const double v = std::abs(speed);
    if (!std::isfinite(v) || v <= speed_.front()) {
      return margin_ * acc_.front();
    }
    if (v >= speed_.back()) {
      return margin_ * acc_.back();
    }
    const std::size_t hi =
        static_cast<std::size_t>(
            std::upper_bound(speed_.begin(), speed_.end(), v) - speed_.begin());
    const std::size_t lo = hi - 1;
    const double w = (v - speed_[lo]) / (speed_[hi] - speed_[lo]);
    return margin_ * (acc_[lo] + w * (acc_[hi] - acc_[lo]));
  }

  // 그 속도에서 실제로 낼 수 있는 곡률 [1/m].
  // kinematic_kappa_max = 기존 tan(delta_max)/wheelbase (필요하면 배율 포함).
  // 둘 중 **작은 쪽**이 진짜 한계다.
  double KappaAt(double speed, double kinematic_kappa_max) const {
    if (!enabled()) {
      return kinematic_kappa_max;
    }
    const double v = std::abs(speed);
    if (!std::isfinite(v) || v < 1e-3) {
      return kinematic_kappa_max;
    }
    return std::min(kinematic_kappa_max, LatAccAt(v) / (v * v));
  }

  // 곡률 kappa 를 실제로 낼 수 있는 최대 속도 [m/s].
  //   · kappa 가 기하 한계 자체를 넘으면 0 (감속으로 살릴 수 없다)
  //   · 모델이 꺼져 있거나 kappa 가 0 이면 +inf (속도 제약 없음)
  // ⚠️ 이분법을 쓰지 않는다 — a_lat(v) 가 4 m/s 부근에서 꺾이므로
  //    kappa*v^2 - a_lat(v) 가 단조라는 보장이 없다. 낮은 속도에서 위로
  //    훑어 **첫 위반 직전**을 돌려준다 (= 그 아래 전 구간이 성립).
  double SpeedCapForKappa(double kappa, double kinematic_kappa_max) const {
    const double k = std::abs(kappa);
    if (!std::isfinite(k) || k < 1e-4) {
      return std::numeric_limits<double>::infinity();
    }
    if (k > kinematic_kappa_max) {
      return 0.0;                        // 조향각 자체가 모자라다
    }
    if (!enabled()) {
      return std::numeric_limits<double>::infinity();
    }
    const double v_hi = speed_.back();
    const double step = 0.02;
    double best = 0.0;
    for (double v = step; v <= v_hi + 1e-9; v += step) {
      if (k * v * v > LatAccAt(v)) {
        break;
      }
      best = v;
    }
    // 표 끝까지 성립하면 그 위는 제약이 없다고 본다 (표 밖은 클램프라
    // a_lat 가 상수이므로 kappa*v^2 가 언젠가 반드시 이긴다 — 그 지점을
    // 닫힌 형태로 마저 구한다).
    if (best >= v_hi - 1e-9) {
      const double a_end = margin_ * acc_.back();
      return std::sqrt(a_end / k);
    }
    return best;
  }

 private:
  std::vector<double> speed_;
  std::vector<double> acc_;
  double margin_{1.0};
};

}  // namespace planning_pkg

#endif  // PLANNING_PKG__STEER_GRIP_MODEL_HPP_
