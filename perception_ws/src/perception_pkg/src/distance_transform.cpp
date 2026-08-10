#include "perception_pkg/distance_transform.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace perception_pkg
{
namespace
{

/// 거리 기준점이 하나도 없는 축을 표현하는 값. inf를 그대로 쓰면 이후
/// 곱셈/비교에서 NaN이 새기 쉬워 큰 유한값으로 둔다.
constexpr double kBig = 1e18;

/// Felzenszwalb & Huttenlocher 1D 제곱거리 변환.
/// f: 입력 비용, d: 출력. v/z는 호출부가 재사용하는 작업버퍼.
void dt1d(
  const std::vector<double> & f, std::vector<double> & d,
  std::vector<int> & v, std::vector<double> & z, int n)
{
  int k = 0;
  v[0] = 0;
  z[0] = -kBig;
  z[1] = kBig;
  for (int q = 1; q < n; ++q) {
    const double fq = f[static_cast<std::size_t>(q)] + static_cast<double>(q) * q;
    auto intersection = [&](int idx) {
        const int vk = v[static_cast<std::size_t>(idx)];
        const double fv = f[static_cast<std::size_t>(vk)] + static_cast<double>(vk) * vk;
        return (fq - fv) / static_cast<double>(2 * q - 2 * vk);
      };
    double s = intersection(k);
    // z[0] = -kBig이고 s는 항상 그보다 크므로 k=0에서 자연히 멈춘다.
    // k > 0 가드는 부동소수 예외 상황에서도 인덱스가 음수로 새지 않게 하는 안전장치.
    while (k > 0 && s <= z[static_cast<std::size_t>(k)]) {
      --k;
      s = intersection(k);
    }
    ++k;
    v[static_cast<std::size_t>(k)] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k) + 1] = kBig;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<std::size_t>(k) + 1] < static_cast<double>(q)) {
      ++k;
    }
    const int vk = v[static_cast<std::size_t>(k)];
    const double dq = static_cast<double>(q - vk);
    d[static_cast<std::size_t>(q)] = dq * dq + f[static_cast<std::size_t>(vk)];
  }
}

}  // namespace

std::vector<float> euclidean_distance_transform(
  const std::vector<uint8_t> & seed, int width, int height, double scale)
{
  std::vector<float> out;
  if (width <= 0 || height <= 0 ||
    seed.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
  {
    return out;
  }

  const std::size_t w = static_cast<std::size_t>(width);
  const std::size_t h = static_cast<std::size_t>(height);

  // 제곱거리 누적 버퍼: seed는 0, 나머지는 무한대에서 시작한다.
  std::vector<double> sq(seed.size());
  for (std::size_t i = 0; i < seed.size(); ++i) {
    sq[i] = seed[i] ? 0.0 : kBig;
  }

  const int max_dim = std::max(width, height);
  std::vector<double> f(static_cast<std::size_t>(max_dim));
  std::vector<double> d(static_cast<std::size_t>(max_dim));
  std::vector<int> v(static_cast<std::size_t>(max_dim));
  std::vector<double> z(static_cast<std::size_t>(max_dim) + 1);

  // ① 열 방향
  for (std::size_t c = 0; c < w; ++c) {
    for (std::size_t r = 0; r < h; ++r) {
      f[r] = sq[r * w + c];
    }
    dt1d(f, d, v, z, height);
    for (std::size_t r = 0; r < h; ++r) {
      sq[r * w + c] = d[r];
    }
  }
  // ② 행 방향
  for (std::size_t r = 0; r < h; ++r) {
    for (std::size_t c = 0; c < w; ++c) {
      f[c] = sq[r * w + c];
    }
    dt1d(f, d, v, z, width);
    for (std::size_t c = 0; c < w; ++c) {
      sq[r * w + c] = d[c];
    }
  }

  // sqrt와 scale 곱을 double로 이어서 계산하고 마지막에 한 번만 float으로 접는다.
  out.resize(seed.size());
  for (std::size_t i = 0; i < seed.size(); ++i) {
    const double value = sq[i];
    out[i] = (value >= kBig) ?
      std::numeric_limits<float>::max() :
      static_cast<float>(std::sqrt(value) * scale);
  }
  return out;
}

}  // namespace perception_pkg
