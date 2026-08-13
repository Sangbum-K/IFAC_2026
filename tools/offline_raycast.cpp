// Copyright 2026 IFAC F1TENTH Team
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {

float cast_one(
    const int8_t* map, int width, int height, float resolution,
    float origin_x, float origin_y, float origin_yaw,
    float world_x, float world_y, float world_yaw, float ray_angle,
    float maximum_range) {
  const float origin_cos = std::cos(origin_yaw);
  const float origin_sin = std::sin(origin_yaw);
  const float shifted_x = world_x - origin_x;
  const float shifted_y = world_y - origin_y;
  const float local_x = origin_cos * shifted_x + origin_sin * shifted_y;
  const float local_y = -origin_sin * shifted_x + origin_cos * shifted_y;

  const float world_angle = world_yaw + ray_angle;
  const float world_dx = std::cos(world_angle);
  const float world_dy = std::sin(world_angle);
  const float local_dx = origin_cos * world_dx + origin_sin * world_dy;
  const float local_dy = -origin_sin * world_dx + origin_cos * world_dy;

  int cell_x = static_cast<int>(std::floor(local_x / resolution));
  int cell_y = static_cast<int>(std::floor(local_y / resolution));
  if (cell_x < 0 || cell_x >= width || cell_y < 0 || cell_y >= height) {
    return maximum_range;
  }
  if (map[cell_y * width + cell_x] >= 50) {
    return 0.0F;
  }

  const float infinity = std::numeric_limits<float>::infinity();
  int step_x = 0;
  int step_y = 0;
  float next_x = infinity;
  float next_y = infinity;
  float delta_x = infinity;
  float delta_y = infinity;

  if (std::abs(local_dx) > 1.0e-8F) {
    step_x = local_dx > 0.0F ? 1 : -1;
    const float boundary_x = (
        step_x > 0 ? (cell_x + 1) * resolution : cell_x * resolution);
    next_x = (boundary_x - local_x) / local_dx;
    delta_x = resolution / std::abs(local_dx);
  }
  if (std::abs(local_dy) > 1.0e-8F) {
    step_y = local_dy > 0.0F ? 1 : -1;
    const float boundary_y = (
        step_y > 0 ? (cell_y + 1) * resolution : cell_y * resolution);
    next_y = (boundary_y - local_y) / local_dy;
    delta_y = resolution / std::abs(local_dy);
  }

  float distance = 0.0F;
  while (distance <= maximum_range) {
    if (next_x < next_y) {
      cell_x += step_x;
      distance = next_x;
      next_x += delta_x;
    } else {
      cell_y += step_y;
      distance = next_y;
      next_y += delta_y;
    }
    if (distance > maximum_range) {
      break;
    }
    if (cell_x < 0 || cell_x >= width || cell_y < 0 || cell_y >= height) {
      return maximum_range;
    }
    if (map[cell_y * width + cell_x] >= 50) {
      return std::max(0.0F, distance);
    }
  }
  return maximum_range;
}

}  // namespace

extern "C" void raycast_many(
    const int8_t* map, int width, int height, float resolution,
    float origin_x, float origin_y, float origin_yaw,
    const float* poses, int pose_count,
    const float* angles, int angle_count,
    float maximum_range, float* ranges) {
#pragma omp parallel for schedule(static)
  for (int pose_index = 0; pose_index < pose_count; ++pose_index) {
    const float* pose = poses + pose_index * 3;
    float* output = ranges + pose_index * angle_count;
    for (int angle_index = 0; angle_index < angle_count; ++angle_index) {
      output[angle_index] = cast_one(
          map, width, height, resolution,
          origin_x, origin_y, origin_yaw,
          pose[0], pose[1], pose[2], angles[angle_index], maximum_range);
    }
  }
}

// Pairwise variant used by scan-deskew analysis: pose[i] and angle[i] describe
// one beam acquired at one timestamp.  raycast_many() intentionally keeps its
// Cartesian-product API for the particle-filter replay path.
extern "C" void raycast_pairs(
    const int8_t* map, int width, int height, float resolution,
    float origin_x, float origin_y, float origin_yaw,
    const float* poses, const float* angles, int count,
    float maximum_range, float* ranges) {
#pragma omp parallel for schedule(static)
  for (int index = 0; index < count; ++index) {
    const float* pose = poses + index * 3;
    ranges[index] = cast_one(
        map, width, height, resolution,
        origin_x, origin_y, origin_yaw,
        pose[0], pose[1], pose[2], angles[index], maximum_range);
  }
}
