#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace lidar_accelerator
{

// Same as decode_and_process but uses process_u8_to_xyzi_f32_v2 (P3 SIMD).
std::vector<float> decode_and_process_v2(
  const uint8_t * compressed,
  std::size_t compressed_len,
  float res,
  const float origin[3],
  float intense_limiter,
  bool deduplicate,
  int downsample_step,
  int max_points,
  std::size_t * out_points);

}  // namespace lidar_accelerator
