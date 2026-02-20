// processing_v2.hpp -- V2: SIMD-friendly two-phase decode.
// Same API as processing.hpp; kept separate for A/B comparison.
//
// Optimisation over V1:
//   - Two separate loops replace the single loop with conditional push_back.
//   - Phase 1 (u8→f32 SoA conversion) is unconditional → auto-vectorised by
//     GCC/Clang with -O3 -march=armv8.2-a+simd (NEON on Jetson).
//   - Phase 2 (intensity filter + gather) reads contiguous arrays, reducing
//     branch mispredictions and improving cache prefetch performance.
//   - __restrict__ hints eliminate pointer aliasing pessimism in the compiler.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace lidar_accelerator
{

std::vector<float> process_u8_to_xyzi_f32_v2(
  const uint8_t * __restrict__ positions_u8,
  std::size_t positions_len,
  const uint8_t * __restrict__ uvs_u8,
  std::size_t uvs_len,
  float res,
  const float origin[3],
  float intense_limiter,
  bool deduplicate,
  int downsample_step,
  int max_points,
  std::size_t * out_points);

}  // namespace lidar_accelerator
