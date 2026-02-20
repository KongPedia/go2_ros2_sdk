// processing_v2.cpp -- V2: two-phase SoA approach for SIMD auto-vectorisation.
//
// Architecture: ARM64 (Cortex-A78AE on Jetson Orin) with 128-bit NEON.
// Compiled with -O3 -march=armv8.2-a+simd -ftree-vectorize by CMake target.
//
// V1 single-pass loop:
//   for i in range(n): convert u8→f32, branch on intensity, push_back  ← not SIMD
//
// V2 two-phase loop:
//   Phase 1: for i in range(n): xs[i]=..., ys[i]=..., ...  ← unconditional, SIMD
//   Phase 2: for i in range(n): if intensity[i]>lim: out.push_back  ← filter pass

// Compiler hints for maximum SIMD on this translation unit
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3,unroll-loops,tree-vectorize")
#endif
#if defined(__aarch64__)
#pragma GCC target("arch=armv8.2-a+simd+fp16")
#endif

#include "lidar_accelerator/processing_v2.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace lidar_accelerator
{

namespace
{

inline bool row_less_lex_v2(
  const std::array<float, 4> & a,
  const std::array<float, 4> & b)
{
  if (a[0] != b[0]) {return a[0] < b[0];}
  if (a[1] != b[1]) {return a[1] < b[1];}
  if (a[2] != b[2]) {return a[2] < b[2];}
  return a[3] < b[3];
}

}  // namespace

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
  std::size_t * out_points)
{
  if (out_points) {*out_points = 0;}
  if (!positions_u8 || !uvs_u8) {return {};}
  if (positions_len % 3 != 0 || uvs_len % 2 != 0) {return {};}

  const std::size_t n_pos = positions_len / 3;
  const std::size_t n_uv = uvs_len / 2;
  const std::size_t n = (n_pos < n_uv) ? n_pos : n_uv;
  if (n == 0) {return {};}

  if (downsample_step <= 0) {downsample_step = 1;}

  const float ox = origin[0], oy = origin[1], oz = origin[2];

  // ------------------------------------------------------------------
  // Phase 1: unconditional u8→f32 conversion (SoA, auto-SIMD on ARM64)
  // Separate contiguous loops allow NEON to process 4 floats per cycle.
  // ------------------------------------------------------------------
  std::vector<float> xs(n), ys(n), zs(n), intensity(n);

  // Position X: positions_u8[i*3+0]
  for (std::size_t i = 0; i < n; ++i) {
    xs[i] = static_cast<float>(positions_u8[i * 3 + 0]) * res + ox;
  }
  // Position Y
  for (std::size_t i = 0; i < n; ++i) {
    ys[i] = static_cast<float>(positions_u8[i * 3 + 1]) * res + oy;
  }
  // Position Z
  for (std::size_t i = 0; i < n; ++i) {
    zs[i] = static_cast<float>(positions_u8[i * 3 + 2]) * res + oz;
  }
  // Intensity = min(u, v) -- branch-free via std::min
  for (std::size_t i = 0; i < n; ++i) {
    const float u = static_cast<float>(uvs_u8[i * 2 + 0]);
    const float v = static_cast<float>(uvs_u8[i * 2 + 1]);
    intensity[i] = u < v ? u : v;
  }

  // ------------------------------------------------------------------
  // Phase 2: intensity threshold filter + gather into AoS output
  // ------------------------------------------------------------------
  std::vector<std::array<float, 4>> filtered;
  filtered.reserve(n);

  for (std::size_t i = 0; i < n; ++i) {
    if (intensity[i] > intense_limiter) {
      filtered.push_back({xs[i], ys[i], zs[i], intensity[i]});
    }
  }

  if (filtered.empty()) {return {};}

  // Downsample (same semantics as V1)
  if (downsample_step > 1) {
    std::vector<std::array<float, 4>> down;
    down.reserve(
      (filtered.size() + static_cast<std::size_t>(downsample_step) - 1) /
      static_cast<std::size_t>(downsample_step));
    for (std::size_t k = 0; k < filtered.size(); k += static_cast<std::size_t>(downsample_step)) {
      down.push_back(filtered[k]);
    }
    filtered.swap(down);
  }

  // Max points (same semantics as V1 -- np.linspace)
  if (max_points > 0 && static_cast<std::size_t>(max_points) < filtered.size()) {
    std::vector<std::array<float, 4>> limited;
    limited.reserve(static_cast<std::size_t>(max_points));
    const std::size_t total = filtered.size();
    const std::size_t m = static_cast<std::size_t>(max_points);
    if (m == 1) {
      limited.push_back(filtered.front());
    } else {
      for (std::size_t k = 0; k < m; ++k) {
        const double t = static_cast<double>(k) / static_cast<double>(m - 1);
        const std::size_t idx = static_cast<std::size_t>(t * static_cast<double>(total - 1));
        limited.push_back(filtered[idx]);
      }
    }
    filtered.swap(limited);
  }

  // Deduplicate (same semantics as V1)
  if (deduplicate) {
    std::sort(filtered.begin(), filtered.end(), row_less_lex_v2);
    auto last = std::unique(filtered.begin(), filtered.end());
    filtered.erase(last, filtered.end());
  }

  std::vector<float> out(filtered.size() * 4);
  for (std::size_t i = 0; i < filtered.size(); ++i) {
    out[i * 4 + 0] = filtered[i][0];
    out[i * 4 + 1] = filtered[i][1];
    out[i * 4 + 2] = filtered[i][2];
    out[i * 4 + 3] = filtered[i][3];
  }

  if (out_points) {*out_points = filtered.size();}
  return out;
}

}  // namespace lidar_accelerator
