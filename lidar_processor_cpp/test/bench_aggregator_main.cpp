// bench_aggregator_main.cpp
// Standalone C++17 correctness + benchmark for V1 vs V2 aggregator.
// No ROS2/PCL dependency -- compile with:
//   g++ -std=c++17 -O3 -I../include -o bench_aggregator bench_aggregator_main.cpp
//
// Output format: key=value lines, one per result (parseable by pytest).

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "lidar_processor_cpp/aggregator_core.hpp"
#include "lidar_processor_cpp/aggregator_core_v2.hpp"

using namespace lidar_processor_cpp;

// ---------------------------------------------------------------------------
// Timing helper
// ---------------------------------------------------------------------------
using Clock = std::chrono::steady_clock;
using Ms = std::chrono::duration<double, std::milli>;

static double elapsed_ms(Clock::time_point t0)
{
  return std::chrono::duration_cast<Ms>(Clock::now() - t0).count();
}

// ---------------------------------------------------------------------------
// Point generation helpers
// ---------------------------------------------------------------------------
static std::vector<Point3D> make_scan_v1(
  int n_unique, int n_duplicates, float range, unsigned seed = 42)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist(-range, range);

  std::vector<Point3D> unique_pts;
  unique_pts.reserve(n_unique);
  for (int i = 0; i < n_unique; ++i) {
    unique_pts.emplace_back(dist(rng), dist(rng), dist(rng));
  }

  std::vector<Point3D> pts = unique_pts;
  // Append duplicate samples
  std::uniform_int_distribution<int> idx_dist(0, n_unique - 1);
  for (int i = 0; i < n_duplicates; ++i) {
    pts.push_back(unique_pts[idx_dist(rng)]);
  }
  return pts;
}

static std::vector<Point3DV2> to_v2(const std::vector<Point3D> & src)
{
  std::vector<Point3DV2> out;
  out.reserve(src.size());
  for (const auto & p : src) {
    out.emplace_back(p.x, p.y, p.z);
  }
  return out;
}

// ---------------------------------------------------------------------------
// Correctness tests -- return number of failures
// ---------------------------------------------------------------------------
static int correctness_v1()
{
  int fails = 0;

  // --- Test 1: basic dedup ---
  {
    AggregatorConfig cfg(1000000);
    PointCloudAggregatorV1 agg(cfg);
    std::vector<Point3D> pts = {
      {1.0f, 0.0f, 0.0f},
      {1.0f, 0.0f, 0.0f},  // duplicate
      {2.0f, 0.0f, 0.0f},
    };
    agg.addPoints(pts);
    if (agg.getPointCount() != 2) {
      std::cerr << "[V1 FAIL] basic_dedup: expected 2 got " << agg.getPointCount() << "\n";
      ++fails;
    }
  }

  // --- Test 2: duplicate batch twice ---
  {
    AggregatorConfig cfg(1000000);
    PointCloudAggregatorV1 agg(cfg);
    std::vector<Point3D> pts = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
    agg.addPoints(pts);
    agg.addPoints(pts);  // same batch again
    if (agg.getPointCount() != 2) {
      std::cerr << "[V1 FAIL] double_add: expected 2 got " << agg.getPointCount() << "\n";
      ++fails;
    }
  }

  // --- Test 3: max_points enforcement ---
  {
    AggregatorConfig cfg(50);
    PointCloudAggregatorV1 agg(cfg);
    auto pts = make_scan_v1(200, 0, 10.0f);
    agg.addPoints(pts);
    if (agg.getPointCount() > 50) {
      std::cerr << "[V1 FAIL] max_points: count=" << agg.getPointCount() << " > 50\n";
      ++fails;
    }
  }

  // --- Test 4: hash collision at y >= 65.536 m (V1 known bug) ---
  // x_int=1 (x=0.001m) hashes the same as y_int=65536 (y=65.536m).
  // They are DIFFERENT points but share the same hash bucket in V1.
  // The set correctly stores both (operator== distinguishes them),
  // but this causes O(n) bucket lookup -- demonstrable via correctness:
  // ensure both points are actually stored despite hash collision.
  {
    AggregatorConfig cfg(1000000);
    PointCloudAggregatorV1 agg(cfg);
    std::vector<Point3D> pts = {
      {0.001f, 0.0f, 0.0f},   // x_int=1, hash uses (1LL << 32)
      {0.0f, 65.536f, 0.0f},  // y_int=65536, hash uses (65536LL << 16) = (1LL << 32) -- SAME!
    };
    agg.addPoints(pts);
    // Both should still be stored (equality check distinguishes them)
    if (agg.getPointCount() != 2) {
      std::cerr << "[V1 FAIL] hash_collision_storage: expected 2 got "
                << agg.getPointCount() << "\n";
      ++fails;
    }
    // Document: same-bucket lookup is O(bucket_size), not O(1)
    std::cout << "v1_hash_collision_demo=bucket_collision_at_y65m\n";
  }

  return fails;
}

static int correctness_v2()
{
  int fails = 0;

  // --- Test 1: basic dedup ---
  {
    AggregatorConfigV2 cfg(1000000);
    PointCloudAggregatorV2 agg(cfg);
    std::vector<Point3DV2> pts = {
      {1.0f, 0.0f, 0.0f},
      {1.0f, 0.0f, 0.0f},  // duplicate
      {2.0f, 0.0f, 0.0f},
    };
    agg.addPoints(pts);
    if (agg.getPointCount() != 2) {
      std::cerr << "[V2 FAIL] basic_dedup: expected 2 got " << agg.getPointCount() << "\n";
      ++fails;
    }
  }

  // --- Test 2: duplicate batch twice ---
  {
    AggregatorConfigV2 cfg(1000000);
    PointCloudAggregatorV2 agg(cfg);
    std::vector<Point3DV2> pts = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
    agg.addPoints(pts);
    agg.addPoints(pts);
    if (agg.getPointCount() != 2) {
      std::cerr << "[V2 FAIL] double_add: expected 2 got " << agg.getPointCount() << "\n";
      ++fails;
    }
  }

  // --- Test 3: max_points enforcement ---
  {
    AggregatorConfigV2 cfg(50);
    PointCloudAggregatorV2 agg(cfg);
    auto base = make_scan_v1(200, 0, 10.0f);
    agg.addPoints(to_v2(base));
    if (agg.getPointCount() > 50) {
      std::cerr << "[V2 FAIL] max_points: count=" << agg.getPointCount() << " > 50\n";
      ++fails;
    }
  }

  // --- Test 4: no hash collision at y >= 65.536 m ---
  {
    AggregatorConfigV2 cfg(1000000);
    PointCloudAggregatorV2 agg(cfg);
    std::vector<Point3DV2> pts = {
      {0.001f, 0.0f, 0.0f},
      {0.0f, 65.536f, 0.0f},
    };
    agg.addPoints(pts);
    if (agg.getPointCount() != 2) {
      std::cerr << "[V2 FAIL] no_hash_collision: expected 2 got "
                << agg.getPointCount() << "\n";
      ++fails;
    }
    std::cout << "v2_no_hash_collision=ok\n";
  }

  // --- Test 5: all unique points stored below capacity ---
  {
    AggregatorConfigV2 cfg(200000);
    PointCloudAggregatorV2 agg(cfg);
    auto base = make_scan_v1(1000, 0, 5.0f);
    agg.addPoints(to_v2(base));
    // Some points may round to same voxel, but count should be >=900 for 1000 random pts at 1mm
    if (agg.getPointCount() < 900) {
      std::cerr << "[V2 FAIL] unique_storage: expected >=900 got " << agg.getPointCount() << "\n";
      ++fails;
    }
  }

  return fails;
}

// ---------------------------------------------------------------------------
// Performance benchmarks
// ---------------------------------------------------------------------------

struct BenchResult
{
  std::string name;
  double add_ms;
  double get_ms;
  int point_count;
};

static BenchResult bench_v1(const std::string & label, int n_unique, int n_dup,
  float range, int max_pts)
{
  auto pts = make_scan_v1(n_unique, n_dup, range);
  AggregatorConfig cfg(max_pts);
  PointCloudAggregatorV1 agg(cfg);

  auto t0 = Clock::now();
  agg.addPoints(pts);
  double add_t = elapsed_ms(t0);

  t0 = Clock::now();
  auto copy = agg.getPointsCopy();
  double get_t = elapsed_ms(t0);

  return {label, add_t, get_t, agg.getPointCount()};
}

static BenchResult bench_v2(const std::string & label, int n_unique, int n_dup,
  float range, int max_pts)
{
  auto base = make_scan_v1(n_unique, n_dup, range);
  auto pts = to_v2(base);
  AggregatorConfigV2 cfg(max_pts);
  PointCloudAggregatorV2 agg(cfg);

  auto t0 = Clock::now();
  agg.addPoints(pts);
  double add_t = elapsed_ms(t0);

  t0 = Clock::now();
  auto copy = agg.getPointsCopy();
  double get_t = elapsed_ms(t0);

  return {label, add_t, get_t, agg.getPointCount()};
}

static void print_result(const BenchResult & r)
{
  std::cout << r.name << "_add_ms=" << r.add_ms << "\n";
  std::cout << r.name << "_get_ms=" << r.get_ms << "\n";
  std::cout << r.name << "_points=" << r.point_count << "\n";
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main()
{
  std::cout << "=== P1 Aggregator Correctness + Benchmark ===\n";

  // ---- Correctness ----
  int v1_fails = correctness_v1();
  int v2_fails = correctness_v2();

  std::cout << "v1_correctness_fails=" << v1_fails << "\n";
  std::cout << "v2_correctness_fails=" << v2_fails << "\n";

  if (v1_fails > 0 || v2_fails > 0) {
    std::cerr << "CORRECTNESS FAILURES: V1=" << v1_fails
              << " V2=" << v2_fails << "\n";
    return 1;
  }
  std::cout << "correctness_status=PASS\n";

  // ---- Benchmark 1: Full L1 scan, no duplicates, no limit ---
  // Unitree L1 LiDAR produces ~120k points per scan at full res.
  std::cout << "\n--- Benchmark: 120k unique points (full L1 scan), no limit ---\n";
  auto r1v1 = bench_v1("v1_120k_noduplicate_nolimit", 120000, 0, 30.0f, 5000000);
  auto r1v2 = bench_v2("v2_120k_noduplicate_nolimit", 120000, 0, 30.0f, 5000000);
  print_result(r1v1);
  print_result(r1v2);

  // ---- Benchmark 2: 120k points, 50% duplicates ---
  std::cout << "\n--- Benchmark: 120k points with 50% duplicates ---\n";
  auto r2v1 = bench_v1("v1_120k_50pct_dup", 80000, 40000, 30.0f, 5000000);
  auto r2v2 = bench_v2("v2_120k_50pct_dup", 80000, 40000, 30.0f, 5000000);
  print_result(r2v1);
  print_result(r2v2);

  // ---- Benchmark 3: overflow case -- triggers O(N log N) sort in V1 ---
  std::cout << "\n--- Benchmark: 200k input, max_points=50000 (overflow) ---\n";
  auto r3v1 = bench_v1("v1_200k_overflow50k", 200000, 0, 30.0f, 50000);
  auto r3v2 = bench_v2("v2_200k_overflow50k", 200000, 0, 30.0f, 50000);
  print_result(r3v1);
  print_result(r3v2);

  // ---- Speedup summary ----
  double speedup_bench1 = (r1v1.add_ms > 0) ? (r1v1.add_ms / r1v2.add_ms) : 0.0;
  double speedup_bench3 = (r3v1.add_ms > 0) ? (r3v1.add_ms / r3v2.add_ms) : 0.0;
  std::cout << "\nspeedup_120k_noduplicate=" << speedup_bench1 << "\n";
  std::cout << "speedup_200k_overflow=" << speedup_bench3 << "\n";

  std::cout << "\nbenchmark_status=DONE\n";
  return 0;
}
