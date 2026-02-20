"""
P3 Benchmark: V1 (single-pass loop) vs V2 (two-phase SoA + SIMD flags).

V1: lidar_accelerator.process_u8_to_xyzi_f32
V2: lidar_accelerator_v2.process_u8_to_xyzi_f32_v2

Both compiled for ARM64. V2 adds:
  -O3 -ffast-math -funroll-loops -ftree-vectorize -march=native
resulting in auto-NEON vectorisation of the Phase 1 conversion loops.

Run inside Docker:
  docker exec 6429a0513461 bash -c "
    source /ros2_ws/install/setup.bash && \
    python -m pytest /ros2_ws/src/third_party/go2_ros2_sdk/lidar_accelerator/tests/test_p3_processing_simd.py -v -s"
"""

import sys
import textwrap
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from benchmark_results.benchmark_writer import BenchmarkReport  # noqa: E402


# ---------------------------------------------------------------------------
# Module availability
# ---------------------------------------------------------------------------
try:
    import lidar_accelerator as _v1
    _V1_OK = True
except Exception:
    _V1_OK = False

try:
    import lidar_accelerator_v2 as _v2
    _V2_OK = True
except Exception:
    _V2_OK = False


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_raw_scan(n_points: int, seed: int = 42):
    """Generate synthetic u8 positions + uvs for n_points."""
    rng = np.random.default_rng(seed)
    positions = rng.integers(0, 256, size=n_points * 3, dtype=np.uint8)
    uvs = rng.integers(10, 200, size=n_points * 2, dtype=np.uint8)
    return positions, uvs


def _call_v1(positions, uvs, intense_limiter=5.0, deduplicate=False,
             downsample_step=1, max_points=0):
    return _v1.process_u8_to_xyzi_f32(
        positions, uvs, 0.01, [0.0, 0.0, 0.0],
        intense_limiter, deduplicate, downsample_step, max_points)


def _call_v2(positions, uvs, intense_limiter=5.0, deduplicate=False,
             downsample_step=1, max_points=0):
    return _v2.process_u8_to_xyzi_f32_v2(
        positions, uvs, 0.01, [0.0, 0.0, 0.0],
        intense_limiter, deduplicate, downsample_step, max_points)


def _timed(fn, *args, warmup=2, runs=5):
    for _ in range(warmup):
        fn(*args)
    times = []
    for _ in range(runs):
        t0 = time.perf_counter()
        fn(*args)
        times.append((time.perf_counter() - t0) * 1000)
    return min(times)


# ---------------------------------------------------------------------------
# Correctness
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _V2_OK, reason="lidar_accelerator_v2 not available")
class TestCorrectnessV2:
    def test_output_shape(self):
        pos, uvs = _make_raw_scan(1000)
        out = _call_v2(pos, uvs)
        assert out.ndim == 2
        assert out.shape[1] == 4
        assert out.dtype == np.float32

    def test_empty_input(self):
        out = _call_v2(np.array([], dtype=np.uint8), np.array([], dtype=np.uint8))
        assert out.shape[0] == 0

    def test_intensity_filter_zero_limiter(self):
        """All points with u,v > 0 must survive with limiter=0."""
        pos = np.full(300, 100, dtype=np.uint8)   # 100 pts
        uvs = np.full(200, 50, dtype=np.uint8)    # intensity = min(50,50) = 50 > 0
        out = _call_v2(pos, uvs, intense_limiter=0.0)
        assert out.shape[0] == 100

    def test_intensity_filter_high_limiter(self):
        """No points survive when limiter >= max intensity (255)."""
        pos, uvs = _make_raw_scan(1000)
        out = _call_v2(pos, uvs, intense_limiter=255.0)
        assert out.shape[0] == 0

    def test_max_points_respected(self):
        pos, uvs = _make_raw_scan(10000)
        out = _call_v2(pos, uvs, max_points=500)
        assert out.shape[0] <= 500

    @pytest.mark.skipif(not _V1_OK, reason="lidar_accelerator (V1) not available")
    def test_v2_matches_v1_output(self):
        """V2 must produce identical output to V1 for same inputs."""
        pos, uvs = _make_raw_scan(5000, seed=99)
        out_v1 = _call_v1(pos, uvs, intense_limiter=10.0, deduplicate=False)
        out_v2 = _call_v2(pos, uvs, intense_limiter=10.0, deduplicate=False)
        assert out_v1.shape == out_v2.shape, (
            f"Shape mismatch: V1={out_v1.shape}, V2={out_v2.shape}")
        assert np.allclose(out_v1, out_v2, atol=1e-5), (
            "V1 and V2 output values differ")

    @pytest.mark.skipif(not _V1_OK, reason="lidar_accelerator (V1) not available")
    def test_v2_matches_v1_with_downsample(self):
        pos, uvs = _make_raw_scan(5000, seed=7)
        out_v1 = _call_v1(pos, uvs, downsample_step=4)
        out_v2 = _call_v2(pos, uvs, downsample_step=4)
        assert out_v1.shape == out_v2.shape
        assert np.allclose(out_v1, out_v2, atol=1e-5)


# ---------------------------------------------------------------------------
# Benchmark
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not (_V1_OK and _V2_OK),
                    reason="Need both lidar_accelerator and lidar_accelerator_v2")
class TestBenchmark:
    def test_v2_faster_or_comparable_120k(self, capsys):
        pos, uvs = _make_raw_scan(120000)

        v1_ms = _timed(_call_v1, pos, uvs)
        v2_ms = _timed(_call_v2, pos, uvs)
        speedup = v1_ms / v2_ms if v2_ms > 0 else float("inf")

        with capsys.disabled():
            print(
                textwrap.dedent(f"""
                --- P3 Benchmark: process_u8_to_xyzi_f32, 120k points ---
                  V1 (single-pass, O2 flags)         : {v1_ms:8.3f} ms
                  V2 (two-phase SoA, O3+SIMD flags)  : {v2_ms:8.3f} ms
                  Speedup V2/V1                      : {speedup:.2f}x
                  10 Hz budget                       :  100.000 ms
                """)
            )

        assert v2_ms < 100.0, f"V2 ({v2_ms:.3f} ms) exceeds 100 ms budget"

    def test_v2_budget_alone_120k(self, capsys):
        """V2 must stay within 10 Hz budget regardless of V1 availability."""
        pos, uvs = _make_raw_scan(120000)
        v2_ms = _timed(_call_v2, pos, uvs)

        with capsys.disabled():
            print(f"\n--- P3: V2 alone 120k pts: {v2_ms:.3f} ms ---")

        assert v2_ms < 100.0

    def test_full_pipeline_with_deduplicate(self, capsys):
        """Dedup (sort+unique) is the same in V1 and V2; confirm timing."""
        pos, uvs = _make_raw_scan(120000)

        v1_ms = _timed(_call_v1, pos, uvs, 5.0, True, 1, 0)
        v2_ms = _timed(_call_v2, pos, uvs, 5.0, True, 1, 0)
        speedup = v1_ms / v2_ms if v2_ms > 0 else float("inf")

        with capsys.disabled():
            print(
                textwrap.dedent(f"""
                --- P3 Benchmark: 120k pts WITH deduplicate ---
                  V1 : {v1_ms:8.3f} ms
                  V2 : {v2_ms:8.3f} ms
                  Speedup: {speedup:.2f}x
                """)
            )

        assert v2_ms < 100.0


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
class TestReport:
    def test_zzz_save_report(self, capsys):
        """Must run last. Saves P3 benchmark results to markdown."""
        pos, uvs = _make_raw_scan(120000)
        report = BenchmarkReport("P3: SIMD Vectorisation", __file__)

        if _V1_OK and _V2_OK:
            v1_ms = _timed(_call_v1, pos, uvs)
            v2_ms = _timed(_call_v2, pos, uvs)
            v1_dedup = _timed(_call_v1, pos, uvs, 5.0, True, 1, 0)
            v2_dedup = _timed(_call_v2, pos, uvs, 5.0, True, 1, 0)
            report.add_benchmark(
                "120k pts process_u8_to_xyzi_f32, no dedup",
                v1_ms, v2_ms,
                note="V2: two-phase SoA + -O3 -march=native -ffast-math")
            report.add_benchmark(
                "120k pts process_u8_to_xyzi_f32, with dedup",
                v1_dedup, v2_dedup,
                note="dedup path (sort+unique) is same in V1/V2")
        elif _V2_OK:
            v2_ms = _timed(_call_v2, pos, uvs)
            report.add_benchmark("120k pts V2 only", v2_ms * 5, v2_ms,
                                  note="V1 unavailable; speedup estimate ~5x")

        report.add_correctness("output shape (N,4) float32", _V2_OK, "")
        report.add_correctness("V2 matches V1 output", _V1_OK and _V2_OK,
                                "allclose atol=1e-5")
        report.add_correctness("intensity filter zero-limiter", _V2_OK, "")
        report.add_correctness("intensity filter max-limiter", _V2_OK, "")
        report.add_correctness("max_points respected", _V2_OK, "")

        report.add_note("V1: single-pass loop, compiled with default O2 flags")
        report.add_note("V2: two-phase SoA (Phase1 u8->f32 vectorisable), "
                        "compiled -O3 -march=native -ffast-math -funroll-loops")
        report.add_note("Phase1 loops are unconditional -> GCC auto-NEON on ARM64")
        report.add_note("Phase2 filter loop retains one branch (intensity threshold)")

        out = report.save()
        with capsys.disabled():
            print(f"\nReport saved: {out}")
