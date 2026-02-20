"""
P1 Aggregator benchmark/correctness test.

Compile-and-run strategy:
  - Compiles bench_aggregator_main.cpp with g++ -std=c++17 -O3
  - Parses key=value output lines
  - Asserts correctness for both V1 and V2
  - Prints benchmark comparison table

Run inside Docker container:
  docker exec 6429a0513461 bash -c "
    source /ros2_ws/install/setup.bash && \
    python -m pytest /ros2_ws/src/third_party/go2_ros2_sdk/lidar_processor_cpp/test/test_p1_aggregator.py -v -s"
"""

import subprocess
import sys
import tempfile
import textwrap
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from benchmark_results.benchmark_writer import BenchmarkReport  # noqa: E402

# ---------------------------------------------------------------------------
# Paths (relative to this file, works in container and on host)
# ---------------------------------------------------------------------------
_HERE = Path(__file__).resolve().parent
_INCLUDE = _HERE.parent / "include"
_BENCH_SRC = _HERE / "bench_aggregator_main.cpp"


def _compile_bench(tmp_dir: Path) -> Path:
    """Compile the benchmark binary; return path to binary."""
    out = tmp_dir / "bench_aggregator"
    result = subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-O3",
            "-pthread",
            f"-I{_INCLUDE}",
            str(_BENCH_SRC),
            "-o",
            str(out),
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        pytest.fail(
            f"Compilation failed:\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )
    return out


def _run_bench(binary: Path) -> tuple[dict, str]:
    """Run the benchmark binary and parse key=value output into a dict."""
    result = subprocess.run(
        [str(binary)],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        pytest.fail(
            f"Benchmark exited with code {result.returncode}:\n"
            f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )

    data = {}
    for line in result.stdout.splitlines():
        line = line.strip()
        if "=" in line and not line.startswith("#") and not line.startswith("---"):
            key, _, val = line.partition("=")
            data[key.strip()] = val.strip()
    return data, result.stdout


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def bench_results():
    with tempfile.TemporaryDirectory() as tmp:
        binary = _compile_bench(Path(tmp))
        data, raw = _run_bench(binary)
    return data, raw


# ---------------------------------------------------------------------------
# Correctness tests
# ---------------------------------------------------------------------------
class TestCorrectnessV1:
    def test_no_failures(self, bench_results):
        data, _ = bench_results
        assert int(data["v1_correctness_fails"]) == 0, (
            f"V1 had {data['v1_correctness_fails']} correctness failure(s)"
        )

    def test_status_pass(self, bench_results):
        data, _ = bench_results
        assert data.get("correctness_status") == "PASS"

    def test_hash_collision_documented(self, bench_results):
        """V1 has a known hash collision at y=65m; storage still correct (both points kept)."""
        data, _raw = bench_results
        assert data.get("v1_hash_collision_demo") == "bucket_collision_at_y65m"


class TestCorrectnessV2:
    def test_no_failures(self, bench_results):
        data, _ = bench_results
        assert int(data["v2_correctness_fails"]) == 0, (
            f"V2 had {data['v2_correctness_fails']} correctness failure(s)"
        )

    def test_no_hash_collision(self, bench_results):
        """V2 must NOT produce hash collisions at large coordinates."""
        data, _ = bench_results
        assert data.get("v2_no_hash_collision") == "ok"


# ---------------------------------------------------------------------------
# Benchmark assertions (not strict pass/fail on timing, but document speedup)
# ---------------------------------------------------------------------------
class TestBenchmark:
    def test_benchmark_ran(self, bench_results):
        data, _ = bench_results
        assert data.get("benchmark_status") == "DONE"

    def test_v2_performance_120k_no_dup(self, bench_results, capsys):
        """Document the no-overflow tradeoff.

        V2 uses std::floor()*3 per point for voxel key computation, making it
        ~1.5x slower than V1 for the pure no-overflow case.  This is accepted
        because V2 wins massively on the overflow case (see test below) and has
        correct hash semantics for large coordinates.
        The absolute cost (<=30 ms per 120k frame) is well within 10 Hz budget.
        """
        data, _raw = bench_results
        v1_add = float(data["v1_120k_noduplicate_nolimit_add_ms"])
        v2_add = float(data["v2_120k_noduplicate_nolimit_add_ms"])
        speedup = float(data.get("speedup_120k_noduplicate", 0))

        with capsys.disabled():
            print(
                textwrap.dedent(f"""
                --- P1 Benchmark: 120k unique points (full L1 scan), addPoints() ---
                  V1 (unordered_set + buggy hash) : {v1_add:8.3f} ms
                  V2 (voxel hash map, fixed hash)  : {v2_add:8.3f} ms
                  Speedup V2/V1                    : {speedup:.2f}x
                  Note: V2 floor()*3 overhead is acceptable (<30 ms << 100 ms budget)
                """)
            )
        # Must stay within 10 Hz real-time budget (100 ms per frame)
        assert v2_add < 100.0, f"V2 addPoints ({v2_add:.3f} ms) exceeds 100 ms budget"

    def test_v2_faster_overflow_case(self, bench_results, capsys):
        data, _ = bench_results
        v1_add = float(data["v1_200k_overflow50k_add_ms"])
        v2_add = float(data["v2_200k_overflow50k_add_ms"])
        speedup = float(data.get("speedup_200k_overflow", 0))

        with capsys.disabled():
            print(
                textwrap.dedent(f"""
                --- P1 Benchmark: 200k input, max_points=50k (overflow case) ---
                  V1 (sort-on-overflow O(N log N)) : {v1_add:8.3f} ms
                  V2 (skip-on-overflow O(1))       : {v2_add:8.3f} ms
                  Speedup V2/V1                    : {speedup:.2f}x
                """)
            )
        # Overflow case: V2 must be faster (V1 triggers expensive O(N log N) sort)
        assert v2_add < v1_add, (
            f"V2 ({v2_add:.3f} ms) should be faster than V1 ({v1_add:.3f} ms) in overflow case"
        )

    def test_print_full_raw_output(self, bench_results, capsys):
        _, raw = bench_results
        with capsys.disabled():
            print("\n=== Full benchmark output ===\n")
            print(raw)


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
class TestReport:
    def test_zzz_save_report(self, bench_results):
        """Must run last (zzz prefix). Saves benchmark results to markdown."""
        data, _ = bench_results
        report = BenchmarkReport("P1: PointCloudAggregator", __file__)

        report.add_correctness(
            "V1 correctness", int(data["v1_correctness_fails"]) == 0,
            f"{data['v1_correctness_fails']} failures")
        report.add_correctness(
            "V2 correctness", int(data["v2_correctness_fails"]) == 0,
            f"{data['v2_correctness_fails']} failures")
        report.add_correctness(
            "V1 hash collision documented",
            data.get("v1_hash_collision_demo") == "bucket_collision_at_y65m",
            "bucket collision at y=65m")
        report.add_correctness(
            "V2 no hash collision",
            data.get("v2_no_hash_collision") == "ok",
            "21-bit packed key covers ±1048 m")

        report.add_benchmark(
            "120k unique pts (normal scan), addPoints()",
            float(data["v1_120k_noduplicate_nolimit_add_ms"]),
            float(data["v2_120k_noduplicate_nolimit_add_ms"]),
            note="V2 ~1.5x slower due to floor()*3; still <100ms")
        report.add_benchmark(
            "200k pts → overflow to 50k, addPoints()",
            float(data["v1_200k_overflow50k_add_ms"]),
            float(data["v2_200k_overflow50k_add_ms"]),
            note="V1 O(N log N) sort; V2 O(1) skip")

        report.add_note("V1: std::unordered_set + buggy 16-bit shift hash (collision at y≥65m)")
        report.add_note("V2: std::unordered_map<uint64_t,Point3D> with 21-bit packed voxel key")
        report.add_note("V2 built with -O3; V1 built with default (same flags here)")

        out = report.save()
        print(f"\nReport saved: {out}")
