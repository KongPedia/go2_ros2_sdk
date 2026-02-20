#!/usr/bin/env python3
"""
run_all_benchmarks.py -- Run P1/P2/P3 benchmark tests and generate BENCHMARK_SUMMARY.md.

Usage (inside Docker):
  source /ros2_ws/install/setup.bash
  python /ros2_ws/src/third_party/go2_ros2_sdk/run_all_benchmarks.py

Or via docker exec:
  docker exec 6429a0513461 bash -c "
    source /ros2_ws/install/setup.bash && \\
    python /ros2_ws/src/third_party/go2_ros2_sdk/run_all_benchmarks.py"
"""

import subprocess
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
_RESULTS_DIR = _ROOT / "benchmark_results"

_TESTS = [
    (
        "P1",
        _ROOT / "lidar_processor_cpp" / "test" / "test_p1_aggregator.py",
    ),
    (
        "P2",
        _ROOT / "lidar_processor_cpp" / "test" / "test_p2_aggregator_filter.py",
    ),
    (
        "P3",
        _ROOT / "lidar_accelerator" / "tests" / "test_p3_processing_simd.py",
    ),
]


def run_pytest(test_path: Path, label: str) -> bool:
    print(f"\n{'=' * 60}")
    print(f"  Running {label}: {test_path.name}")
    print(f"{'=' * 60}")
    result = subprocess.run(
        [sys.executable, "-m", "pytest", str(test_path), "-v", "--tb=short", "-q"],
        text=True,
    )
    return result.returncode == 0


def main() -> int:
    all_passed = True
    for label, path in _TESTS:
        if not path.exists():
            print(f"[SKIP] {label}: {path} not found")
            continue
        passed = run_pytest(path, label)
        if not passed:
            print(f"[WARN] {label} had test failures -- report may be incomplete")
            all_passed = False

    print(f"\n{'=' * 60}")
    print("  Generating BENCHMARK_SUMMARY.md ...")
    print(f"{'=' * 60}")

    sys.path.insert(0, str(_ROOT))
    try:
        from benchmark_results.benchmark_writer import consolidate
        out = consolidate()
        print(f"\nSummary written to: {out}")
        print()
        print(out.read_text(encoding="utf-8"))
    except FileNotFoundError as e:
        print(f"[ERROR] Could not consolidate: {e}")
        return 1

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
