#!/usr/bin/env python3
"""SPSC 环形缓冲的并发正确性验证（主机侧，无需硬件）。

跑三个测试，全部通过才算 SPSC 改造有效：

  1. spsc_race_test      —— 双线程真抢占，400 万样本：无撕裂、序号单调、无丢失。
                            这是"无需临界区"的正向证据。
  2. spsc_mutation_test  —— 变异测试：把两个历史缺陷（先 publish 后写、
                            满则覆盖最旧）重新实现一遍，必须被同一套判据捕获。
                            这是"判据不是空测"的反向证据。
  3. 与 VQF 回归串联     —— SPSC 改造不得改变任何数值（见 run_vqf_eskf.py）。

用法（在仓库根目录）：
    python PYRo/tests/run_spsc_test.py
"""

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def build_and_run(cxx: str, sources: list, includes: list, name: str, work: Path) -> int:
    exe = work / (name + (".exe" if sys.platform.startswith("win") else ""))
    cmd = [cxx, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-pthread"]
    cmd += ["-I" + str(p) for p in includes]
    cmd += [str(s) for s in sources] + ["-o", str(exe)]
    print("building " + name, flush=True)
    subprocess.run(cmd, check=True)
    print("running " + name, flush=True)
    return subprocess.run([str(exe)], check=False, timeout=180).returncode


def main() -> int:
    repo = Path(__file__).resolve().parents[2]
    tests = repo / "PYRo" / "tests"
    imu = repo / "PYRo" / "Component" / "IMU"

    cxx = shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        raise SystemExit("需要主机 g++ / clang++ 在 PATH 上（或设置 CXX）。")

    work = Path(tempfile.mkdtemp(prefix="spsc_test_"))
    failures = 0

    # 1) 正向：真实抢占下 SPSC 不撕裂
    failures += build_and_run(
        cxx, [tests / "spsc_race_test.cpp"], [imu], "spsc_race_test", work)

    # 2) 反向：变异测试证明判据有效
    failures += build_and_run(
        cxx, [tests / "spsc_mutation_test.cpp"], [imu], "spsc_mutation_test", work)

    print("build directory " + str(work))
    if failures:
        print("FAILED: %d test(s) failed" % failures)
        return 1
    print("PASS: SPSC concurrency invariants hold (no lock, no tearing, no loss)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
