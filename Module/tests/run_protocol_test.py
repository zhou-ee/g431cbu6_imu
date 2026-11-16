#!/usr/bin/env python3
"""Build and run the protocol codec test on the host.

只编译 Communication/imu_protocol.cpp 与测试本身：协议层是纯函数、没有 HAL/FreeRTOS
依赖，所以可以直接在主机上逐字节核对 PROTOCOL.md 的字节向量与原码回环。
"""

import subprocess
import sys
import tempfile
from pathlib import Path


def main() -> int:
    module_dir = Path(__file__).resolve().parents[1]  # .../IMU_Module/Module
    out_dir = Path(tempfile.mkdtemp(prefix="protocol_test_"))
    exe = out_dir / "test_protocol.exe"

    cmd = [
        "g++",
        "-std=c++17",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I", str(module_dir / "Communication"),
        "-I", str(module_dir / "IMU"),
        "-I", str(module_dir / "Config"),
        str(module_dir / "tests" / "test_protocol.cpp"),
        str(module_dir / "Communication" / "imu_protocol.cpp"),
        "-o", str(exe),
    ]
    subprocess.run(cmd, check=True)
    result = subprocess.run([str(exe)], check=False)
    print(f"Build directory {out_dir}")
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
