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
    repo_dir = module_dir.parent                       # .../IMU_Module
    crc_dir = repo_dir / "PYRo" / "Algorithm" / "CRC"
    defs_dir = repo_dir / "PYRo" / "Core" / "Def"      # pyro_core_def.h（fusion 头要用）
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
        "-I", str(crc_dir),
        "-I", str(defs_dir),
        str(module_dir / "tests" / "test_protocol.cpp"),
        str(module_dir / "Communication" / "imu_protocol.cpp"),
        # CRC 实现在 PYRo 层，测试仍可在主机上独立编译（无需 HAL/FreeRTOS）
        str(crc_dir / "pyro_crc.cpp"),
        "-o", str(exe),
    ]
    subprocess.run(cmd, check=True)
    result = subprocess.run([str(exe)], check=False)
    print(f"Build directory {out_dir}")
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
