"""Host regression using the vendored CMSIS-DSP scalar kernels.

Run from the repository root with Python 3 and native gcc/g++ on PATH.
Build products are placed in a temporary directory.
"""
from pathlib import Path
import argparse
import os
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-dir", type=Path,
                        help="Optional directory containing the previous VQF .h/.cpp for comparison")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    dsp = repo / "third_party/CMSIS-DSP"
    work = Path(tempfile.mkdtemp(prefix="vqf_eskf_test_"))
    cc = os.environ.get("CC") or shutil.which("gcc")
    cxx = os.environ.get("CXX") or shutil.which("g++")
    if not cc or not cxx:
        raise SystemExit("Native gcc and g++ are required on PATH, or set CC and CXX.")
    includes = [dsp / "Include", dsp / "PrivateInclude", repo / "CubeMX/Drivers/CMSIS/Include",
                repo / "PYRo/Core/Def", repo / "PYRo/Algorithm/Matrix",
                repo / "PYRo/Algorithm/KF", repo / "PYRo/Algorithm/Butterworth",
                repo / "PYRo/Component/IMU"]
    flags = ["-O2", "-g", "-D__GNUC_PYTHON__", "-DARM_MATH_MATRIX_CHECK"]
    flags += ["-I" + str(path) for path in includes]
    kernels = ["MatrixFunctions/arm_mat_" + name + "_f32.c" for name in
               ["init", "add", "sub", "mult", "trans", "inverse", "cholesky",
                "solve_lower_triangular", "solve_upper_triangular"]]
    kernels += ["QuaternionMathFunctions/arm_" + name + "_f32.c" for name in
                ["quaternion_normalize", "quaternion_product_single", "quaternion2rotation"]]
    objects = []
    for kernel in kernels:
        output = work / (Path(kernel).stem + ".o")
        subprocess.run([cc, "-std=c11", *flags, "-c", str(dsp / "Source" / kernel),
                        "-o", str(output)], check=True)
        objects.append(str(output))
    targets = [(repo / "PYRo/Algorithm/VQF", "eskf", [])]
    if args.baseline_dir:
        targets.append((args.baseline_dir.resolve(), "baseline", ["-DVQF_TEST_BASELINE"]))
    for source, name, defines in targets:
        executable = work / (name + (".exe" if os.name == "nt" else ""))
        subprocess.run([cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", *flags, *defines,
                        "-I" + str(source), str(repo / "PYRo/tests/vqf_eskf_test.cpp"),
                        str(source / "pyro_algo_vqf.cpp"),
                        str(repo / "PYRo/Algorithm/Matrix/pyro_algo_matrix.cpp"), *objects,
                        "-Wl,--wrap=arm_mat_mult_f32", "-o", str(executable)], check=True)
        print("Running " + name, flush=True)
        subprocess.run([str(executable)], check=True, timeout=90)
    print("Build directory " + str(work))


if __name__ == "__main__":
    main()
