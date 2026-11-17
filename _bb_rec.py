"""通过 SWD 实时采集 IMU 内部量到本地 CSV（telnet 4444 版本）。

为什么不走板载环形缓冲：
  板载缓冲只有 256 行 × 20 B（1.28 s @200 Hz），抓不住"剧烈运动→静止"全过程。

为什么用 telnet 4444 而不是 tcl 6666：
  tcl 端口在本机这套 OpenOCD 上不返回 banner、握手不稳；
  telnet 端口会正常给出 "> " 提示符，实测可用。

采样率：每次 = 1 条 mdw（96 B 连续区）+ 1 条 mdw（tick），
受 SWD 往返与 OpenOCD 命令解析限制，约 20~40 Hz。
对"约半秒后 yaw 突变"这个时间尺度足够（跨跳变有 10+ 个点）。

用法：
    python bb_rec.py <seconds> <out.csv>
运行时按提示做动作：静止 → 剧烈运动 3 s → 立刻静止 3 s。
"""
import re
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(r"E:\Software\Develop\Embeded\Pack\lsm6dsv_VQF\IMU_Module")
ELF = ROOT / "build" / "Release" / "IMU_Module.elf"
NM = "arm-none-eabi-nm"
OPENOCD = "openocd"

# 连续可读区间：从 imu_service_acc_rejected(0x2000569C) 起 36 个字 = 144 B。
# 布局已用 nm 地址 + 实际值双向核对（acc_z=9.73 是重力，cap_rounds 单调增长）。
# 注意：必须从 acc_rejected 起读，roll/pitch/yaw/rest 都在 0x200056C0 之前。
BASE = 0x2000569C
NWORDS = 36

IDX = {
    "acc_rej": 0, "nis": 1, "acc_conf": 2, "hz_quality": 3, "hz_zeroed": 4,
    # rest_acc_lp[3] @ +20B
    "rest_acc_lp_x": 5, "rest_acc_lp_y": 6, "rest_acc_lp_z": 7,
    # rest_gyr_lp[3] @ +32B —— _correct_rest 的观测源，诊断核心
    "rest_lp_x": 8, "rest_lp_y": 9, "rest_lp_z": 10,
    "rest_t": 11, "rest": 12,
    "yaw": 13, "pitch": 14, "roll": 15,
    "bias_x": 16, "bias_y": 17, "bias_z": 18,
    "gyro_x": 19, "gyro_y": 20, "gyro_z": 21,
    "acc_x": 22, "acc_y": 23, "acc_z": 24,
    "cap_us_avg": 25, "cap_us_max": 26, "cap_rounds": 27,
    "cap_stack": 28, "ins_stack": 29, "heap_free": 30,
    "loop_us_avg": 31, "vqf_us_avg": 32, "vqf_us_max": 33,
    "vqf_status": 34, "vqf_warn": 35,
}


def symtab() -> dict:
    out = subprocess.run([NM, "-S", "--defined-only", str(ELF)],
                         capture_output=True, text=True, check=True).stdout
    t = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) >= 4:
            t[p[3]] = int(p[0], 16)
    return t


class Telnet:
    def __init__(self, port=4444):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=10)
        self.s.settimeout(5)
        self.buf = b""
        # 吃掉 banner（含 IAC 协商字节 \xff...）直到提示符
        self._drain(time.time() + 3)

    def _drain(self, deadline):
        while time.time() < deadline:
            try:
                chunk = self.s.recv(65536)
            except socket.timeout:
                return
            if not chunk:
                return
            self.buf += chunk

    def cmd(self, line):
        self.s.sendall((line + "\n").encode())
        deadline = time.time() + 5
        self.buf = b""
        while time.time() < deadline:
            try:
                chunk = self.s.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                break
            self.buf += chunk
            if self.buf.rstrip().endswith(b">"):
                break
        return self.buf.decode(errors="replace")

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass


def f32(u):
    return struct.unpack("<f", struct.pack("<I", u & 0xFFFFFFFF))[0]


def parse_mdw(txt, want):
    """解析 telnet 返回的 mdw。实际格式（含回显与 NUL 前缀）：
        mdw 0x200056C4 4\\r\\n\\x000x200056c4: 3c81a2eb bc0f539b ... \\r\\n\\r> "
    因此按行找 '0x........: ' 之后的所有 8 位十六进制串。"""
    vals = []
    for line in txt.splitlines():
        line = line.strip("\x00\r ")
        m = re.match(r"^(0x[0-9a-fA-F]{8}):\s*(.*)$", line)
        if not m:
            continue
        for tok in m.group(2).split():
            try:
                vals.append(int(tok, 16))
            except ValueError:
                pass
    return vals if len(vals) >= want else None


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "bb_rec.csv"
    tab = symtab()
    tick_addr = tab["xTickCount"]

    proc = subprocess.Popen(
        [OPENOCD, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32g4x.cfg",
         "-c", "adapter speed 2000", "-c", "init"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rows = []
    try:
        time.sleep(4)
        tn = Telnet()
        tn.cmd("halt")
        tn.cmd("resume")
        print("recording... do: still -> shake 3s -> still 3s", flush=True)
        t0 = time.time()
        while time.time() - t0 < secs:
            a = parse_mdw(tn.cmd(f"mdw 0x{BASE:08X} {NWORDS}"), NWORDS)
            if a is None:
                continue
            tk = parse_mdw(tn.cmd(f"mdw 0x{tick_addr:08X} 1"), 1)
            rows.append({
                "t_host": round(time.time() - t0, 4),
                "tick_ms": tk[0] if tk else 0,
                "yaw": f32(a[IDX["yaw"]]),
                "pitch": f32(a[IDX["pitch"]]),
                "roll": f32(a[IDX["roll"]]),
                "rest": a[IDX["rest"]],
                "rest_t": f32(a[IDX["rest_t"]]),
                "rest_lp_x": f32(a[IDX["rest_lp_x"]]),
                "rest_lp_y": f32(a[IDX["rest_lp_y"]]),
                "rest_lp_z": f32(a[IDX["rest_lp_z"]]),
                "rest_acc_lp_x": f32(a[IDX["rest_acc_lp_x"]]),
                "rest_acc_lp_y": f32(a[IDX["rest_acc_lp_y"]]),
                "rest_acc_lp_z": f32(a[IDX["rest_acc_lp_z"]]),
                "acc_conf": f32(a[IDX["acc_conf"]]),
                "nis": f32(a[IDX["nis"]]),
                "acc_rej": a[IDX["acc_rej"]],
                "bias_x": f32(a[IDX["bias_x"]]),
                "bias_y": f32(a[IDX["bias_y"]]),
                "bias_z": f32(a[IDX["bias_z"]]),
                "gyro_x": f32(a[IDX["gyro_x"]]),
                "gyro_y": f32(a[IDX["gyro_y"]]),
                "gyro_z": f32(a[IDX["gyro_z"]]),
                "acc_x": f32(a[IDX["acc_x"]]),
                "acc_y": f32(a[IDX["acc_y"]]),
                "acc_z": f32(a[IDX["acc_z"]]),
                "cap_us_avg": a[IDX["cap_us_avg"]],
                "cap_us_max": a[IDX["cap_us_max"]],
                "cap_rounds": a[IDX["cap_rounds"]],
                "ins_stack": a[IDX["ins_stack"]],
                "heap_free": a[IDX["heap_free"]],
                "loop_us_avg": a[IDX["loop_us_avg"]],
                "vqf_us_avg": a[IDX["vqf_us_avg"]],
                "vqf_us_max": a[IDX["vqf_us_max"]],
                "vqf_warn": a[IDX["vqf_warn"]],
            })
        tn.close()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()

    if not rows:
        print("NO DATA")
        return 1
    cols = list(rows[0].keys())
    with open(out, "w", encoding="utf-8", newline="") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(str(r[c]) for c in cols) + "\n")
    dur = rows[-1]["t_host"] - rows[0]["t_host"]
    print(f"rows={len(rows)} duration={dur:.1f}s rate={len(rows)/dur:.1f} Hz")
    print(f"csv -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
