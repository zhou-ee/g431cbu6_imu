"""黑匣子操控与回读。

用法：
    python bb.py flash      # 烧录并启动（复位记录器）
    python bb.py reset      # 只清空记录器（不重新烧录）
    python bb.py dump       # 回读记录并解析成表格
    python bb.py status     # 只看实时状态

设计要点：
  - OpenOCD 一次性启动、用完即退（避免 DAPLink 被常驻进程占死）。
  - 记录器是环形缓冲：head 指向下一个写入位置，total 是累计行数。
    回读时按 total 与 kRows 算出有效行范围，按写入顺序输出。
"""
import re
import struct
import subprocess
import sys
from pathlib import Path

ROOT = Path(r"E:\Software\Develop\Embeded\Pack\lsm6dsv_VQF\IMU_Module")
ELF = ROOT / "build" / "Release" / "IMU_Module.elf"
NM = "arm-none-eabi-nm"
OPENOCD = "openocd"
GDB = "arm-none-eabi-gdb"

ROWS = 256
ROW_BYTES = 20


def symtab() -> dict:
    out = subprocess.run([NM, "-S", "--defined-only", str(ELF)],
                         capture_output=True, text=True, check=True).stdout
    t = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) >= 4:
            t[p[3]] = int(p[0], 16)
    return t


def ocd(commands, expect_load=False):
    """一次性 OpenOCD：init 后执行命令再 shutdown。"""
    args = [OPENOCD, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32g4x.cfg",
            "-c", "adapter speed 1000", "-c", "init"]
    for c in commands:
        args += ["-c", c]
    args += ["-c", "shutdown"]
    r = subprocess.run(args, capture_output=True, text=True)
    return r.stdout + r.stderr


def do_flash():
    gdb_script = f"""
set pagination off
set confirm off
target extended-remote localhost:3333
monitor reset halt
load
monitor reset run
detach
quit
"""
    # 需要一个临时 openocd 常驻来接受 gdb
    p = subprocess.Popen(
        [OPENOCD, "-f", "interface/cmsis-dap.cfg", "-f", "target/stm32g4x.cfg",
         "-c", "adapter speed 1000", "-c", "init"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        import time
        time.sleep(3)
        g = Path(r"C:\Users\zhou_\AppData\Local\Temp\imu_bb\flash.gdb")
        g.parent.mkdir(parents=True, exist_ok=True)
        g.write_text(gdb_script, encoding="ascii")
        r = subprocess.run([GDB, "-batch", "-x", str(g), str(ELF)],
                           capture_output=True, text=True)
        ok = "Transfer rate" in (r.stdout + r.stderr)
        print("flashed" if ok else "FLASH FAILED")
        if not ok:
            print((r.stdout + r.stderr)[-1500:])
    finally:
        p.terminate()
        p.wait(timeout=10)


def read_words(addr, count):
    """用 OpenOCD mdw 读内存，返回 int 列表。"""
    out = ocd([f"halt", f"mdw 0x{addr:08X} {count}", "resume"])
    vals = []
    for line in out.splitlines():
        if ": " not in line:
            continue
        head, _, rest = line.partition(": ")
        if not re.match(r"^0x[0-9a-fA-F]{8}$", head.strip()):
            continue
        for tok in rest.split():
            if re.match(r"^[0-9a-fA-F]{8}$", tok):
                vals.append(int(tok, 16))
    return vals


def read_one(addr):
    v = read_words(addr, 1)
    return v[0] if v else None


def as_float(u32):
    return struct.unpack("<f", struct.pack("<I", u32 & 0xFFFFFFFF))[0]


def row_from(block):
    """block: 5 个 uint32。对应 blackbox::row_t 的 20 字节布局。"""
    w0, w1, w2, w3, w4 = block
    tick_mod = w0 & 0xFFFF
    rest = (w0 >> 16) & 0xFF
    flags = (w0 >> 24) & 0xFF
    return {
        "tick": tick_mod,
        "rest": rest,
        "flags": flags,
        "yaw": as_float(w1),
        "bias_z": as_float(w2),
        "dyaw": as_float(w3),
        "rest_t": as_float(w4),
    }


def do_status():
    t = symtab()
    head = read_one(t["imu_bb_head"])
    total = read_one(t["imu_bb_total"])
    ev = read_one(t["imu_bb_events"])
    frozen = read_one(t["imu_bb_frozen"])
    mdy = as_float(read_one(t["imu_bb_max_dyaw"]))
    initr = read_one(t["imu_module_init_result"])
    yaw = as_float(read_one(t["imu_service_yaw"]))
    rest = read_one(t["imu_service_rest"])
    rest_t = as_float(read_one(t["imu_service_rest_t"]))
    bz = as_float(read_one(t["imu_service_bias_rps"] + 8))
    print(f"init={initr}  head={head}  total={total}  events={ev}  "
          f"FROZEN={frozen}  max_dyaw={mdy:.3f}")
    print(f"yaw={yaw:.3f}deg  rest={rest}  rest_t={rest_t:.3f}s  bias_z={bz:.6f}rad/s")


def do_reset():
    t = symtab()
    for name in ("imu_bb_head", "imu_bb_total", "imu_bb_events", "imu_bb_div",
                 "imu_bb_rest_prev", "imu_bb_frozen"):
        ocd(["halt", f"mww 0x{t[name]:08X} 0", "resume"])
    ocd(["halt", f"mww 0x{t['imu_bb_max_dyaw']:08X} 0", "resume"])
    ocd(["halt", f"mww 0x{t['imu_bb_last_yaw']:08X} 0", "resume"])
    ocd(["halt", f"mww 0x{t['imu_bb_enable']:08X} 1", "resume"])
    print("blackbox reset (enable=1, frozen=0)")


def do_dump():
    t = symtab()
    base = t["imu_bb_rows"]
    total = read_one(t["imu_bb_total"])
    head = read_one(t["imu_bb_head"])
    words = read_words(base, ROWS * ROW_BYTES // 4)
    if len(words) < ROWS * 5:
        print(f"WARN: got {len(words)} words, expected {ROWS*5}")
    n_rows = min(total, ROWS)
    start = (head - n_rows) % ROWS if head >= n_rows else (head - n_rows) % ROWS
    print(f"total={total} head={head} rows={n_rows}")
    print(f"{'#':>4} {'tick':>6} {'yaw':>9} {'dyaw':>8} {'bias_z':>10} "
          f"{'rest_t':>7} {'R':>1} {'F':>1}")
    for k in range(n_rows):
        idx = (start + k) % ROWS
        blk = words[idx * 5: idx * 5 + 5]
        if len(blk) < 5:
            continue
        r = row_from(blk)
        mark = ""
        if r["flags"] & 0x01:
            mark += " REST-JUMP"
        if r["flags"] & 0x02:
            mark += " DYAW-ALARM"
        print(f"{k:>4} {r['tick']:>6} {r['yaw']:>9.3f} {r['dyaw']:>8.3f} "
              f"{r['bias_z']:>10.6f} {r['rest_t']:>7.3f} {r['rest']:>1} "
              f"{r['flags']:>1}{mark}")


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    {"flash": do_flash, "reset": do_reset, "dump": do_dump,
     "status": do_status}[cmd]()


if __name__ == "__main__":
    main()
