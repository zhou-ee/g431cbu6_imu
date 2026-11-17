"""一次性更新 _bb_rec.py 的字段布局（加装 rest_gyr_lp / rest_acc_lp 后重新校准）。"""
from pathlib import Path

P = Path(r"E:\Software\Develop\Embeded\Pack\lsm6dsv_VQF\IMU_Module\_bb_rec.py")
src = P.read_text(encoding="utf-8")

OLD_START = "BASE = 0x2000569C"
OLD_END = "}\n\n\ndef symtab"
i = src.find(OLD_START)
j = src.find(OLD_END)
if i < 0 or j < 0:
    raise SystemExit("layout block not found")

NEW = '''BASE = 0x2000569C
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
'''
src = src[:i] + NEW + src[j:]

# 替换 rows.append 块里的字段映射
OLD_ROW_A = '                "roll": f32(a[IDX["roll"]]),'
k = src.find(OLD_ROW_A)
if k < 0:
    raise SystemExit("row block not found")
k2 = src.find('            })', k)
NEW_ROW = '''                "yaw": f32(a[IDX["yaw"]]),
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
'''
src = src[:k] + NEW_ROW + src[k2:]
P.write_text(src, encoding="utf-8")
print("updated; rest_lp_z present:", "rest_lp_z" in src)
