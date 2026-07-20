"""
离线验证 qr_vision.py ROI 路径
--------------------------------
测试 4 条解码路径，对比成功率，诊断 ROI margin 裁掉 finder pattern 的假设。

用法（在项目根目录）：
    python drone_control/tools/analyze_qr_roi_offline.py
"""

import json
import os
import sys
import glob
import math
from pathlib import Path

# ── 路径设置 ──────────────────────────────────────────────
ROOT = Path(__file__).resolve().parents[2]
WI_DIR = ROOT / "drone_control" / "warehouse_inventory"
TEST_DIR = ROOT / "drone_control" / "tools" / "test_data_warehouse_inventory_20260719" / "vision_debug"

sys.path.insert(0, str(WI_DIR))

import numpy as np
import cv2
from pyzbar.pyzbar import decode as pyzbar_decode
from Lcode.qr_vision import QRDecoder, QRMapping


def imread_unicode(path):
    """cv2.imread 不支持含中文的路径，用 np.fromfile 绕过。"""
    buf = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)

# ── 加载映射 ──────────────────────────────────────────────
mapping = QRMapping(WI_DIR / "qr_mapping.txt")
decoder = QRDecoder(mapping)

# ── 收集图片 ──────────────────────────────────────────────
jpg_files = sorted(glob.glob(str(TEST_DIR / "*.jpg")))
print(f"找到 {len(jpg_files)} 张图片\n")

# ── 路径定义 ──────────────────────────────────────────────
# A: 全帧 adaptiveThreshold + pyzbar（基准，之前离线分析确认有效）
# B: _decode_target_roi(laser_aim_px)  ← 真机飞行中实际走的路径
# C: detect(target_point=laser_aim_px) ← 完整修复后路径（B失败→geometry→localized）
# D: detect(target_point=None)         ← 离线/调试路径（包含全帧 tile scan）

results = {"A": [], "B": [], "C": [], "D": []}
roi_fail_details = []  # B 路径失败时的详细信息

for jpg_path in jpg_files:
    json_path = jpg_path.replace(".jpg", ".json")
    meta = {}
    if os.path.exists(json_path):
        meta = json.load(open(json_path))

    frame = imread_unicode(jpg_path)
    if frame is None:
        print(f"  [SKIP] 无法读取: {jpg_path}")
        continue

    h, w = frame.shape[:2]
    laser_aim = meta.get("laser_aim_px")
    target_point = tuple(laser_aim) if laser_aim else None
    state = meta.get("state", "unknown")
    slot = meta.get("slot_label", "?")
    fname = Path(jpg_path).stem

    # ── 路径 A：全帧 adaptiveThreshold + pyzbar ──────────
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    thresh = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 5
    )
    a_results = pyzbar_decode(thresh)
    a_success = any(
        mapping.content_to_number.get(
            r.data.decode("utf-8", errors="replace").strip()
        ) is not None
        for r in a_results
    )
    results["A"].append(a_success)

    # ── 路径 B：_decode_target_roi ──────────────────────
    if target_point is not None:
        b_detection = decoder._decode_target_roi(frame, target_point)
        b_success = b_detection is not None and b_detection.number is not None
    else:
        b_success = None  # 无 laser_aim_px，跳过
    results["B"].append(b_success)

    # 路径 B 失败时：记录 ROI 尺寸 vs. QR 在帧中的像素大小
    if b_success is False:
        roi_w = min(w, max(320, decoder.geometry_roi_width))
        roi_h = min(h, max(320, decoder.geometry_roi_height))
        cx, cy = target_point
        ox = max(0, min(w - roi_w, round(cx - roi_w / 2.0)))
        oy = max(0, min(h - roi_h, round(cy - roi_h / 2.0)))
        # 用全帧 adaptiveThreshold pyzbar 找 QR 真实位置（如果 A 成功）
        qr_in_roi = "N/A"
        qr_center_dist = "N/A"
        if a_success:
            for r in a_results:
                content = r.data.decode("utf-8", errors="replace").strip()
                if mapping.content_to_number.get(content) is not None:
                    rect = r.rect
                    qr_cx = rect.left + rect.width / 2
                    qr_cy = rect.top + rect.height / 2
                    in_roi = (ox <= qr_cx <= ox + roi_w) and (oy <= qr_cy <= oy + roi_h)
                    dist = math.hypot(qr_cx - cx, qr_cy - cy)
                    qr_in_roi = f"{'YES' if in_roi else 'NO'}  QR_center=({qr_cx:.0f},{qr_cy:.0f}) size={rect.width}x{rect.height}px"
                    qr_center_dist = f"{dist:.1f}px from laser_aim"
                    break
        roi_fail_details.append({
            "file": fname,
            "state": state,
            "slot": slot,
            "roi": f"{roi_w}x{roi_h} @ ({ox},{oy})",
            "laser_aim": f"({cx:.0f},{cy:.0f})",
            "qr_in_roi": qr_in_roi,
            "qr_dist": qr_center_dist,
        })

    # ── 路径 C：detect(target_point) ────────────────────
    if target_point is not None:
        c_detection = decoder.detect(frame, target_point)
        c_success = c_detection is not None and c_detection.number is not None
    else:
        c_success = None
    results["C"].append(c_success)

    # ── 路径 D：detect(None) ─────────────────────────────
    d_detection = decoder.detect(frame, None)
    d_success = d_detection is not None and d_detection.number is not None
    results["D"].append(d_success)

# ── 统计 ──────────────────────────────────────────────────
total = len(jpg_files)

def rate(lst):
    valid = [x for x in lst if x is not None]
    if not valid:
        return "N/A"
    ok = sum(1 for x in valid if x)
    return f"{ok}/{len(valid)}  ({100*ok/len(valid):.1f}%)"

print("=" * 60)
print("路径对比（共 %d 张）" % total)
print("=" * 60)
print(f"  A  全帧 adaptiveThreshold + pyzbar（基准）: {rate(results['A'])}")
print(f"  B  _decode_target_roi(laser_aim_px)       : {rate(results['B'])}")
print(f"  C  detect(target_point=laser_aim_px)      : {rate(results['C'])}")
print(f"  D  detect(target_point=None)              : {rate(results['D'])}")

# ── B 路径失败详情 ────────────────────────────────────────
b_failures = [x for x in results["B"] if x is False]
if b_failures:
    print(f"\n── 路径 B 失败帧详情（{len(b_failures)} 帧）──")
    print(f"{'file':<32} {'state':<20} {'slot':<6} {'roi':<22} {'laser_aim':<14} {'QR 是否在 ROI 内'}")
    print("-" * 130)
    for d in roi_fail_details:
        print(f"{d['file']:<32} {d['state']:<20} {d['slot']:<6} {d['roi']:<22} {d['laser_aim']:<14} {d['qr_in_roi']}")
        if d['qr_dist'] != "N/A":
            print(f"  └─ {d['qr_dist']}")
else:
    print("\n路径 B 无失败帧 ✓")

# ── B 成功但 A 失败（即 ROI 比全帧更好？）──────────────────
b_better = sum(
    1 for a, b in zip(results["A"], results["B"])
    if b is True and a is False
)
if b_better:
    print(f"\n⚠  路径 B 成功但 A 失败：{b_better} 帧（ROI 抑制了干扰？）")

# ── B/C 差异（geometry fallback 有没有起效）───────────────
b_fail_c_ok = sum(
    1 for b, c in zip(results["B"], results["C"])
    if b is False and c is True
)
if b_fail_c_ok:
    print(f"\n✓  geometry fallback 救回：{b_fail_c_ok} 帧（B 失败 → C 成功）")
else:
    print(f"\n  geometry fallback 未额外救回任何帧（B 失败时 C 也全部失败）")
