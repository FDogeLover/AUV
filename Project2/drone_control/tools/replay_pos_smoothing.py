import json
from collections import deque

PATH = "test_data_20260709/flight_data_rect_replay.jsonl"

recs = []
with open(PATH) as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        d = json.loads(line)
        if d.get("state") == "NAVIGATE" and "pos" in d and "target" in d:
            recs.append(d)

xy_thresh = 0.10
NEAR_THRESH = 0.25  # 只看"已经比较接近目标"的帧，过滤掉还在飞过去的转场阶段

for label, near_only in [("全部帧(含转场)", False), ("仅接近目标的帧(dx_raw<0.25 and dy_raw<0.25)", True)]:
    raw_ok = 0
    smooth_ok = 0
    total = 0
    window = deque(maxlen=3)
    last_target = None

    for d in recs:
        pos = d["pos"]
        target = d["target"]
        if target != last_target:
            window.clear()
            last_target = target
        dx = abs(pos[0] - target[0])
        dy = abs(pos[1] - target[1])

        window.append((pos[0], pos[1]))
        avg_x = sum(p[0] for p in window) / len(window)
        avg_y = sum(p[1] for p in window) / len(window)
        dxs = abs(avg_x - target[0])
        dys = abs(avg_y - target[1])

        if near_only and not (dx < NEAR_THRESH and dy < NEAR_THRESH):
            continue

        total += 1
        if dx < xy_thresh and dy < xy_thresh:
            raw_ok += 1
        if dxs < xy_thresh and dys < xy_thresh:
            smooth_ok += 1

    print(f"=== {label} ===")
    print(f"  records: {total}")
    print(f"  raw match rate:      {raw_ok}/{total} = {raw_ok/total*100:.1f}%")
    print(f"  smoothed match rate: {smooth_ok}/{total} = {smooth_ok/total*100:.1f}%")
    print()
