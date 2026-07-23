"""离线分析 flight_data.jsonl 中 T265 速度与光流融合速度的同轴/交叉轴相关性。

用于排查光流传感器坐标系是否相对机体旋转了约90度（见 CLAUDE.md 已知问题第2条）。
不修改任何飞控/上位机代码，纯只读分析工具。
"""
import argparse
import json
import math
import sys


def load_samples(path, t265_thresh, of_thresh):
    """读取 jsonl 文件，返回 (t265_vx, t265_vy, of_dx, of_dy) 列表，过滤静止噪声样本。"""
    samples = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "t265_vel" not in rec or "of1_vel_cms" not in rec:
                continue
            vx, vy = rec["t265_vel"]
            dx, dy = rec["of1_vel_cms"]
            if (abs(vx) < t265_thresh and abs(vy) < t265_thresh
                    and abs(dx) < of_thresh and abs(dy) < of_thresh):
                continue
            samples.append((vx, vy, dx, dy))
    return samples


def pearson(xs, ys):
    """纯 stdlib Pearson 相关系数。样本数 < 2 或方差为0时返回 0.0。"""
    n = len(xs)
    if n < 2:
        return 0.0
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    cov = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    var_x = sum((x - mean_x) ** 2 for x in xs)
    var_y = sum((y - mean_y) ** 2 for y in ys)
    denom = math.sqrt(var_x * var_y)
    if denom == 0:
        return 0.0
    return cov / denom


def shifted_correlations(samples, shift):
    """对给定位移 shift，返回 (same_axis_strength, cross_axis_strength)。

    shift > 0 表示光流序列相对 T265 序列向后移（延迟）；shift < 0 表示提前。
    """
    n = len(samples)
    t265_vx = [s[0] for s in samples]
    t265_vy = [s[1] for s in samples]
    of_dx = [s[2] for s in samples]
    of_dy = [s[3] for s in samples]

    if shift >= 0:
        t_vx = t265_vx[: n - shift] if shift < n else []
        t_vy = t265_vy[: n - shift] if shift < n else []
        o_dx, o_dy = of_dx[shift:], of_dy[shift:]
    else:
        s = -shift
        t_vx, t_vy = t265_vx[s:], t265_vy[s:]
        o_dx, o_dy = of_dx[: n - s], of_dy[: n - s]

    r_vx_dx = pearson(t_vx, o_dx)
    r_vy_dy = pearson(t_vy, o_dy)
    r_vx_dy = pearson(t_vx, o_dy)
    r_vy_dx = pearson(t_vy, o_dx)

    same_axis = (abs(r_vx_dx) + abs(r_vy_dy)) / 2
    cross_axis = (abs(r_vx_dy) + abs(r_vy_dx)) / 2
    return same_axis, cross_axis


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", default="flight_data.jsonl",
                         help="flight_data.jsonl 路径 (默认: 当前目录下同名文件)")
    parser.add_argument("--t265-thresh", type=float, default=0.02,
                         help="T265速度噪声阈值 m/s (默认 0.02)")
    parser.add_argument("--of-thresh", type=float, default=2.0,
                         help="光流速度噪声阈值 cm/s (默认 2.0)")
    parser.add_argument("--max-shift", type=int, default=5,
                         help="时延扫描范围 ±N 个采样点 (默认 5)")
    args = parser.parse_args()

    samples = load_samples(args.path, args.t265_thresh, args.of_thresh)
    n = len(samples)
    print(f"有效样本数: {n} (过滤阈值: t265>{args.t265_thresh}m/s, of>{args.of_thresh}cm/s)")
    if n < 30:
        print("警告: 有效样本数偏少 (<30)，以下结论仅供参考。\n")
    if n < 2:
        print("样本不足，无法计算相关系数。")
        sys.exit(1)

    print("\n位移(样本) | 同轴强度 | 交叉轴强度")
    best_same = (-1.0, None)
    best_cross = (-1.0, None)
    rows = []
    for shift in range(-args.max_shift, args.max_shift + 1):
        same_axis, cross_axis = shifted_correlations(samples, shift)
        rows.append((shift, same_axis, cross_axis))
        if same_axis > best_same[0]:
            best_same = (same_axis, shift)
        if cross_axis > best_cross[0]:
            best_cross = (cross_axis, shift)

    for shift, same_axis, cross_axis in rows:
        marker = " 0" if shift == 0 else f"{shift:+d}"
        print(f"{marker:>10} | {same_axis:.2f}     | {cross_axis:.2f}")

    same_r, same_shift = best_same
    cross_r, cross_shift = best_cross
    print()
    if cross_r > same_r:
        print(f"结论: 交叉轴相关性峰值 {cross_r:.2f} (位移{cross_shift}) "
              f"> 同轴相关性峰值 {same_r:.2f} (位移{same_shift})，支持约90°旋转假设。")
    else:
        print(f"结论: 同轴相关性峰值 {same_r:.2f} (位移{same_shift}) "
              f">= 交叉轴相关性峰值 {cross_r:.2f} (位移{cross_shift})，不支持90°旋转假设。")


if __name__ == "__main__":
    main()
