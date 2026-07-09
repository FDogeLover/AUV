"""离线分析转盘纯旋转测试采集的 T265 数据（t265_turntable_log.py 的输出）。

思路：T265 固定在转盘外围、偏离转轴中心一段距离 r。如果转盘只做纯旋转（轴心不动），
T265 自身位置理论上应该精确落在一个以转轴为圆心、半径为 r 的圆上——不需要转盘有
精确角度刻度，只要拟合出这个圆，看每个采样点离圆的偏差（残差）大小即可：
残差小且均匀 = T265 在纯旋转下位置追踪内部自洽；残差大或和转动速度相关 = 旋转时
VIO 追踪本身有问题（这对真实飞行中任何带 yaw 的动作都有参考意义）。

用法:
  python analyze_turntable_rotation.py <turntable_log_xxx.jsonl> [--min-confidence 2]
"""
import argparse
import json
import math


def load_samples(path, min_confidence):
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
            if rec.get("confidence", 0) < min_confidence:
                continue
            samples.append(rec)
    return samples


def fit_circle_kasa(points):
    """代数最小二乘圆拟合(Kasa方法)，纯stdlib。points: [(x,y), ...] -> (cx, cy, r)"""
    n = len(points)
    sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
    for x, y in points:
        z = x * x + y * y
        sx += x
        sy += y
        sxx += x * x
        syy += y * y
        sxy += x * y
        sxz += x * z
        syz += y * z
        sz += z

    # 正规方程组: [sxx sxy sx; sxy syy sy; sx sy n] * [D E F]^T = [-sxz -syz -sz]^T
    a11, a12, a13 = sxx, sxy, sx
    a21, a22, a23 = sxy, syy, sy
    a31, a32, a33 = sx, sy, n
    b1, b2, b3 = -sxz, -syz, -sz

    det = (a11 * (a22 * a33 - a23 * a32)
           - a12 * (a21 * a33 - a23 * a31)
           + a13 * (a21 * a32 - a22 * a31))
    if abs(det) < 1e-9:
        raise ValueError("采样点共线或数量不足，无法拟合圆")

    def cramer(col):
        m = [[a11, a12, a13], [a21, a22, a23], [a31, a32, a33]]
        b = [b1, b2, b3]
        m2 = [row[:] for row in m]
        for i in range(3):
            m2[i][col] = b[i]
        d = (m2[0][0] * (m2[1][1] * m2[2][2] - m2[1][2] * m2[2][1])
             - m2[0][1] * (m2[1][0] * m2[2][2] - m2[1][2] * m2[2][0])
             + m2[0][2] * (m2[1][0] * m2[2][1] - m2[1][1] * m2[2][0]))
        return d / det

    D, E, F = cramer(0), cramer(1), cramer(2)
    cx, cy = -D / 2.0, -E / 2.0
    r = math.sqrt(max(cx * cx + cy * cy - F, 0.0))
    return cx, cy, r


def pearson(xs, ys):
    n = len(xs)
    if n < 2:
        return 0.0
    mx, my = sum(xs) / n, sum(ys) / n
    cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    vx = sum((x - mx) ** 2 for x in xs)
    vy = sum((y - my) ** 2 for y in ys)
    if vx <= 0 or vy <= 0:
        return 0.0
    return cov / math.sqrt(vx * vy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile")
    ap.add_argument("--min-confidence", type=int, default=2)
    args = ap.parse_args()

    samples = load_samples(args.logfile, args.min_confidence)
    print(f"读取 {len(samples)} 条记录（confidence>={args.min_confidence}）")
    if len(samples) < 10:
        print("样本太少，无法分析")
        return

    yaw_range = max(s["yaw_deg"] for s in samples) - min(s["yaw_deg"] for s in samples)
    print(f"yaw覆盖范围：{yaw_range:.1f}°")
    if yaw_range < 150.0:
        print(f"警告：角度覆盖只有{yaw_range:.1f}°，明显偏小。圆拟合在弧段较短时"
              "曲率约束很弱，拟合出的圆心/半径可能严重失真（实测覆盖<60°时半径"
              "误差可达10%+，覆盖300°时误差<1mm）。以下结果仅供参考，建议重新"
              "采集、让转动总角度跨度覆盖到180°以上再下结论。")
        print()

    points = [(s["x"], s["y"]) for s in samples]
    cx, cy, r = fit_circle_kasa(points)
    print(f"拟合圆心=({cx:.4f}, {cy:.4f})m，半径={r:.4f}m")

    residuals = [math.hypot(s["x"] - cx, s["y"] - cy) - r for s in samples]
    abs_res = [abs(v) for v in residuals]
    mean_res = sum(abs_res) / len(abs_res)
    max_res = max(abs_res)
    print(f"残差(离拟合圆的距离)：均值={mean_res*100:.2f}cm，最大={max_res*100:.2f}cm")

    # 角速度只在相邻样本间有定义，长度天然比 residuals 少1；
    # 用 samples[i]/residuals[i] (i>=1) 和 samples[i-1] 的差分配对，
    # 跳过时间戳异常(dt过小/非正)的相邻对而不是塞入占位值，避免污染相关系数。
    ang_speeds = []
    paired_abs_res = []
    for i in range(1, len(samples)):
        dt = samples[i]["t"] - samples[i - 1]["t"]
        if dt <= 1e-3:
            continue
        dyaw = samples[i]["yaw_deg"] - samples[i - 1]["yaw_deg"]
        while dyaw > 180:
            dyaw -= 360
        while dyaw < -180:
            dyaw += 360
        ang_speeds.append(abs(dyaw / dt))
        paired_abs_res.append(abs_res[i])

    if len(ang_speeds) >= 2:
        corr = pearson(ang_speeds, paired_abs_res)
        print(f"残差与角速度的相关系数：{corr:.3f}（接近0=误差与转速无关/纯静态偏差；"
              f"较高=转得快时追踪误差更大，暗示动态追踪滞后）")
    else:
        print("有效角速度样本不足(<2)，跳过残差-转速相关性分析")

    print()
    print("解读参考：")
    print(f"  - 残差均值远小于半径({r*100:.1f}cm)的10%左右 -> T265纯旋转下位置自洽性良好")
    print("  - 残差明显偏大或有跳变(尤其转动瞬间) -> 旋转时VIO追踪本身不稳，"
          "对真实飞行中任何yaw动作都有参考意义")
    print("  注意：采集端(t265.py)对x/y/z做了低通滤波(alpha=0.15)但yaw未滤波，"
          "残差与角速度的相关性里可能混入滤波滞后本身的贡献，不能单凭这一项"
          "相关系数就断定是VIO追踪问题——需要结合残差是否随时间收敛/发散、"
          "是否只在转动瞬间出现来综合判断。")


if __name__ == "__main__":
    main()
