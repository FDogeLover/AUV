"""离线回放：用2026-07-07真实杆子测试飞行的T265轨迹(pos/yaw)，按实测命中率合成雷达候选点，
对比旧版(机体系角度/距离容差)和新版(世界系位置聚类) PoleTracker 匹配逻辑能否确认出杆子。

注意：此工具依赖原 basic_radar/Lcode/Lradar.py，该目录已删除，工具暂不可用。
不接真实雷达/飞控，纯离线分析工具。

运行：
    # 原 basic_radar/ 已删除，Lradar.py 不再存在，此工具暂无法运行
    cd drone_control/tools && python replay_pole_tracker.py
"""
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "basic_radar"))  # 原 basic_radar/ 已删除，此 import 会失败

from Lcode.Lradar import PoleTracker, world_to_body_angle_dist  # noqa: E402

FLIGHT_LOG = os.path.join(
    os.path.dirname(__file__),
    "test_data_20260707",
    "flight_data_pole_tracker_real_flight_20260707.jsonl.bak",
)
POLE_WORLD = (-1.0, 0.0)  # 起飞点180°方向1m处，见问题13笔记
YAW_SIGN = 1  # 尚未标定，合成数据生成和新版tracker用同一个假设值，见spec"该脚本不验证的内容"

# 真实飞行日志采样间隔约65ms(~15Hz)，但2026-07-07真机测试时PoleTracker实际轮询是0.5s一次
# (跟真实雷达帧率无关，是当时台架/飞行测试脚本自己的轮询节奏)。不按这个间隔重采样的话，
# window=6的历史窗口只覆盖零点几秒，飞机移动量太小，测不出真实场景里的方位角摆动。
POLL_INTERVAL_S = 0.5

# 2026-07-07台架实测命中率(distance_m, hit_probability)，线性插值，范围外钳位到端点值
HIT_RATE_TABLE = [(0.70, 0.90), (1.00, 0.70), (1.55, 0.10)]

ANGLE_NOISE_SIGMA_DEG = 3.0   # 模拟"混合像素"命中角度跳动
DIST_NOISE_SIGMA_MM = 20.0    # 参考台架测距误差量级(~2cm)


def hit_probability(distance_m):
    if distance_m <= HIT_RATE_TABLE[0][0]:
        return HIT_RATE_TABLE[0][1]
    if distance_m >= HIT_RATE_TABLE[-1][0]:
        return HIT_RATE_TABLE[-1][1]
    for (d0, p0), (d1, p1) in zip(HIT_RATE_TABLE, HIT_RATE_TABLE[1:]):
        if d0 <= distance_m <= d1:
            frac = (distance_m - d0) / (d1 - d0)
            return p0 + frac * (p1 - p0)
    return HIT_RATE_TABLE[-1][1]


def load_trajectory(path):
    """读取jsonl，返回 [(t, x, y, yaw_rad), ...]，只保留含 pos 字段的记录。"""
    traj = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "pos" not in rec:
                continue
            x, y = rec["pos"][0], rec["pos"][1]
            yaw_deg = rec.get("t265_yaw_deg", 0.0)
            traj.append((rec["t"], x, y, math.radians(yaw_deg)))
    return traj


def resample_trajectory(traj, interval_s=POLL_INTERVAL_S):
    """贪心重采样：只保留距离上一个保留帧的时间差 >= interval_s 的帧，模拟真机测试
    时PoleTracker实际的轮询节奏(而不是飞行日志本身高得多的记录频率)。第一帧总是保留。"""
    if not traj:
        return []
    resampled = [traj[0]]
    last_t = traj[0][0]
    for frame in traj[1:]:
        t = frame[0]
        if t - last_t >= interval_s:
            resampled.append(frame)
            last_t = t
    return resampled


def synthesize_candidate(x, y, yaw_rad, rng):
    """按当前位姿和已知杆子世界坐标，掷骰子决定这一帧是否产生命中，命中则返回
    带噪声的机体系候选 (angle_deg, distance_mm)，未命中返回 None。"""
    true_angle, true_dist_mm = world_to_body_angle_dist(
        POLE_WORLD[0], POLE_WORLD[1], x, y, yaw_rad, yaw_sign=YAW_SIGN
    )
    p_hit = hit_probability(true_dist_mm / 1000.0)
    if rng.random() >= p_hit:
        return None
    angle = (true_angle + rng.gauss(0, ANGLE_NOISE_SIGMA_DEG)) % 360.0
    dist_mm = max(1.0, true_dist_mm + rng.gauss(0, DIST_NOISE_SIGMA_MM))
    return angle, dist_mm


class _FrameRadar:
    """喂给 PoleTracker.update() 的一次性假雷达：只有当前这一帧的候选点。"""
    def __init__(self, candidate):
        self._scan = {}
        if candidate is not None:
            angle, dist_mm = candidate
            self._scan[round(angle) % 360] = (dist_mm, 100)

    def get_scan(self):
        return dict(self._scan)


class OldPoleTrackerSim:
    """2026-07-07台架验证版本的匹配逻辑(机体系角度/距离容差)，只为跟新版对比用，
    不是生产代码，复现自重构前的 PoleTracker.confirmed_poles()。"""
    def __init__(self, window=6, min_hits=3, angle_tol_deg=4, dist_tol_mm=150):
        self.min_hits = min_hits
        self.angle_tol_deg = angle_tol_deg
        self.dist_tol_mm = dist_tol_mm
        from collections import deque
        self._history = deque(maxlen=window)

    def update(self, candidate):
        self._history.append([candidate] if candidate is not None else [])

    def confirmed(self):
        all_candidates = [c for frame in self._history for c in frame]
        n = len(all_candidates)
        used = [False] * n
        for i in range(n):
            if used[i]:
                continue
            a1, d1 = all_candidates[i]
            group = [(a1, d1)]
            used[i] = True
            for j in range(i + 1, n):
                if used[j]:
                    continue
                a2, d2 = all_candidates[j]
                if abs(a2 - a1) <= self.angle_tol_deg and abs(d2 - d1) <= self.dist_tol_mm:
                    group.append((a2, d2))
                    used[j] = True
            if len(group) >= self.min_hits:
                return True, len(group)
        return False, 0


def run_replay(seed=42):
    rng = random.Random(seed)
    raw_traj = load_trajectory(FLIGHT_LOG)
    traj = resample_trajectory(raw_traj)

    old_tracker = OldPoleTrackerSim()
    new_tracker = PoleTracker(yaw_sign=YAW_SIGN)

    old_confirmed_timeline = []   # 每帧True/False，是否处于confirmed状态
    new_confirmed_timeline = []
    new_last_result = None

    for idx, (t, x, y, yaw_rad) in enumerate(traj):
        candidate = synthesize_candidate(x, y, yaw_rad, rng)

        old_tracker.update(candidate)
        ok, hits = old_tracker.confirmed()
        old_confirmed_timeline.append(ok)

        new_tracker.update(_FrameRadar(candidate), x, y, yaw_rad)
        confirmed = new_tracker.confirmed_poles()
        new_confirmed_timeline.append(bool(confirmed))
        if confirmed:
            new_last_result = confirmed[0]

    def first_true_idx(timeline):
        for i, v in enumerate(timeline):
            if v:
                return i
        return None

    def dropout_count(timeline, first_idx):
        """首次确认之后，又变回"未确认"的帧数——衡量确认状态稳不稳，
        不是只看"有没有确认过"。"""
        if first_idx is None:
            return None
        return sum(1 for v in timeline[first_idx:] if not v)

    old_first = first_true_idx(old_confirmed_timeline)
    new_first = first_true_idx(new_confirmed_timeline)
    old_dropouts = dropout_count(old_confirmed_timeline, old_first)
    new_dropouts = dropout_count(new_confirmed_timeline, new_first)

    print(f"重采样后帧数(轮询间隔{POLL_INTERVAL_S}s): {len(traj)}（原始日志帧数: "
          f"{len(raw_traj)}）")
    print(f"已知杆子世界坐标(假设): {POLE_WORLD}")
    print("-" * 60)
    print(f"旧版(机体系角度/距离容差): "
          f"首次确认={'第'+str(old_first)+'帧' if old_first is not None else '全程未确认'}, "
          f"首次确认后又丢失确认的帧数={old_dropouts}")
    print(f"新版(世界系位置聚类): "
          f"首次确认={'第'+str(new_first)+'帧' if new_first is not None else '全程未确认'}, "
          f"首次确认后又丢失确认的帧数={new_dropouts}", end="")
    if new_last_result is not None:
        err_x = new_last_result["x"] - POLE_WORLD[0]
        err_y = new_last_result["y"] - POLE_WORLD[1]
        err_m = math.hypot(err_x, err_y)
        print(f"，最后一次确认坐标=({new_last_result['x']:.3f}, {new_last_result['y']:.3f})，"
              f"误差={err_m*100:.1f}cm")
    else:
        print()


if __name__ == "__main__":
    run_replay()
