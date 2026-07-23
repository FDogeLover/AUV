"""replay_pole_tracker.py 里合成数据生成逻辑的单元测试。

运行：cd drone_control/tools && python -m pytest test_replay_pole_tracker.py -v
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from replay_pole_tracker import hit_probability, resample_trajectory


class TestHitProbability:
    def test_known_table_points(self):
        assert hit_probability(0.70) == pytest.approx(0.90, abs=1e-6)
        assert hit_probability(1.00) == pytest.approx(0.70, abs=1e-6)
        assert hit_probability(1.55) == pytest.approx(0.10, abs=1e-6)

    def test_linear_interpolation_midpoint(self):
        # 0.70~1.00m 之间线性插值，0.85m 是中点
        assert hit_probability(0.85) == pytest.approx(0.80, abs=1e-6)

    def test_clamped_outside_known_range(self):
        assert hit_probability(0.30) == pytest.approx(0.90, abs=1e-6)  # 比0.70近，钳位到0.70的值
        assert hit_probability(2.50) == pytest.approx(0.10, abs=1e-6)  # 比1.55远，钳位到1.55的值


class TestResampleTrajectory:
    def test_keeps_first_frame_and_respects_interval(self):
        # 原始记录间隔0.1s，按0.5s重采样，应该只保留 t=0,0.5,1.0 附近的帧(贪心：
        # 保留每个>=上一个保留帧+interval的第一帧)
        traj = [(round(i * 0.1, 2), float(i), 0.0, 0.0) for i in range(11)]  # t=0.0..1.0
        resampled = resample_trajectory(traj, interval_s=0.5)
        kept_ts = [t for t, x, y, yaw in resampled]
        assert kept_ts[0] == 0.0
        for t1, t2 in zip(kept_ts, kept_ts[1:]):
            assert t2 - t1 >= 0.5 - 1e-9

    def test_real_log_interval_much_faster_than_poll_interval(self):
        # 真实飞行日志采样间隔~65ms，远快于PoleTracker真机轮询的0.5s，
        # 这正是本次发现的bug根源：不重采样的话tracker窗口只覆盖零点几秒，
        # 完全测不出真实场景里的方位角摆动
        fast_traj = [(round(i * 0.065, 3), 0.0, 0.0, 0.0) for i in range(20)]  # ~1.3s范围
        resampled = resample_trajectory(fast_traj, interval_s=0.5)
        assert len(resampled) < len(fast_traj)
