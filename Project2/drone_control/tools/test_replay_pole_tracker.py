"""replay_pole_tracker.py 里合成数据生成逻辑的单元测试。

运行：cd drone_control/tools && python -m pytest test_replay_pole_tracker.py -v
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from replay_pole_tracker import hit_probability


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
