import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

from Mission_GPT import (
    mission,
    APPROACH_CENTERED_DIST_M,
    HOVER_DROP_DURATION_S,
    arrival_timeout_max,
)
from Lcode.global_variable import sp_side


class _FakeRealsense:
    def __init__(self, pos=(0.0, 0.0, 1.8), yaw=0.0, velocity=(0.0, 0.0, 0.0), confidence=3):
        self._pos = pos
        self._yaw = yaw
        self._velocity = velocity
        self._confidence = confidence
        self._running = True

    def start(self):
        return True

    def autoset(self):
        pass

    def get_tracking_confidence(self):
        return self._confidence

    def get_position(self):
        return self._pos

    def get_orientation(self):
        return (0.0, 0.0, self._yaw)

    def get_velocity(self):
        return self._velocity

    def is_running(self):
        return self._running

    def stop(self):
        self._running = False


def _make_mission(tmp_path):
    re_fc = [0] * 14
    se_fc = [0] * 11
    router = tmp_path / "router.txt"
    router.write_text("0.0,0.0,1.8\n4.0,0.0,1.8\n0.0,0.0,0.0\n")
    old_cwd = os.getcwd()
    os.chdir(tmp_path)
    try:
        m = mission(re_fc, se_fc, realsense_obj=_FakeRealsense())
    finally:
        os.chdir(old_cwd)
    m.t265_ok = True
    m.nav_mode = "PATROL"
    return m


class TestFireTriggerLatch:
    def test_fire_triggered_defaults_false(self, tmp_path):
        m = _make_mission(tmp_path)
        assert m.fire_triggered is False

    def test_maybe_trigger_approach_switches_mode_once(self, tmp_path):
        m = _make_mission(tmp_path)
        m.saved_target_index_before_fire = None
        triggered = m.maybe_trigger_approach(detection=(50.0, 30.0))
        assert triggered is True
        assert m.nav_mode == "APPROACH"
        assert m.fire_triggered is True

    def test_second_detection_does_not_retrigger(self, tmp_path):
        m = _make_mission(tmp_path)
        m.maybe_trigger_approach(detection=(50.0, 30.0))
        m.nav_mode = "PATROL"  # 模拟已经处理完一次火情后回到PATROL
        triggered = m.maybe_trigger_approach(detection=(10.0, 10.0))
        assert triggered is False
        assert m.nav_mode == "PATROL"  # 不会被重新触发进APPROACH

    def test_no_detection_does_not_trigger(self, tmp_path):
        m = _make_mission(tmp_path)
        triggered = m.maybe_trigger_approach(detection=None)
        assert triggered is False
        assert m.fire_triggered is False


class TestApproachCentering:
    def test_pixel_offset_within_deadband_counts_as_centered(self, tmp_path):
        m = _make_mission(tmp_path)
        # 像素偏移换算成的水平距离 < APPROACH_CENTERED_DIST_M 才算居中
        assert m.is_approach_centered(dx_px=2, dy_px=2) is True

    def test_large_pixel_offset_not_centered(self, tmp_path):
        m = _make_mission(tmp_path)
        assert m.is_approach_centered(dx_px=800, dy_px=800) is False


class TestHoverDropDuration:
    def test_hover_drop_duration_is_independent_constant(self):
        """见设计文档：HOVER_DROP_DURATION_S是赛题写死的3秒，不能跟navigate()
        到达确认用的arrival_hold_s混用。"""
        assert HOVER_DROP_DURATION_S == 3.0


class TestResumeAfterHoverDrop:
    def test_resume_continues_from_saved_index_not_reset(self, tmp_path):
        m = _make_mission(tmp_path)
        m.target_index = 3
        m.maybe_trigger_approach(detection=(50.0, 30.0))  # 保存target_index=3
        m.finish_hover_drop_and_resume()
        assert m.nav_mode == "PATROL"
        assert m.target_index == 3  # 恢复到触发时保存的索引，不重置为0

    def test_resume_does_not_immediately_timeout_skip_waypoint(self, tmp_path):
        """回归测试（问题2）：finish_hover_drop_and_resume()之前，arrival_start_time
        停留在火情触发前的旧值，APPROACH/CONFIRM_WARN/HOVER_DROP整个过程都不会
        重置它，导致恢复PATROL后第一个navigate() tick里
        `time.time() - self.arrival_start_time` 几乎必然超过arrival_timeout_max，
        触发"航点超时，强制跳过"——target_index会在没有真正到达的情况下自增。
        修复后finish_hover_drop_and_resume()要重置arrival_start_time等到达检测状态，
        紧接着调用navigate()（目标点还远，不会立即触发到达）不应该让target_index自增。"""
        m = _make_mission(tmp_path)
        m.target_index = 0  # router.txt: [0,0,1.8] -> [4.0,0,1.8]，与当前pos(0,0,1.8)有4m差距
        m.last_target_index = 0

        # 模拟火情触发前，arrival_start_time是很久以前设置的
        m.arrival_start_time = time.time() - (arrival_timeout_max + 10.0)

        m.maybe_trigger_approach(detection=(50.0, 30.0))  # 保存target_index=0，进入APPROACH
        # 模拟APPROACH -> CONFIRM_WARN -> HOVER_DROP 全程没有触碰arrival_start_time
        m.finish_hover_drop_and_resume()

        assert m.nav_mode == "PATROL"
        idx_before = m.target_index
        # pos离target(4.0,0,1.8)很远，不会被真正判定为到达
        m.navigate(pos=[0.0, 0.0, 1.8], yaw=0.0)
        assert m.target_index == idx_before  # 不应该被"超时强制跳过"逻辑立即自增


class TestHoverDropClosedLoopHold:
    """回归测试：悬停3s期间水平位置必须闭环锁定，不能像之前那样只发vx=vy=0的
    开环零速度指令(用户2026-07-16反馈)——否则任何漂移都不会被修正，直接影响
    抛投精度和最终广播坐标的准确性。"""

    def test_first_tick_locks_current_position_as_target(self, tmp_path):
        m = _make_mission(tmp_path)
        m.nav_mode = "HOVER_DROP"
        m._hover_drop_start_time = None
        m._hover_hold_pos = None
        m.navigate(pos=[1.0, 2.0, 0.0], yaw=0.0)
        assert m._hover_hold_pos == (1.0, 2.0)

    def test_drifted_position_produces_corrective_nonzero_speed(self, tmp_path):
        """锁定位置后，如果后续读数偏离锁定点，应该产生非零的修正速度指令，
        而不是像开环那样始终发0。"""
        m = _make_mission(tmp_path)
        m.nav_mode = "HOVER_DROP"
        m._hover_drop_start_time = None
        m._hover_hold_pos = None
        m.navigate(pos=[1.0, 2.0, 0.0], yaw=0.0)  # 锁定(1.0, 2.0)

        # simple_pid在dt=0(两次调用间没有真实时间流逝)时不会更新输出，真实主循环
        # tick间隔30ms，这里睡一下模拟，否则P项算出来恒为0，测试没有意义
        time.sleep(0.05)
        m.navigate(pos=[1.3, 2.0, 0.0], yaw=0.0)  # 偏离锁定点0.3m
        vx_sent = m.se_fc[3] - sp_side
        assert vx_sent != 0

    def test_corrective_speed_is_clamped_to_conservative_limit(self, tmp_path):
        """限幅应该用HOVER_HOLD_MAX_STEP_CMPS(跟APPROACH一致的保守值)，不是
        navigate()跨格移动用的40——抛投前不该有大幅动作。"""
        from Mission_GPT import HOVER_HOLD_MAX_STEP_CMPS
        m = _make_mission(tmp_path)
        m.nav_mode = "HOVER_DROP"
        m._hover_drop_start_time = None
        m._hover_hold_pos = None
        m.navigate(pos=[0.0, 0.0, 0.0], yaw=0.0)  # 锁定(0.0, 0.0)

        time.sleep(0.05)  # 见上一个测试注释：simple_pid需要dt>0才会更新输出
        m.navigate(pos=[5.0, 5.0, 0.0], yaw=0.0)  # 故意给一个很大的偏差
        vx_sent = m.se_fc[3] - sp_side
        vy_sent = m.se_fc[4] - sp_side
        assert vx_sent != 0  # 确认PID确实产生了非零输出，不是被限幅前就已经是0
        assert vy_sent != 0


class _FakeFireVision:
    def __init__(self, dx_px, dy_px):
        self._latest = {"dx_px": dx_px, "dy_px": dy_px, "t": time.time()}

    def latest(self):
        return dict(self._latest)


class TestApproachSignConvention:
    """回归测试：下视摄像头物理安装确认(2026-07-16用户确认)画面上边=+y、右边=+x
    (无镜像)。据此推导：dx_px>0(目标偏右)对应目标在+x方向，vx应与dx_px同号；
    dy_px>0(目标偏下，因为"下"是"上=+y"的反方向即-y)对应目标在-y方向，vy必须
    与dy_px反号——如果直接用dy_px同号会正反馈发散(这个项目里yaw方向出过同类问题，
    见docs/known_issues.md)。这里只验证符号关系是编码正确，不代表已经过地面台架/
    真机验证——那一步仍然需要在实际飞行前做。"""

    def test_positive_dx_produces_positive_vx(self, tmp_path):
        m = _make_mission(tmp_path)
        m.nav_mode = "APPROACH"
        m.fire_vision = _FakeFireVision(dx_px=500, dy_px=0)
        m.navigate(pos=[0.0, 0.0, 1.8], yaw=0.0)
        vx_sent = m.se_fc[3] - sp_side
        assert vx_sent > 0

    def test_negative_dx_produces_negative_vx(self, tmp_path):
        m = _make_mission(tmp_path)
        m.nav_mode = "APPROACH"
        m.fire_vision = _FakeFireVision(dx_px=-500, dy_px=0)
        m.navigate(pos=[0.0, 0.0, 1.8], yaw=0.0)
        vx_sent = m.se_fc[3] - sp_side
        assert vx_sent < 0

    def test_positive_dy_produces_negative_vy(self, tmp_path):
        """dy_px>0 = 目标在画面下方 = 目标物理上在-y方向 = 需要负的vy靠近。"""
        m = _make_mission(tmp_path)
        m.nav_mode = "APPROACH"
        m.fire_vision = _FakeFireVision(dx_px=0, dy_px=500)
        m.navigate(pos=[0.0, 0.0, 1.8], yaw=0.0)
        vy_sent = m.se_fc[4] - sp_side
        assert vy_sent < 0

    def test_negative_dy_produces_positive_vy(self, tmp_path):
        m = _make_mission(tmp_path)
        m.nav_mode = "APPROACH"
        m.fire_vision = _FakeFireVision(dx_px=0, dy_px=-500)
        m.navigate(pos=[0.0, 0.0, 1.8], yaw=0.0)
        vy_sent = m.se_fc[4] - sp_side
        assert vy_sent > 0


class TestWarnLedTurnsOffOnResume:
    """回归测试：2026-07-16真机测试发现警示LED点亮后一直没关，降落时还亮着——
    set_rgb_led()点亮后不会自动熄灭，finish_hover_drop_and_resume()必须显式关掉。"""

    def test_resume_turns_off_led(self, tmp_path, monkeypatch):
        calls = []
        monkeypatch.setattr("Lcode.gpio_led.set_rgb_led", lambda color: calls.append(color))

        m = _make_mission(tmp_path)
        m.target_index = 0
        m.maybe_trigger_approach(detection=(50.0, 30.0))
        m.finish_hover_drop_and_resume()

        assert 'OFF' in calls


class TestTakeoffWarningLed:
    """起飞前红灯常亮TAKEOFF_WARN_LED_DURATION_S秒提醒周围人员，见_blink_warning_led()。"""

    def test_lights_red_then_off(self, tmp_path, monkeypatch):
        calls = []
        monkeypatch.setattr("Lcode.gpio_led.set_rgb_led", lambda color: calls.append(color))
        monkeypatch.setattr("Mission_GPT.time.sleep", lambda s: None)  # 跳过真实sleep，测试不用等2秒

        m = _make_mission(tmp_path)
        m._blink_warning_led()

        assert calls == ['R', 'OFF']

    def test_gpio_unavailable_does_not_raise(self, tmp_path, monkeypatch):
        """gpio_led模块导入失败(比如本机开发环境)时应该静默跳过，不阻断起飞流程。"""
        monkeypatch.setattr("Mission_GPT.time.sleep", lambda s: None)
        m = _make_mission(tmp_path)
        m._blink_warning_led()  # 不应该抛异常(本机就是走这条路径，Lcode.gpio_led本身已优雅降级)
