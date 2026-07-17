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

    def get_raw_imu(self):
        return (0.0, 0.0, 9.8, 0.0, 0.0, 0.0)

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


def _make_mission_4wp(tmp_path):
    """4航点版本，专供TestCruiseVsPrecisionArrivalThreshold：
    PRECISION_HEAD_WAYPOINTS=1(idx=0) + PRECISION_TAIL_WAYPOINTS=2(idx=2/3)
    精确，idx=1是唯一的巡航(掠过式)航点。"""
    re_fc = [0] * 14
    se_fc = [0] * 11
    router = tmp_path / "router.txt"
    router.write_text("0.0,0.0,1.8\n2.0,0.0,1.8\n4.0,0.0,1.8\n0.0,0.0,0.0\n")
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

    def test_trigger_saves_debug_snapshot(self, tmp_path):
        """2026-07-16新增：火情触发时应该调用fire_vision.save_snapshot()存一张
        现场画面，方便事后调试分析。"""
        class _FakeFireVisionWithSnapshot:
            def __init__(self):
                self.snapshot_calls = []

            def save_snapshot(self, reason="fire_triggered"):
                self.snapshot_calls.append(reason)
                return "/fake/path.jpg"

        m = _make_mission(tmp_path)
        m.fire_vision = _FakeFireVisionWithSnapshot()
        m.maybe_trigger_approach(detection=(50.0, 30.0))

        assert m.fire_vision.snapshot_calls == ["fire_triggered"]


class _FakeFireVisionNoDetection:
    def __init__(self):
        self.snapshot_calls = []

    def latest(self):
        return {"dx_px": None, "dy_px": None, "t": 0.0}

    def save_snapshot(self, reason="fire_triggered"):
        self.snapshot_calls.append(reason)
        return "/fake/path.jpg"


class TestPatrolMissSnapshot:
    """2026-07-16新增：PATROL态检测不到火源时，每隔
    PATROL_MISS_SNAPSHOT_INTERVAL_S也存一张画面，跟触发时存的清晰图对比，
    验证运动模糊是不是巡航中检测不到的原因。"""

    def test_saves_snapshot_when_interval_elapsed(self, tmp_path):
        m = _make_mission(tmp_path)
        m.fire_vision = _FakeFireVisionNoDetection()
        m._last_miss_snapshot_time = 0.0  # 模拟"已经很久没存过"，节流窗口已过
        m.navigate(pos=[0.0, 0.0, 1.8], yaw=0.0)
        assert m.fire_vision.snapshot_calls == ["patrol_no_detect"]

    def test_does_not_save_again_within_throttle_window(self, tmp_path):
        m = _make_mission(tmp_path)
        m.fire_vision = _FakeFireVisionNoDetection()
        m._last_miss_snapshot_time = time.time()  # 刚存过，还在节流窗口内
        m.navigate(pos=[0.0, 0.0, 1.8], yaw=0.0)
        assert m.fire_vision.snapshot_calls == []

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
        m.target_index = 0  # router.txt idx0目标是[0,0,1.8]
        m.last_target_index = 0

        # 模拟火情触发前，arrival_start_time是很久以前设置的
        m.arrival_start_time = time.time() - (arrival_timeout_max + 10.0)

        m.maybe_trigger_approach(detection=(50.0, 30.0))  # 保存target_index=0，进入APPROACH
        # 模拟APPROACH -> CONFIRM_WARN -> HOVER_DROP 全程没有触碰arrival_start_time
        m.finish_hover_drop_and_resume()

        assert m.nav_mode == "PATROL"
        idx_before = m.target_index
        # pos离target[0,0,1.8]故意给3米偏差(超过CRUISE_XY_THRESH)，既不会被"掠过式"
        # 巡航判定立即通过，也验证不会被"超时强制跳过"逻辑立即自增
        m.navigate(pos=[3.0, 0.0, 1.8], yaw=0.0)
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


class TestFireDetectHeightGate:
    """2026-07-17新增：低于FIRE_DETECT_MIN_HEIGHT_M不触发火情检测——真机测试
    发现起飞/降落阶段下视画面里出现的小红点，其实是无人机自己下视激光测距的
    反射光点，不是真实火源(之前误以为是"灯罩"本身)。近地时这个激光点在HSV/
    形状上跟真实火源难以区分，加高度门槛避免起飞/降落阶段被自己的激光误触发。"""

    def test_detection_below_height_threshold_does_not_trigger(self, tmp_path):
        m = _make_mission(tmp_path)
        m.fire_vision = _FakeFireVision(dx_px=50.0, dy_px=30.0)
        m.navigate(pos=[0.0, 0.0, 0.5], yaw=0.0)  # 0.5m < FIRE_DETECT_MIN_HEIGHT_M(1.0m)
        assert m.nav_mode == "PATROL"
        assert m.fire_triggered is False

    def test_detection_above_height_threshold_triggers(self, tmp_path):
        m = _make_mission(tmp_path)
        m.fire_vision = _FakeFireVision(dx_px=50.0, dy_px=30.0)
        m.navigate(pos=[0.0, 0.0, 1.5], yaw=0.0)  # 1.5m >= FIRE_DETECT_MIN_HEIGHT_M
        assert m.nav_mode == "APPROACH"
        assert m.fire_triggered is True

    def test_detection_exactly_at_threshold_triggers(self, tmp_path):
        """边界值本身应该算达标(>=不是>)，不多留隐藏的0.01m盲区。"""
        from Mission_GPT import FIRE_DETECT_MIN_HEIGHT_M
        m = _make_mission(tmp_path)
        m.fire_vision = _FakeFireVision(dx_px=50.0, dy_px=30.0)
        m.navigate(pos=[0.0, 0.0, FIRE_DETECT_MIN_HEIGHT_M], yaw=0.0)
        assert m.nav_mode == "APPROACH"


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


class _FakeFireVisionStale:
    """latest()始终返回同一份固定时间戳的旧数据，模拟视觉线程卡死后
    get_frame()返回冻结帧的场景(不随每次调用推进t)。"""
    def __init__(self, dx_px, dy_px, t):
        self._latest = {"dx_px": dx_px, "dy_px": dy_px, "t": t}

    def latest(self):
        return dict(self._latest)


class TestApproachStalenessGuard:
    """2026-07-17新增：PATROL分支触发APPROACH前会检查latest()新鲜度
    (FIRE_VISION_STALE_S)，但_do_approach()本身伺服修正之前没做同样检查——
    如果视觉线程卡死、latest()一直返回同一份冻结坐标，_do_approach()会
    持续伺服到不再代表真实目标位置的点上。修复后应该跟"没有检测"一样处理：
    悬停等待(vx=vy=0)，不主动修正。"""

    def test_stale_detection_does_not_produce_corrective_speed(self, tmp_path):
        from Mission_GPT import FIRE_VISION_STALE_S
        m = _make_mission(tmp_path)
        m.nav_mode = "APPROACH"
        # t设成远早于FIRE_VISION_STALE_S的过去时刻，模拟冻结的旧检测
        stale_t = time.time() - FIRE_VISION_STALE_S - 1.0
        m.fire_vision = _FakeFireVisionStale(dx_px=500, dy_px=0, t=stale_t)

        m.navigate(pos=[0.0, 0.0, 1.8], yaw=0.0)

        vx_sent = m.se_fc[3] - sp_side
        vy_sent = m.se_fc[4] - sp_side
        assert vx_sent == 0
        assert vy_sent == 0

    def test_fresh_detection_still_produces_corrective_speed(self, tmp_path):
        """回归守卫：新鲜检测不应该被这次改动误伤。"""
        m = _make_mission(tmp_path)
        m.nav_mode = "APPROACH"
        m.fire_vision = _FakeFireVisionStale(dx_px=500, dy_px=0, t=time.time())

        m.navigate(pos=[0.0, 0.0, 1.8], yaw=0.0)

        vx_sent = m.se_fc[3] - sp_side
        assert vx_sent > 0


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


class TestCruiseVsPrecisionArrivalThreshold:
    """2026-07-16用户反馈：巡航航点(弓字形中间点)不需要精确到达，只有最后
    PRECISION_TAIL_WAYPOINTS个航点(为land()做准备)才要严格精度。测试用
    _make_mission_4wp的4个航点(idx 0/1/2/3)：PRECISION_HEAD_WAYPOINTS=1让
    idx=0是精确航点(起飞后原地爬升到巡航高度)，PRECISION_TAIL_WAYPOINTS=2让
    idx=2/3是精确航点(为land()做准备)，idx=1是唯一的巡航航点(宽松阈值)。

    2026-07-16进一步改成"掠过式"：巡航航点一旦水平位置进入CRUISE_XY_THRESH
    范围就立刻切下一个目标(target_index自增)，不再走滑动窗口确认+停留观察，
    也就不再往_arrival_window里追加——测试改成直接看target_index有没有自增。

    2026-07-17新增PRECISION_HEAD_WAYPOINTS=1：真机测试实测到起飞后第一个
    航点(原点,巡航高度)被当成巡航航点掠过——起点水平位置本来就已经达标(还
    没开始移动)，只看xy不看z的掠过式逻辑会在高度还只有起飞离地高度时就判定
    "到达"并立刻推进，导致爬升和水平移动同时发生，而不是先原地爬升到位再
    移动。改成idx=0也走精确航点的严格流程(要求z一并收敛)后修复。"""

    def test_head_waypoint_requires_z_convergence_not_just_xy(self, tmp_path):
        """2026-07-17新增：idx=0是精确(头部)航点，xy已经达标但z还差得远
        (0.27m vs 目标1.8m，模拟起飞刚离地的情况)时不应该掠过——这正是
        真机测试实测到的"边爬升边平移"问题，验证z收敛前不会推进航点。"""
        m = _make_mission_4wp(tmp_path)
        m.target_index = 0
        m.last_target_index = 0
        m.arrival_start_time = time.time()
        m.navigate(pos=[0.0, 0.0, 0.27], yaw=0.0)
        assert m.target_index == 0  # 不应该掠过，z还没收敛
        assert m._arrival_window[-1] is False

    def test_cruise_waypoint_flies_through_on_loose_offset(self, tmp_path):
        """idx=1是唯一的巡航航点：0.25m偏差超过严格阈值(confidence=3时0.10m)但在
        CRUISE_XY_THRESH(0.4m)以内，应该单帧内立刻掠过(target_index自增)，
        不需要多帧确认、不需要停留。"""
        m = _make_mission_4wp(tmp_path)
        m.target_index = 1
        m.last_target_index = 1
        m.navigate(pos=[2.25, 0.0, 1.8], yaw=0.0)
        assert m.target_index == 2

    def test_cruise_waypoint_does_not_advance_when_far(self, tmp_path):
        m = _make_mission_4wp(tmp_path)
        m.target_index = 1
        m.last_target_index = 1
        m.arrival_start_time = time.time()  # 避免__init__默认值0.0被当成"早就超时"
        m.navigate(pos=[4.0, 2.0, 1.8], yaw=0.0)
        assert m.target_index == 1

    def test_precision_waypoint_still_requires_confirm_window(self, tmp_path):
        """同样0.25m偏差，在最后的精确航点(idx=2)不应该立刻掠过，还是要走
        滑动窗口确认+停留观察那一套，证明精度分级依然按航点索引区分。"""
        m = _make_mission_4wp(tmp_path)
        m.target_index = 2
        m.last_target_index = 2
        m.arrival_start_time = time.time()  # 避免__init__默认值0.0被当成"早就超时"
        m.navigate(pos=[4.25, 0.0, 1.8], yaw=0.0)
        assert m.target_index == 2  # 单帧不会立刻掠过
        assert m._arrival_window[-1] is False  # 精确航点仍然用严格阈值判定这一帧
