"""land() 降落等待期间飞行日志记录的单元测试。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic && python -m pytest test_land_logging.py -v
"""
import io
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Mission_GPT import mission


class FakeRealsense:
    """伪造T265接口，land()会在等待循环里自己重新采样，不依赖外部传入的旧值。"""
    def __init__(self, pos, yaw, vel=(0.0, 0.0, 0.0)):
        self._pos = pos
        self._yaw = yaw
        self._vel = vel

    def get_position(self):
        return list(self._pos)

    def get_orientation(self):
        return [0.0, 0.0, self._yaw]

    def get_velocity(self):
        return list(self._vel)


def _make_mission_for_land():
    re_fc = [0] * 14
    se_fc = [0] * 11
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
    m.t265_ok = True
    m.realsense = FakeRealsense(pos=(0.12, -0.05, 0.03), yaw=0.01)
    m.state = "LAND"
    return m


class FakeSerialFcRef:
    """伪造串口对象，只提供 land() 需要读取的激光高度属性。"""
    def __init__(self, laser_height_m, motor_pwm_mask=None):
        self._last_laser_height_cm = laser_height_m
        self.debug_data = {"motor_pwm_mask": motor_pwm_mask} if motor_pwm_mask is not None else {}


class TestLandLogging:
    def test_land_logs_position_when_immediately_unlocked(self):
        """land()原本从触发到确认/超时全程不写任何飞行日志，导致降落物理下降过程
        完全没有位置数据(2026-07-08真机测试发现)。这里验证哪怕最简单的"一进来就
        已经上锁"这种情况，也至少要记录一条日志。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0  # 已经上锁，land()应该立刻确认退出

        m.land()

        m._log_file.seek(0)
        lines = [line for line in m._log_file.readlines() if line.strip()]
        assert len(lines) >= 1
        entry = json.loads(lines[-1])
        assert entry["state"] == "LAND"
        assert entry["pos"][0] == 0.12
        assert entry["pos"][1] == -0.05

    def test_land_logs_laser_height_not_raw_t265_z(self):
        """land()原本自采样时直接用realsense.get_position()的原始Z轴——但loop()
        正常流程里Z轴其实是被激光高度覆盖过的(T265自身Z轴未标定，不是真实高度，
        见 takeoff() 同样直接读 _last_laser_height_cm 而不用T265 Z)。land()这里
        如果继续用原始T265 Z，降落阶段记录的"高度"数据是假的，没法用来验证物理
        降落过程，这正是加这份日志的初衷。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0  # 立刻确认退出，只需要一条日志
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.07)  # 激光真实高度7cm
        # T265原始Z(FakeRealsense pos[2]=0.03)明显不同于激光高度，用来验证没有被误用

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["pos"][2] == 0.07

    def test_land_logs_unlock_sta(self):
        """降落确认依赖的unlock_sta字段本身原来没有写进日志，事后只能靠高度曲线
        间接推断"确认超时"到底是物理没执行还是确认通道滞后。直接记录这个字段能让
        排查更直接。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0  # 已上锁

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["unlock_sta"] == 0

    def test_land_logs_motor_pwm_mask(self):
        """问题7 2026-07-08发现unlock_sta可能假阳性(读到已上锁但电机实际未停转)。
        固件新增了电机PWM非零位掩码字段(帧2)，land()应该把它也记进日志，方便
        下次复现时直接对比unlock_sta和这个掩码是否矛盾。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.07, motor_pwm_mask=0x0F)  # 4个电机都还在转

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["motor_pwm_mask"] == 0x0F

    def test_land_logs_motor_pwm_mask_none_when_unavailable(self):
        """serial_fc_ref没有debug_data(比如刚连上还没收到过帧2)时，字段应该是
        None而不是抛异常。"""
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc[5] = 0
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.07)  # 没传motor_pwm_mask

        m.land()

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["motor_pwm_mask"] is None


class TransientUnlockList(list):
    """模拟re_fc[5](unlock_sta)按预设序列变化，序列用完后保持最后一个值不变。
    用于测试降落确认去抖逻辑，不依赖真实串口数据。"""
    def __init__(self, base, seq):
        super().__init__(base)
        self._seq = seq
        self._calls = 0

    def __getitem__(self, idx):
        if idx == 5:
            call = self._calls
            self._calls += 1
            if call < len(self._seq):
                return self._seq[call]
            return self._seq[-1]
        return super().__getitem__(idx)


class TestLandUnlockDebounce:
    """2026-07-09大范围20点网格测试真机观察到疑似假阳性：终端打印"降落确认：
    已上锁"并退出，但用户确认电机实际没有停转/没有真正降落；飞行日志显示确认
    发生前unlock_sta全程是1，从未记录到0。原逻辑单次读到0就立刻确认退出，容易
    被单帧通信噪声/校验巧合触发误判。这里改成要求连续LAND_UNLOCK_CONFIRM_COUNT
    次都读到0才真正确认。"""

    def test_single_glitch_does_not_confirm(self, monkeypatch):
        import Mission_GPT as mg
        monkeypatch.setattr(mg, "LAND_CONFIRM_TIMEOUT_S", 0.3)  # 加速测试，不用等25秒真实超时
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        # 第3次读到0(瞬间glitch)，之后恢复为1一直到超时
        m.re_fc = TransientUnlockList([0] * 14, seq=[1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])

        logged = []
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))
        monkeypatch.setattr(mg.logger, "warning", lambda msg, *a, **k: logged.append(("warning", msg)))

        m.land()

        assert ("info", "降落确认：已上锁") not in logged
        assert ("warning", "降落确认超时，强制退出") in logged

    def test_confirms_after_consecutive_unlock_reads(self, monkeypatch):
        import Mission_GPT as mg
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        # 前2次还是1(未上锁)，之后持续读到0(真实持续上锁)
        m.re_fc = TransientUnlockList([0] * 14, seq=[1, 1, 0, 0, 0, 0, 0, 0])

        logged = []
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))

        m.land()

        assert ("info", "降落确认：已上锁") in logged
