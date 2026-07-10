import simple_pid


class PID:
    def __init__(self, type=0, target=0, p=None, i=None, d=None) -> None:
        self.xyp = 0.7   # 2026-07-10基线复测：Kp=0.85(+20%,未配D)震荡明显(pitch标准差1.44°/落地偏差
                          # 12.5cm)，Kp=0.75+Kd=0.065(联合调整)稳定性接近基线但速度反而更慢(0.137m/s)。
                          # 之前"基线约0.15-0.18m/s"是从矩形路径测试估算的，跟今天两组用的0.5m单轴航线
                          # 不是同一条件，这里用完全相同的航线复测原始0.7/0.05增益，补一个严格可比的基线
        self.xyi = 0.002
        self.xyd = 0.05
        self.yawp = 1.5
        self.yawi = 0.0
        self.yawd = 0.3
        self.xylimit = 40
        self.yawlimit = 30

        if type == 0:
            kp = p if p is not None else self.xyp
            ki = i if i is not None else self.xyi
            kd = d if d is not None else self.xyd
            self.pid = simple_pid.PID(kp, ki, kd, target)
            self.pid.output_limits = (-self.xylimit, self.xylimit)
        else:
            kp = p if p is not None else self.yawp
            ki = i if i is not None else self.yawi
            kd = d if d is not None else self.yawd
            self.pid = simple_pid.PID(kp, ki, kd, target)
            self.pid.output_limits = (-self.yawlimit, self.yawlimit)

    def set_target(self, target):
        self.pid.setpoint = target

    def get_pid(self, current):
        return self.pid(current)

    def reset(self):
        self.pid.reset()
