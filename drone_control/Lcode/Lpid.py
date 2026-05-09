import simple_pid
class PID:
    def __init__(self,type=0,target=0) -> None:
        self.xyp=0.7
        self.xyi=0.002
        self.xyd=0.00
        self.yawp=1.5
        self.yawi=0.0
        self.yawd=0.3
        self.xylimit=40
        self.yawlimit=30
        if type==0:
            self.pid=simple_pid.PID(self.xyp,self.xyi,self.xyd,target)
            self.pid.output_limits=(-self.xylimit,self.xylimit)
        else:
            self.pid=simple_pid.PID(self.yawp,self.yawi,self.yawd,target)
            self.pid.output_limits=(-self.yawlimit,self.yawlimit)
        pass
    def set_target(self, target):
        self.pid.setpoint = target

    def get_pid(self,current):
        return self.pid(current)
    def reset(self):
        self.pid.integral = 0
        self.pid.last_error = None
