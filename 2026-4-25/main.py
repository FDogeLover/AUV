import os,sys
from Lcode.global_variable import sp_side, lock
import Lcode.Lprotocol
import time
from Lcode.Logger import logger

from Mission_GPT import mission

# ======================== 变量 ===========================
re_fc = [0, 0, 0]  # 飞控反传信息 任务模式/x积分值/y积分值
re_dmz = [('A9', 'B1'),('A10', 'B2'),('A11', 'B3')]#地面站反传信息 三个禁飞区坐标/x/y

se_fc = [170, 0, sp_side, sp_side, 120, sp_side, 0, sp_side, 255]#发送给飞控的数据 帧头/启动命令/x/y/z高度/yaw/任务切换标志位/速度偏置量/帧尾
se_dmz =[170, 0xA9, 0xB1, 0, 0, 255]#发送给地面站的数据 帧头/动物所在坐标x/y/动物类别/动物数量/帧尾

run_sign = False


# ===================== 串口初始化 =====================
serial_fc = Lcode.Lprotocol.Serial_fc("/dev/ttyS6", 460800)
serial_fc.port_open()
serial_fc.listen_start(re_fc)
serial_fc.send_start(se_fc)

serial_dmz = Lcode.Lprotocol.Serial_dmz("/dev/ttyS7", 115200)
serial_dmz.port_open()
serial_dmz.listen_start(re_dmz)
serial_dmz.send_start(se_dmz)

# ==================== 只创建一次任务！====================
mission1 = mission(re_fc, se_fc, re_dmz, se_dmz)

while(1):
    if not run_sign:
        mission1.start()
        run_sign = True

    time.sleep(0.1)