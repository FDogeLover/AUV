"""
基本飞行 — 主入口

运行:
  python main.py

依赖:
  pyrealsense2 (或使用模拟 fallback)
  pyserial
  simple_pid
  numpy

启动顺序:
  1. 创建 T265 实例
  2. 打开飞控串口 → 启动监听 + 发送线程
  3. 创建任务 → 启动状态机
  4. 保持主线程存活
"""
import os
import time
from Lcode.global_variable import sp_side, lock
import Lcode.Lprotocol
from Lcode.Logger import logger
from Mission_GPT import mission
from t265 import t265_class


# ======================== 变量 ========================
re_fc = [0, 0, 0, 0]  # [mission_stage, integral_x, integral_y, laser_cm]

# AA 02 task_sta com_x+sp com_y+sp com_z com_yaw+sp next_task sp_side CK FF
se_fc = [170, 2, 0, sp_side, sp_side, 120, sp_side, 0, sp_side, 0, 255]


# ===================== 入口 =====================
def main():
    logger.info("=" * 40)
    logger.info("basic_flight — 基本飞行控制器")
    logger.info("=" * 40)

    # 1. T265
    realsense = t265_class()

    # 2. 飞控串口
    port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
    serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
    serial_fc.listen_start(re_fc)
    serial_fc.send_start(se_fc, realsense, vel_freq=100, cmd_freq=50)

    # 3. 创建任务
    mission1 = mission(re_fc, se_fc, realsense, serial_fc)

    # 4. 启动
    mission1.start()

    # 5. 保持
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("用户中断")
        mission1.emergency()
        serial_fc.send_end()
        serial_fc.close()


if __name__ == "__main__":
    main()
