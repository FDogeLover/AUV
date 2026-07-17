"""警示LED / 抛投机构接口。2026-07-16警示LED硬件已确认就绪(参照
Desktop/GPIO测试/LED测试.ipynb)，warn_led()改为调用Lcode/gpio_led.py真实GPIO；
2026-07-17抛投机构(舵机)硬件也已确认就绪(参照Desktop/GPIO测试/舵机示例.ipynb)，
drop_bag()改为调用Lcode/gpio_servo.py真实PWM舵机。两者调用方(Mission_GPT.py)
都不用改动，接口签名不变。"""
from Lcode.Logger import logger
from Lcode.gpio_led import set_rgb_led
from Lcode.gpio_servo import set_servo_angle, SERVO_ANGLE_OPEN


def warn_led() -> bool:
    """点亮警示LED(常亮红色)，示警识别到火情。GPIO不可用(本机开发环境/硬件未接)
    时降级为仅打日志，不影响调用方(仍返回True，表示"示警动作已发起")。"""
    if set_rgb_led('R'):
        logger.info("warn_led(): 警示LED已点亮(红色)")
    else:
        logger.info("[占位] warn_led(): GPIO不可用，仅记录日志（硬件未接入或非板载环境）")
    return True


def drop_bag() -> bool:
    """触发抛投机构舵机转到释放角度(180°)，打开舱门释放灭火包。只发一次角度
    指令不等待转动完成(不做阻塞式time.sleep())，不自动复位——赛题只需单次
    抛投，复位留给地面维护。GPIO不可用(本机开发环境/硬件未接)时降级为仅打
    日志，不影响调用方(仍返回True，表示"抛投动作已发起")。"""
    if set_servo_angle(SERVO_ANGLE_OPEN):
        logger.info("drop_bag(): 抛投舵机已转到释放角度(180°)")
    else:
        logger.info("[占位] drop_bag(): GPIO不可用，仅记录日志（硬件未接入或非板载环境）")
    return True
