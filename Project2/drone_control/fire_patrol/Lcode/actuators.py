"""警示LED / 抛投机构接口。2026-07-16警示LED硬件已确认就绪(参照
Desktop/GPIO测试/LED测试.ipynb)，warn_led()改为调用Lcode/gpio_led.py真实GPIO；
抛投机构硬件仍未就绪，drop_bag()继续日志占位，调用方(Mission_GPT.py)不用改动。"""
from Lcode.Logger import logger
from Lcode.gpio_led import set_rgb_led


def warn_led() -> bool:
    """点亮警示LED(常亮红色)，示警识别到火情。GPIO不可用(本机开发环境/硬件未接)
    时降级为仅打日志，不影响调用方(仍返回True，表示"示警动作已发起")。"""
    if set_rgb_led('R'):
        logger.info("warn_led(): 警示LED已点亮(红色)")
    else:
        logger.info("[占位] warn_led(): GPIO不可用，仅记录日志（硬件未接入或非板载环境）")
    return True


def drop_bag() -> bool:
    """触发抛投机构释放灭火包。硬件未就绪，先打日志占位。"""
    logger.info("[占位] drop_bag(): 抛投灭火包（硬件未接入）")
    return True
