"""警示LED / 抛投机构占位接口。硬件均未就绪(见设计文档"占位接口"一节)，
GPIO映射留TBD——先打日志占位，硬件到位后在这两个函数内部接GPIO调用，
调用方(Mission_GPT.py)不用改动。"""
from Lcode.Logger import logger


def warn_led() -> bool:
    """点亮警示LED，示警识别到火情。硬件未就绪，先打日志占位。"""
    logger.info("[占位] warn_led(): 警示LED点亮（硬件未接入）")
    return True


def drop_bag() -> bool:
    """触发抛投机构释放灭火包。硬件未就绪，先打日志占位。"""
    logger.info("[占位] drop_bag(): 抛投灭火包（硬件未接入）")
    return True
