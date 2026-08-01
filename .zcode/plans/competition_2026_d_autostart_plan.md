# 计划：D题无人机统一自启动

## 问题描述与目标

RDK与Cyber Camera同时上电后，由一个常驻入口完成共享硬件预检；T265在人工拔插完成前不得创建管线。小车发送现有`CAR_START.task_mode`后，入口选择任务一或任务二，安全地ACK并启动已验证的联合测试配置。任务成功结束后不得自动重新武装。

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|---|---|---|
| 同时启动任务一、任务二进程 | 实现简单 | 蓝牙、相机、飞控串口和GPIO互相争抢，不可用 |
| 先消费CAR_START再启动子进程 | 改动较少 | ACK、重传和5秒安全倒计时容易与子进程预检错位 |
| 单进程统一预检与任务分派 | 串口唯一所有者，保留现有ACK和安全门禁 | 新入口需要复用两套任务构造和遥测逻辑 |

选择单进程统一预检与任务分派。共享硬件只初始化一次；T265仅检查USB枚举，收到有效CAR_START后才构造对象、启动管线和校零。

## 改动范围

- `drone_control/competition_2026_d/auto_start.py`：统一预检、双任务READY、任务选择、灯光、ACK、任务构造、遥测和清理。
- `drone_control/competition_2026_d/test_auto_start.py`：任务选择、幂等ACK、错误帧、T265 USB枚举和门禁测试。
- `drone_control/competition_2026_d/competition-2026-d-autostart.service`：RDK systemd单元模板。
- `drone_control/competition_2026_d/task1_start.py`及测试：联合模式统一1.2m、Cyber Camera慢启动等待（已完成）。

## 风险点

- T265不得在CAR_START前启动；仅允许读取`lsusb`枚举结果。
- `CAR_START`只有在相机双向链路、飞控心跳、舵机锁定、蓝牙和T265枚举均有效时才ACK。
- 任务ID灯在ACK前短暂显示；红灯在ACK前点亮，保证小车5秒倒计时与无人机红灯同步。
- 重复CAR_START只回复ACK，不重复构造任务或起飞。
- 任务成功结束后systemd不重启；异常退出才重启。
- 停止服务或异常时必须停止任务、关闭串口、停止T265并关闭灯。

## 验证方式

- 单元测试覆盖任务1/2选择、错误flags、零session、未知任务、重复帧和USB枚举。
- 运行`competition_2026_d`完整pytest回归。
- 板端先用`DRONE_DRY_RUN=1`和未装桨状态验证开机、OLED双OK、T265拔插、任务灯、CAR_START ACK及任务分派。
- 未经单独安全确认不启用systemd、不执行带桨起飞。
