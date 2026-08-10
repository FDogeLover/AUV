# 串口协议 Lprotocol

## :memo: 本节介绍

`Lcode/Lprotocol.py` 负责上位机与飞控之间的串口双向通信，使用匿名数传 AA 帧协议。这是整个系统的数据动脉——T265 速度、飞控姿态、控制指令全部经过这里。

<div class="learning-goals" markdown="1">

### :trophy: 学习目标


1. 知道 `Serial_fc` 类的职责和三线程架构
2. 理解 AA 帧协议的帧格式（帧头/帧ID/数据/校验/帧尾）
3. 能说出每个发送/接收线程的频率和内容
4. 知道 DRY_RUN 模式下串口如何降级

</div>

---

## 三线程架构

```
Lprotocol.py (Serial_fc)
├── 监听线程    飞控→Pi   50Hz（姿态/激光高度/解锁状态/调试帧）
├── T265发送    Pi→飞控   100Hz（速度+位置）
└── 指令发送    Pi→飞控   50Hz（速度指令）
```

| 线程 | 方向 | 频率 | 帧ID | 内容 |
|------|------|------|------|------|
| 监听 (`listen_fc`) | 飞控→Pi | 50Hz | `0x01` | 姿态角、解锁状态、激光高度、光流数据 |
| 监听 (调试帧) | 飞控→Pi | 2Hz | `0x02` | 飞控速度估计、光流IMU、电机PWM掩码 |
| T265发送 (`_send_t265_loop`) | Pi→飞控 | 100Hz | `0x01`/`0x03` | T265速度(cm/s) + 位置(cm) |
| 指令发送 (`_send_command_loop`) | Pi→飞控 | 50Hz | `0x02` | 任务状态 + 速度指令 |

## AA 帧协议格式

```
飞控下行帧:
  AA | frame_id | len | DATA[len] | checksum | FF

上行帧 (Pi→飞控):
  AA 01: T265速度帧   — vx_cm, vy_cm, yaw_x100 (8字节)
  AA 02: 指令帧       — task_sta, com_x/y/z/yaw (速度指令)
  AA 03: T265位置帧   — x_cm, y_cm, z_cm (12字节)
```

---

## API 参考

<div class="api-class-card">
  <div class="api-class-name">class Serial_fc</div>
  <div class="api-class-desc">飞控串口双向通信，三线程收发</div>
</div>

### 构造方法

<div class="api-method">
  <div class="api-method-sig">
    <span class="type-hint">Serial_fc</span>(<span class="param-name">port</span>: str, <span class="param-name">baudrate</span>: int)
  </div>
  <div class="api-method-desc">打开串口并初始化线程控制标志。设置 read timeout=1.0s 和 write_timeout=1.0s 防止阻塞卡死。</div>
</div>

| 参数 | 类型 | 说明 |
|------|------|------|
| `port` | `str` | 串口路径，默认 `/dev/ttyS6` |
| `baudrate` | `int` | 波特率，默认 `460800` |

### 生命周期方法

<div class="api-section-label">监听线程</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">listen_start</span>(<span class="param-name">rxbuffer</span>: List[int]) <span class="api-returns">→ None</span>
  </div>
  <div class="api-method-desc">启动飞控监听线程（daemon），清空输入缓冲区后开始接收下行帧。数据写入传入的 rxbuffer 列表。</div>
</div>

| 参数 | 类型 | 说明 |
|------|------|------|
| `rxbuffer` | `List[int]` | 共享缓冲区，监听线程将解析后的字段追加到此列表 |

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">listen_end</span>(<span class="param-name">timeout</span>: float = 1.5) <span class="api-returns">→ None</span>
  </div>
  <div class="api-method-desc">停止监听线程。先设标志位，再调用 `cancel_read()` 唤醒阻塞的 read()，最后 join 线程。必须在线程退出后才能 close fd。</div>
</div>

<div class="api-section-label">发送线程</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">send_start</span>(<span class="param-name">comlist</span>: List[int] = None, <span class="param-name">t265_obj</span> = None, <span class="param-name">vel_freq</span>: int = 100, <span class="param-name">cmd_freq</span>: int = 50) <span class="api-returns">→ None</span>
  </div>
  <div class="api-method-desc">启动 T265 发送线程和指令发送线程（均为 daemon）。两个线程可独立启动。</div>
</div>

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `comlist` | `List[int]` | `None` | 指令帧共享列表，Mission 状态机实时写入速度指令 |
| `t265_obj` | `t265_class` | `None` | T265 对象，用于读取速度和位置发送给飞控 |
| `vel_freq` | `int` | `100` | T265 速度帧发送频率 (Hz) |
| `cmd_freq` | `int` | `50` | 指令帧发送频率 (Hz) |

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">send_end</span>(<span class="param-name">timeout</span>: float = 1.5) <span class="api-returns">→ None</span>
  </div>
  <div class="api-method-desc">停止两个发送线程，join 等待退出。</div>
</div>

<div class="api-section-label">资源清理</div>

<div class="api-method">
  <div class="api-method-sig">
    .<span class="param-name">close</span>() <span class="api-returns">→ None</span>
  </div>
  <div class="api-method-desc">兜底清理：先 send_end() 再 listen_end()，最后关闭串口 fd。重复调用是幂等的。</div>
</div>

### 实例属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `ser` | `serial.Serial` | pyserial 串口对象 |
| `debug_data` | `dict` | 调试帧(0x02)最新数据：fc_vel/of_acc/of_gyr/motor_pwm_mask |
| `_last_laser_height_cm` | `float` | 最后有效的激光高度 (m)，过滤异常值 |

---

## 使用示例

```python
from Lcode.Lprotocol import Serial_fc
from Lcode.global_variable import lock

rxbuffer = []
comlist = [0xAA, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF]  # 指令帧模板

fc = Serial_fc(port="/dev/ttyS6", baudrate=460800)
fc.listen_start(rxbuffer)
fc.send_start(comlist=comlist, t265_obj=t265, vel_freq=100, cmd_freq=50)

# ... 飞行过程中 Mission_GPT 实时更新 comlist 中的速度指令 ...

fc.send_end()
fc.listen_end()
fc.close()
```

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DRONE_FC_PORT` | `/dev/ttyS6` | 飞控串口路径 |
| `DRONE_DRY_RUN` | `0` | `1` = 模拟模式，不打开真实串口 |

---

## :material-help: 常见问题

??? question "为什么 write_timeout 必须设置？"
    pyserial 默认无 write_timeout。如果飞控端串口线断开或缓冲区写不出去，`ser.write()` 会**无限阻塞且不抛异常**，两个发送线程会静默卡死，整个飞控失控。设置 `write_timeout=1.0` 后超时会抛 `SerialTimeoutException`，线程能正常退出并记录错误。

??? question "rxbuffer 为什么用 List 而不是单独变量？"
    监听线程和 Mission 主循环共享 rxbuffer。使用 List 配合 `with lock:` 临界区，可以原子性地 clear + append 全部字段，避免 Mission 读到"新 yaw + 旧时间戳"的撕裂快照。

??? question "调试帧 (0x02) 的 motor_pwm_mask 有什么用？"
    用于诊断 `land()` 假阳性锁桨问题（#22）。`unlock_sta` 可能读到 0 但电机实际未停转，`motor_pwm_mask` 的 bit0~3 对应 m1~m4 的 PWM 是否非零，是独立于飞控状态字段的硬件级佐证。

---

← [状态机流程](../state-machine.md) | [PID控制器 Lpid →](lpid.md)
