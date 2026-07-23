# ANO_LX_FC_倾角保护版 - 代码检查问题

## 严重问题

1. `FcSrc/User_Task.c:25` — pi_ctrl_mode = 1 硬编码覆盖
   - UserTask_OneKeyCmd() 开头无条件设置 pi_ctrl_mode=1，覆盖RC遥控器 CH5 开关设置的模式
   - 导致遥控模式下也被强制切换为 T265 模式

2. `Mycode/angle_protect.c` — 倾角保护依赖外部IMU数据
   - fc_att.st_data.rol_x100 由数传协议 0x03 帧填充
   - 若外部IMU断线，倾角保护立即失效（数据冻结在最后值）
   - 缺少通信超时检测

3. `DriversMcu/STM32F407/Drivers/Drv_PwmOut.c:205,211` — TIM8预装载配置写错
   - TIM_OC3PreloadConfig(TIM1, ...) 应为 TIM8
   - TIM_OC4PreloadConfig(TIM1, ...) 应为 TIM8
   - 导致TIM8通道3/4预装载配置失效

4. `FcSrc/ANO_LX.c` 中 ESC_Output() 浮点运算
   - 在1ms中断中执行 pwm[i] = pwm_to_esc.pwm_m1 * 0.1f 浮点乘法
   - 可优化为整数除法 pwm[i] = pwm_to_esc.pwm_m1 / 10

## 中等问题

5. 全局变量 pi_ctrl_mode 分散在多处引用
   - 定义在 ANO_LX.c，被 LX_FC_EXT_Sensor.c、User_Task.c 等多处引用
   - 建议统一为枚举类型管理

6. 注释为GB2312编码，UTF-8环境显示乱码

7. 串口数据轮询效率
   - DrvUartDataCheck() 在1ms中断中轮询5个UART缓冲区
   - 数据量大时可能影响中断响应

8. SBUS帧校验码轮询方式效率较低
   - 直接比较 0x00/0x04/0x14/0x24/0x34 即可

## 架构建议

9. Mycode 与 FcSrc 耦合
   - Mycode/my_protocol.c 直接修改 ANO_LX.c 的全局变量
   - 建议将 angle_protect.c 整合进 FcSrc

10. 飞控状态机缺少异常恢复路径

---

# drone_control - 代码检查问题

## 严重Bug

1. `Mission_GPT.py:10` — rgb_led 模块不存在
   - from Lcode.rgb_led import rgb_led 导入不存在模块
   - 第287行调用 rgb_led('R', 1) 将引发 ModuleNotFoundError

2. `Mission_GPT.py:20` — t265_class() 重复实例化
   - 模块级又创建了一个全局 realsense 实例
   - 与 main.py 传入的实例不同，引用 Mission_GPT.realsense 会出错

3. `Mission_GPT.py:159-162` — 串口看门狗被注释
   - 飞控回传超时检测完全注释
   - 断开连接时仍会继续发送指令

4. `test_simulation.py` — 引用已移除的 K230 检测功能
   - m.detecting, m.grid_results, _grid_from_real(), ANIMAL_LABELS 等已不存在
   - 当前完全无法运行

## 中等问题

5. 串口路径硬编码
   - /dev/ttyS6, /dev/ttyS7, /dev/ttyS3 均硬编码
   - 无环境变量或配置文件机制，Windows上无法测试

6. `Lprotocol.py` — Serial_gpio 类完全未使用（死代码）

7. 缺少串口断线重连机制，断开导致永久性故障

8. 锁使用不统一：混用 lock.acquire()/release() 和 with lock:

9. 缺少飞行数据日志（位置/PID/状态转换的结构化记录）

10. `Logger.py` — fileHdl/consoleHdl 作为属性污染 logger 命名空间

## 架构建议

11. 多线程竞争 re_fc 读取：主线程读 re_fc 时无锁保护

12. takeoff() sleep 1s 后无确认机制，注释"等待飞控完成"但无实际检查