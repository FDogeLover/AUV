# K230 视觉代码 - 代码检查问题

## 文件清单

| 文件 | 行数 | 功能 |
|---|---|---|
| dataset_capture.py | 244 | 离线数据集自动采集，5类按键切换，HDMI预览 |
| animal_detect_visual.py | 338 | YOLOv8n实时检测 + HDMI可视化 + UART JSON输出 |
| animal_detect_yolov8n.py | 353 | YOLOv8n检测 + Pi双向串口协议(AA..FF二进制帧) |

Pi端配套：drone_control/Lcode/k230_client.py

## 严重问题

1. AnimalDetectApp 代码重复
   - animal_detect_visual.py 和 animal_detect_yolov8n.py 各自定义了 AnimalDetectApp 类
   - _parse_dets、postprocess、config_preprocess 完全相同
   - 建议提取公共基类到 k230/detector_base.py

2. uart_rx_thread 无退出机制
   - animal_detect_yolov8n.py 的守护线程只在 OSError 时退出
   - 主循环正常结束后线程仍可能继续运行
   - 建议增加退出标志

3. UART 线程安全问题
   - animal_detect_yolov8n.py 的接收线程 uart.read() 与主线程 uart.write() 并发
   - 无锁保护，可能导致数据损坏
   - 建议加锁或把写入也放到接收线程

## 中等问题

4. 文件名误导
   - animal_detect_yolov8n.py 实际是 Pi 双向通信专用版
   - 而 animal_detect_visual.py 包含完整的推理+可视化代码
   - 建议改名：animal_detect_yolov8n.py -> animal_detect_pi_protocol.py

5. 缺少 os.exitpoint()
   - animal_detect_yolov8n.py 主循环中未调用
   - dataset_capture.py 和 animal_detect_visual.py 都有
   - IDE 可能无法优雅中断

6. UART write() 无完整性检查
   - 未校验写入字节数是否等于帧长度
   - Pi 端可能收到格式错误的帧

7. get_uart_data 仅保留最高置信度类别
   - 多种类同时出现时合并到最高置信度类别
   - 多动物场景信息丢失

8. kmodel 路径硬编码且不一致
   - visual版: /sdcard/examples/mycode/animal_yolov8n_v2_best.kmodel
   - yolov8n版: new_animal_v2.kmodel
   - 建议统一为可配置参数或环境变量

## 轻度问题

9. dataset_capture.py save_jpg() 无写入校验
   - TF卡已满时会静默写入空文件
   - 后续索引扫描可能出错

10. animal_detect_visual.py 缺少 RGB LED 错误指示
    - dataset_capture.py 有完整的 led_pulse_n 错误指示
    - 建议复用该模式

11. draw_result 坐标映射精度问题
    - int(round(v, 0)) 等价于 int(v)
    - 负坐标时 int(-0.6)=0 而非 -1
    - 影响极小（1像素偏差）

## 架构建议

12. k230_client.py 在 drone_control/Lcode/ 而非 k230/ 下
    - 虽然是合理设计（Pi端代码归Pi端），但不熟悉项目的人可能找不到
    - 可以考虑在 k230/ 下加一个 README 说明协议定义位置