# warehouse_inventory

2024 立体货架盘点无人机系统（D 题）独立版本，基线来自
`drone_control/basic`。

## 当前状态

该赛题于 **2026-07-22 阶段验收并归档**。完整 A→B→C→D 40 航点路线连续两轮
实飞均飞完全程、正常降落上锁；最新一轮盘点 `24/24`、`missing_slots=[]`、
`complete=true`。入口仍默认安全锁定，每次经现场安全确认后必须显式设置
`DRONE_WAREHOUSE_MISSION_READY=1`才会进入物理按键、T265/飞控检查和红灯 5 秒警示链路。

最终状态包括：24 货位规划、异步 raw-only QR 扫码、0°/180°云台、激光、
T265 航向保持、广播、完整状态/飞行/视觉日志，以及“货位未识别、超时或重复 QR
只跳过当前格，不截断剩余航线”的容错策略。飞控、定位、激光等硬件级故障仍保留
安全退出/降落保护。

设计方案见：

`../../docs/superpowers/specs/2026-07-18-warehouse-inventory-design.md`

## 已继承能力

- T265 定位和飞控串口；
- 精确/巡航双导航模式；
- 固定起飞航向保持；
- BCM17 绿灯等待、按键后初始化 T265/飞控、红灯 5 秒再起飞；
- 现有降落、日志和资源监控测试。

## 路径坐标约定

- 起飞点为 T265 局部原点；
- `+Y` 沿场地 5 m 长边；
- `-X` 沿场地 4 m 宽边；
- `+Z` 向上。

路线规划必须统一通过坐标变换模块生成，禁止在各货位中单独交换 X/Y 或补正负号。

## 训练通信与调试开关

- 地面广播端口：`/dev/bt_serial`；
- 当前没有地面站接收端，训练模式只广播，绝不等待 ACK 或阻塞飞行；
- `DRONE_STATE_DEBUG_LOG=1`：记录每个状态的进入、周期数据、退出、持续时间和原因；
- `DRONE_VISION_DEBUG_CAPTURE=1`：按固定点或航行频率保存调试图片；
- `DRONE_QR_DECODE_PROFILE=raw`：最终实飞配置，只解码原始 ROI；
- `DRONE_QR_FOV_PRECHECK=0`：关闭快速 FOV 预检，直接进入有界完整扫码；
- `DRONE_HEADING_SOURCE=t265`、`DRONE_HEADING_HOLD_MAX_DPS=3`：最终航向保持配置；
- `DRONE_CAMERA_ZOOM=1`：可选的 UVC 数字/光学变焦级别；用于二维码过小或相邻二维码过多时的单货位测试，默认不设置；
- `DRONE_VISION_SERVO=1`：在每个货位先进入 `VISUAL_SERVO`，利用二维码几何框
  做有界的 X 横向微调和 Z 高度微调，稳定后再进入现有 3/5 帧内容确认；默认
  关闭。伺服不会控制朝货架的 Y 深度，且受目标丢失、超时、速度和位移上限保护。
  方向/增益可用 `DRONE_VISION_SERVO_X_SIGN`、`DRONE_VISION_SERVO_Z_SIGN`、
  `DRONE_VISION_SERVO_X_KP`、`DRONE_VISION_SERVO_Z_KP` 调整；首次真机测试建议先
  开启调试照片并用单货位。
- 正式任务可关闭高频状态数据和图片，但状态转换、故障、急停与结果摘要始终保留。

## 已确认视觉原型

`reference_qr_prototype/` 保存了板端
`/home/sunrise/Desktop/视觉测试/二维码/` 的只读快照，用于后续适配：

- UVC 摄像头与激光共同固定在舵机上；
- `pwmchip0/pwm0` 控制 0°/180°；
- BCM19 控制激光 PWM；
- OpenCV `QRCodeDetector` 解码；
- `qr_mapping.txt` 将实际二维码内容映射到编号 1~24。

这些文件是原型参考，不直接被 `main.py` 导入。正式实现需要增加多帧确认、货位
状态约束、约 0.5 秒非阻塞点亮、云台稳定等待、异常强制关激光和调试拍照开关。

## 最终路线与归档

- 扫描顺序：`A1,A2,A3,A6,A5,A4 → B6,B5,B4,B1,B2,B3 → C1,C2,C3,C6,C5,C4 → D6,D5,D4,D1,D2,D3`；
- A→B 和 C→D 均走上端 `X=-2.80 m`；B 面 `Y=1.65 m`，D 面 `Y=3.45 m`；
- 上/下层扫码高度分别为 `1.25/0.85 m`，降落点 `(-2.50, 3.50, 0.20)`；
- 本机最终数据：`drone_control/tools/data_archive/test_data_20260722/warehouse_full_abcd_continue_on_miss_v2_*`；
- 板端统一归档：`/home/sunrise/Desktop/FJJ/test_data/warehouse_inventory_20260722/`；
- 最终总结：`docs/session-summaries/2026-07-22-warehouse-inventory-final.md`。
