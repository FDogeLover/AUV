# 立体货架盘点 QR 码视觉解码实测结论

## QR 内容与映射

二维码内容是大小写敏感的 URL。`qr_mapping.txt` 使用 `编号<TAB>URL`，必须完整包含1~24。桌面备份与当前 `QRMapping` 格式兼容。

## 解码路径演变

| 版本 | 解码方式 | 实飞结果 |
|------|----------|----------|
| 原始版 | ROI多变体(pyzbar) | A1全部qr_timeout，ROI裁掉finder pattern |
| adaptiveThreshold版 | 全帧adaptiveThreshold+pyzbar | A1在1.25m三次成功，板端~200ms/帧 |
| 备份识别器版(最终) | 下采样800px→pyzbar→OpenCV回退 | 同adaptiveThreshold版，增加了OpenCV容错 |

## 2026-07-20 实飞结论

- A1 在 **1.25m 高度** 三次成功解码（使用 `DRONE_INVENTORY_SCAN_Z=1.25` 和模型默认 `top_qr_z_m=1.25` 各一次验证）
- 共识门槛从 `window=5/required=3` 降到 `window=3/required=2`（与桌面备份的2帧确认一致）
- 默认 `require_laser_inside=True` 保留；解码统计已加入 `ScanResult.decode_stats`
- 视觉伺服因板端性能限制（单帧解码~200ms）+飞行抖动导致连续闭环不收敛，暂不启用
- 激光改物理调正，不开环软件补偿
- K230 作为后续赛题升级路径，当前不引入

## 桌面备份识别器的启示

备份 `qr_recognizer.py` 方案：pyzbar优先 → OpenCV `detectAndDecode` 回退 → 2帧确认 → 3秒去抖

采纳项：
- pyzbar优先 + OpenCV轻量回退
- 下采样加速
- 2帧共识（与window=3/required=2等效）

不采纳项：
- 3秒debounce锁（已由航点+generation+InventoryStore去重）
- 阻塞式激光pulse/舵机（副作用必须在主线程）

## 后续

- 激光物理调正后可用 `DRONE_LASER_AIM_X_RATIO` 和 `DRONE_LASER_AIM_Y_RATIO` 微调
- A2~A6解码待 consensus 门槛降低后复飞验证
- 若仍不通过，考虑逐货位微调扫码坐标
