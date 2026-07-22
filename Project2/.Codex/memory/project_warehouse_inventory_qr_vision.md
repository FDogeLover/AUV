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

## 2026-07-22 最终实飞方案

- 实飞对比后使用 `DRONE_QR_DECODE_PROFILE=raw`，只对原始 ROI 解码；变体链路
  保留在代码中作为调试回退，不是最终飞行配置；
- `DRONE_QR_FOV_PRECHECK=0`，不让快速 FOV 检查因“无 QR”阻塞货位；
- 异步扫码保持飞行控制循环；未检测、解码失败、`qr_timeout` 和 `qr_duplicate`
  均只将当前货位记为缺失并继续下一格，不提前降落；
- 2026-07-22 完整路线最新一轮 24 个货位全部识别，`missing_slots=[]`；
- 该赛题已阶段归档，不再追加视觉伺服或 K230 改造。
