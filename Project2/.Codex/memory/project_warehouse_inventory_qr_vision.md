# 立体货架盘点 QR 码视觉解码实测结论

## QR 内容与映射

二维码内容是大小写敏感的 URL（如 `https://5v60oq.58u.cn/a/AV8YYYN/`），不是
简单数字。`qr_mapping.txt` 使用 `编号<TAB>URL`，必须完整包含1~24。桌面备份
`C:\Users\FJJ\Desktop\二维码备份\qr_mapping.txt` 与当前 `QRMapping` 格式兼容；
URL不能忽略大小写或自动纠正。

## 2026-07-19 飞行帧离线结论

对79张1280×720真实飞行调试帧：

- pyzbar直接解彩色/灰度：0成功；
- OpenCV直接 `detectAndDecode()`：0成功；
- `adaptiveThreshold(gray, 255, GAUSSIAN_C, BINARY, block=31, C=5)` 后 pyzbar
  可解码，是当批数据唯一确认有效的预处理；
- 同一画面可能含多个货架二维码，目标选择不能简单取解码结果第一项。

主要困难：QR只有约80~150px、货架横杆可能截断finder pattern、低对比度和明显
透视变形。

## 当前飞行解码路径（2026-07-20）

`InventoryMissionCoordinator._scan_worker()` 调用：

```python
decoder.detect(frame, target_point=laser_aim)
```

只要带 `target_point`，`QRDecoder.detect()` 就只进入 `_decode_target_roi()`。该路径：

- 在激光/光轴附近裁剪约560×600 ROI；
- 尝试原图、灰度、CLAHE、2倍放大以及1/2/3倍自适应阈值；
- 内容解码只调用 pyzbar；
- 不执行 OpenCV `QRCodeDetector` 内容回退；
- 也不执行全帧 `_decode_search`，避免板端数秒至约17秒阻塞。

这与2026-07-19文档里“target_point走 `_fast_geometry_search + _decode_localized`”的旧描述
不同：该方案因实时耗时过高已撤回。当前真实状态是 **ROI + pyzbar-only**。

## 2026-07-20 A1 结果

A1测试进入 `VERIFY_QR` 后约8.68秒产生 `qr_timeout`。超时结果被主线程正确消费并
进入RETURN，所以“没有后续路径”不能单独归因于视觉；参见
`project_warehouse_inventory_async_return.md`。

当前日志只记录最终 `qr_timeout`，不能判断是：

1. pyzbar从未解码；
2. 解出内容但不在映射；
3. 解码成功但激光点不在QR内/距边缘不足12px；
4. 有零星成功但未达到默认5帧窗口3次共识。

## 桌面备份识别器的启示

备份 `qr_recognizer.py` / `qr_recognizer_hw.py` 使用：

```text
pyzbar优先 → OpenCV detectAndDecode回退 → 连续2帧确认
```

值得采纳：

- 相机采集线程与识别循环分离；
- 只处理新frame_id；
- pyzbar优先、OpenCV轻量回退；
- 简单多帧确认；
- 映射表反查。

不能直接照搬：

- 320×240适合近距离大码，当前远距离小QR应保留高分辨率/ROI像素；
- 3秒debounce锁当前任务不需要，航点、generation和InventoryStore已有去重；
- 硬件识别器里阻塞式激光pulse/每6码转舵机不应进入扫码worker，副作用必须留在
  飞行主线程。

## 建议改进

1. 在 `_decode_target_roi()` 所有pyzbar变体失败后，对有界ROI只增加一次OpenCV
   `detectAndDecode`；不要恢复全帧几何搜索。
2. 增加每个scan generation的统计：读取帧数、pyzbar成功数、OpenCV成功数、未知
   映射数、laser-outside数、consensus最高计数。
3. 诊断测试可临时使用 `DRONE_QR_REQUIRE_LASER_INSIDE=0` 区分“完全解不出”和
   “被激光约束拒绝”；可用窗口3/要求2验证确认门槛，但不能在无日志证据时盲调。
4. 用最新真实A1图片分别离线跑：全帧、ROI、pyzbar、OpenCV ROI和阈值变体；ROI
   可能裁掉finder pattern，必须以真实图验证。
5. 其他货位扫描坐标等完整飞行画面后逐点微调，不从route-only结果推断二维码居中。
