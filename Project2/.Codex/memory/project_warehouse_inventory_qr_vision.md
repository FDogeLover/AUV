# 立体货架盘点 QR 码视觉解码实测结论

## 关键发现（2026-07-19 飞行数据离线分析）

### 1. QR 码内容格式
二维码内容为 URL 格式（如 `https://5v60oq.58u.cn/a/AV8YYYN/`），不是简单数字。
通过 `qr_mapping.txt` 将 URL 映射到货位编号 1~24，该文件是解码的必要依赖。

### 2. pyzbar 裸调用零成功率
对 79 张飞行调试帧（1280×720，`VERIFY_QR` 状态 + `VISUAL_SERVO` 状态）：
- `pyzbar_decode(color_img)`：0 解码成功
- `pyzbar_decode(gray_img)`：0 解码成功
- `cv2.QRCodeDetector().detectAndDecode()`：0 解码成功

### 3. 唯一有效的预处理：自适应阈值
`cv2.adaptiveThreshold(gray, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, block=31, C=5)` 后 pyzbar 可成功解码。
同一张图片 pyzbar 能解出多个 URL（不同 block 参数 → 对应不同 QR 码），说明图中同时有多个货架二维码。

失败原因：
- 金属货架横杆截断 finder pattern
- QR 码在画面中只有 80~150px（19cm QR 在约 80cm 距离+广角下）
- 白墙白纸低对比度
- 俯仰角 30-40° 导致透视变形

### 4. 原代码 detect() early-return bug（已修复）
`qr_vision.py` 的 `QRDecoder.detect()` 原第 668 行：
```python
if pyzbar_decode is not None:
    return None  # ← 直接放弃，不走 OpenCV fallback
```
此行阻断了 `_fast_geometry_search` + `_decode_localized`（含 2x/3x 放大 + `_decode_warped` 透视矫正）的 fallback 路径。

**修复方案（commit `0cfb9ab`）**：
- 移除 early-return
- 在 `target_point is not None` 分支内独立调用 `_fast_geometry_search` → `_decode_localized`
- 明确 `return None` 不落入 `_decode_search`（全帧 tile 扫描，数秒级，会阻塞多帧采样）
- `_decode_search` 仅当 `target_point is None`（离线/调试路径）时可达

### 5. 测试数据位置
本机：`drone_control/tools/test_data_warehouse_inventory_20260719/vision_debug/`
板端：`/home/sunrise/Desktop/FJJ/test_data/warehouse_inventory_20260719/vision_debug/`

共 79 张 .jpg + 79 张 .json（含 state/slot/position/detected_number 等元数据）。

## 待验证（下次真机测试）
- 修复后的 detect() 是否能在真实飞行帧中解码成功
- `_decode_target_roi` 的 adaptiveThreshold 变体（block=31, C=5，已内置）是否在 ROI
  裁剪后也能成功——离线测试显示全帧成功但 ROI 内失败，ROI margin 可能裁掉了部分 finder pattern
- 是否需要加大 `geometry_roi_width/height` 或在 `_decode_target_roi` 中尝试更多 block/C 组合
