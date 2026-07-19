## 修复 qr_vision.py 的 pyzbar early-return 问题

### 问题
`qr_vision.py:668-669` — pyzbar 在 fast path 失败后直接 `return None`，完全跳过了 OpenCV 几何定位 + 透视矫正的 fallback 路径。从实际飞行数据看，pyzbar 在金属货架遮挡、俯仰角大、QR 码小等条件下零解码成功。

### 修复方案
移除第 668-669 行的 `if pyzbar_decode is not None: return None`，让 pyzbar 失败后继续走 `_fast_geometry_search` + `_decode_localized`（含 2x/3x 放大 + `_decode_warped` 透视矫正）。

### 性能考量
- `_fast_geometry_search`（OpenCV `detect`）约 50-200ms
- `_decode_localized`（裁剪 + 放大 + 透视矫正）约 20-50ms
- 合计约 100-250ms/帧，远快于全帧 tile 扫描（`_decode_search` 数秒级）
- 二维码共识层（`QRConsensus`）已有多帧确认机制，不需要每帧都解码成功

### 修改文件
- `drone_control/warehouse_inventory/Lcode/qr_vision.py` 第 665-669 行
- `drone_control/warehouse_inventory/test_qr_vision.py` 补充验证 pyzbar 失败时 fallback 到 OpenCV 路径的测试

### 具体改动

**qr_vision.py `detect()` 方法：**
```python
# 旧代码（第 665-669 行）：
            # pyzbar is the deployed flight decoder.  Do not fall through to
            # the multi-pass OpenCV search on every frame: on the board that
            # path takes several seconds and prevents multi-frame sampling.
            if pyzbar_decode is not None:
                return None

# 改为：
            # pyzbar fast path failed; fall through to the geometry search
            # which adds perspective correction (warp) and multi-scale
            # retry.  This is more expensive than raw pyzbar but far cheaper
            # than the full-frame _decode_search tile scan.
```

**test_qr_vision.py 新增测试：**
- 验证当 pyzbar 返回空结果时，detect() 继续调用 `_fast_geometry_search` 而非直接返回 None
- mock `pyzbar_decode` 返回空列表 + mock `cv2.QRCodeDetector.detect` 返回有效几何，确认 fallback 路径可达
