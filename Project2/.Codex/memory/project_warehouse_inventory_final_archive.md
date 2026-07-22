# 立体货架盘点赛题最终归档

2026-07-22 阶段验收。A→B→C→D 40 航点最短路线连续两轮飞完全程并正常
降落上锁；最新一轮 24/24、`missing_slots=[]`、`complete=true`。

最终配置：raw-only、FOV precheck 关闭、异步扫码、T265 heading hold、最大 3°/s。
无 QR/解码失败/timeout/duplicate 均继续下一货位；硬件和飞行安全故障仍降落。

最终几何：A→B、C→D 上端 `X=-2.80`；B `Y=1.65`；D `Y=3.45`；扫码高度
`1.25/0.85`；降落 `(-2.50,3.50,0.20)`。

详细报告：`docs/session-summaries/2026-07-22-warehouse-inventory-final.md`。
本机数据：`drone_control/tools/data_archive/test_data_20260722/warehouse_full_abcd_continue_on_miss_v2_*`。
板端数据：`/home/sunrise/Desktop/FJJ/test_data/warehouse_inventory_20260722/`。
