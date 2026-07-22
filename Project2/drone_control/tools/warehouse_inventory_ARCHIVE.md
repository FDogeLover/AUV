# warehouse_inventory 2026-07-22 归档索引

## 最终验收组

- `warehouse_full_abcd_continue_on_miss_v2_console_20260722.log`：最新一轮完整控制台日志；
- `warehouse_full_abcd_continue_on_miss_v2_flight_data_full_20260722.jsonl`：归档时的追加飞行数据；
- `warehouse_full_abcd_continue_on_miss_v2_inventory_results_20260722.json`：最新一轮24/24结果；
- `warehouse_full_abcd_continue_on_miss_v2_fc_log_20260722.log`：归档时的飞控日志；
- `warehouse_inventory_state_full_day_20260722.jsonl`：7月22日累计状态记录；
- `warehouse_full_abcd_continue_on_miss_v2_vision_debug_20260722/`：248张视觉调试图，包含同名调试目录累积数据。

两轮最终飞行使用相同的`tee`文件名，所以控制台日志只保留第二轮。用户现场确认
两轮都飞完全程；24/24结果来自最新一轮。

## 关键历史组

- `warehouse_full_abcd_shortest_v1_*`：首次最短完整路线，D2 duplicate后提前降落，22/24；
- `warehouse_C_t265_signfixed_v6_*`：C面T265 yaw符号修正阶段；
- `warehouse_B_raw_full_*`、`warehouse_C_raw_full_*`：raw-only分面扫码验证；
- `warehouse_*height_step*`：高度变化与yaw诊断；
- 其余B/C分面日志和图片为参数演进过程，保留用于回溯，不代表最终运行配置。

总结见`docs/session-summaries/2026-07-22-warehouse-inventory-final.md`。

## 板端统一归档

板端路径：`/home/sunrise/Desktop/FJJ/test_data/warehouse_inventory_20260722/`。

- `runtime_and_backups/`：任务根目录中的控制台、飞控、状态、盘点结果和历史备份；
- `vision_debug_all/`：板端原 `warehouse_inventory/vision_debug/` 全部视觉调试数据；
- 目录中原有的最终飞行副本和B面分面数据继续保留。

整理后约 226 MB、2039 个文件。板端 `.gitignore` 已忽略 `test_data/`、
`vision_debug/`、`*.log*`、`*.jsonl*` 和 `inventory_results.json*`，运行产物不再进入 Git。
