# 板子同步与测试分析工具

2026-07-26 新增板子同步脚本和 T-016 专用分析脚本

- **关键发现**：Git Bash 无 rsync，改用 `tar | ssh` 流式传输实现增量同步；原 `flight_log_analyzer.py` 的 XY 偏差计算有误，需用 `analyze_t016.py` 精确分析
- **影响范围**：`tools/sync_to_board.sh`、`tools/analyze_t016.py`
- **后续动作**：若需同步到板子，始终用 `sync_to_board.sh` 而非手动 scp

## sync_to_board.sh

```bash
./tools/sync_to_board.sh basic        # 同步 basic 版本
./tools/sync_to_board.sh              # 默认同步 basic
```

使用 `tar cf - --exclude=__pycache__ <dir> | ssh tar xf -` 流式传输，自动排除缓存和日志文件。

## analyze_t016.py

```bash
python tools/analyze_t016.py <flight_data.jsonl>
```

按三段 TAKEOFF 分割三次飞行，正确计算 XY 偏差（到目标点而非到原点），对照通过标准输出 ✅/❌。
