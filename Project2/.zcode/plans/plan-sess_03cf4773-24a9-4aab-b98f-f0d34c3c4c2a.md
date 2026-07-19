## 文档全量更新计划

### 目标
基于项目实际情况更新文档：移除 Codex–Claude 协作规范、更新项目结构、同步已知问题。

### 修改文件清单

| # | 文件 | 操作 |
|---|------|------|
| 1 | `.Codex/CLAUDE.md` | 移除 Codex-Claude 协作规范（第 207-248 行），更新项目结构（添加 `warehouse_inventory`、`fire_patrol`、`CodeWiki`） |
| 2 | `.claude/CLAUDE.md` | 移除 Codex-Claude 协作规范（第 205-246 行），同步项目结构和已知问题（补问题 38-39，修正问题 16/27/35 状态） |
| 3 | `docs/known_issues.md` | 新增问题 38（串口监听线程关闭竞态）和问题 39（BCM17 一键起飞门禁）的详细 `<details>` 条目，修正问题 29 条目顺序 |

### 具体变更细节

**1. `.Codex/CLAUDE.md` 项目结构更新：**
- `drone_control/` 下新增：
  - `├── fire_patrol/ ← 空地协同消防赛题模块（2026-07-17 真机验证通过）`
  - `├── warehouse_inventory/ ← 立体货架盘点赛题模块（2026-07-19 集成状态机）`
- 根目录新增：`├── CodeWiki/ ← 项目 wiki 文档索引`
- 使用方式新增仓库盘点命令

**2. `.claude/CLAUDE.md` 已知问题同步：**
- 问题 16：`🔴/待真机` → `✅/赛题验收`
- 问题 27：`🟡` → `✅/赛题验收`
- 问题 35：补充更多样本细节
- 新增问题 38、39（与 `.Codex/CLAUDE.md` 一致）

**3. `docs/known_issues.md` 补充：**
- 按现有格式为问题 38、39 补充完整 `<details>` 条目
- 将问题 29 条目移到正确位置