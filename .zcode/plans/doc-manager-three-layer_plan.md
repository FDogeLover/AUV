# 计划：doc-manager 三层文档/记忆体系全覆盖

## 问题描述 & 目标

### 现状
doc-manager 技能只管理 `docs/` 下的 Markdown 文档，但项目实际存在**三层文档/记忆体系**：
- **① 项目文档** `docs/`（~29 个 .md 文件）
- **② 智能体记忆** `.Codex/memory/` + `.Codex/MEMORY.md` + `.claude/MEMORY.md`（~51+ 文件）
- **③ 全局 ZCode 记忆** `.zcode/cli/memories/projects/project2-*/`（~3 文件）

目前 doc-manager 的检查/地图/同步/核验/更新五个模式均不感知②③层，导致：
- 引用的路径断裂无法跨层检测
- `.Codex/memory/` 索引与实际文件可能不同步
- TODO.md 状态变更但记忆文件未反映

### 目标
将 doc-manager 的覆盖范围扩展到全部三层文档/记忆体系，实现跨层一致性管理。

## 方案选择

| 方案 | 优点 | 缺点/风险 |
|------|------|----------|
| **方案A：完全集成（选定）** | 一次性解决跨层一致性问题；各模式能力对齐；架构清晰 | 改动量大（需重写 SKILL.md 全部模式）；需仔细定义各层读写权限 |
| 方案B：分层模块化——SkILL中定义接口，各层独立实现 | 风险隔离好 | 代码冗余；跨层校验逻辑分散；实施周期长 |
| 方案C：渐进式——先加 /doc check 跨层检测，其他模式后续 | 改动最小 | 能力不完整；用户期望一次到位 |

**选择方案A，理由：** doc-manager 是技能描述（SKILL.md），不是代码实现。改动本质是更新行为规范文档，不存在"部分实施"风险。一次性完整更新比反复修改更高效。

## 改动范围

### 核心文件
- `.agents/skills/doc-manager/SKILL.md` — 唯一需要修改的文件

### 具体变更

#### 1. 概述部分重写
- 新增「三层文档/记忆体系」定义表
- 新增各层读写权限矩阵
- 范围描述从 `docs/` 扩展到全部三层

#### 2. 模式1 — /doc check 增强
- 新增 `.Codex/MEMORY.md` 索引校验：检查所有引用文件是否存在
- 新增 `.Codex/memory/*.md` 内部引用校验
- 新增 `.claude/MEMORY.md` / `CLAUDE.md` 引用校验
- 新增 `.zcode/` topic 文件引用校验（只读检测）
- 新增跨层一致性检查：TODO.md T-xxx 状态 vs `.Codex/memory/` 记录

#### 3. 模式2 — /doc map 增强
- 输出改为三层统一目录树
- 每层标注文件数和读写状态标记

#### 4. 模式3 — /doc sync 增强
- 搜索范围扩大到 `.Codex/memory/*.md`、`.Codex/MEMORY.md`、`.claude/MEMORY.md`
- `.zcode/` 文件扫描并报告（只读，不自动改）
- 保持 `.Codex/MEMORY.md` 索引格式规范

#### 5. 模式4 — /doc todo 增强
- 新增 TODO.md vs `.Codex/memory/` 跨层状态一致性检查
- 新增 TODO.md vs `.zcode/memory_summary.md` 一致性检测

#### 6. 模式5 — /doc update 增强
- 新功能创建 → 提示更新 `.Codex/memory/` + `MEMORY.md` 索引
- TODO 状态变更 → 检测是否需要更新 `.Codex/memory/` 反馈文件
- 会话总结生成 → 自动检查 `.Codex/MEMORY.md` 索引是否需要更新引用

#### 7. 通用约束更新
- 补充各层操作规范
- 明确 `.zcode/` 只读原则
- 明确 `.Codex/MEMORY.md` 格式保护

### 不变部分
- 不修改任何代码文件（.py/.c/.json）
- 不修改 `.claude/CLAUDE.md`（只读）
- 不创建新文件（仅修改 SKILL.md）
- 不引入外部依赖

## 风险点

| 风险 | 级别 | 应对 |
|------|------|------|
| 写入 `.Codex/memory/` 时破坏前导格式 | 低 | 遵循现有文件格式惯例，只追加/替换路径 |
| `.Codex/MEMORY.md` 索引格式复杂 | 低 | 保持 `- [标题](路径) — 描述` 格式，不做语义修改 |
| `.zcode/` 文件格式变化 | 低 | 始终只读检测，不写入 |
| 修改后原有 `/doc check` 行为退化 | 中 | 修改后需验证 docs/ 原有检查逻辑不受影响 |

## 验证方式

1. **桌面验证**：修改后运行 `/doc check`，确认 docs/ 原有检查正常，新增跨层项目也触发
2. **桌面验证**：运行 `/doc map`，确认三层统一视图正确显示
3. **桌面验证**：运行 `/doc todo`，确认跨层一致性检查生效
4. **人工审查**：通读 SKILL.md，确认所有模式的流程描述清晰完整
5. **Qoder 二审**：实施完成后提交 Implementation Review
