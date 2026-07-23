---
name: feedback-codex-review-rigor
description: 用户会用Codex审查Claude的回答和代码质量，需要保持严谨
metadata: 
  node_type: memory
  type: feedback
  originSessionId: be844c9c-0401-4be6-b727-d2430c131d4c
---

用户会用 Codex 审查这个项目里 Claude 给出的回答和代码质量。

**Why**：用户在2026-07-09明确告知这一点，作为对"保持严谨、追求高质量"的要求提出。存在外部审查意味着代码里的逻辑bug、无声失效的分支、未经验证的数值结论都可能被指出来，不能满足于"能跑起来"。

**How to apply**：
- 写新代码（尤其是数据分析类脚本、数值计算逻辑）后，如果条件允许应自己先跑合成数据/边界case验证一遍，而不是只做语法检查就交付。
- 对写出的解读性文字（比如"残差多大算正常"这类阈值判断）要注明局限性和适用条件，不要给出未经验证就下的断言。
- 交付前主动复查一遍逻辑边界（长度检查、除零、索引对齐等），这类问题曾经真实出现过（见近期为转盘T265分析脚本写的 `analyze_turntable_rotation.py` 时发现的死代码bug和非原子读取问题）。
