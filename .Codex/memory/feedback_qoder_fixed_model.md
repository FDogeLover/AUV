# Qoder 固定审查模型

- 用户于 2026-07-23 明确要求：Qoder 不得使用 `Auto`。
- 所有 Qoder Plan Review、Implementation Review、preflight、statemachine 和其他审查调用，都必须在命令中显式指定：

  ```powershell
  -m "Qwen3.8-Max-Preview"
  ```

- 不得因响应慢而静默切换到其他模型；若指定模型不可用，应报告用户。
