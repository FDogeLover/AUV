#!/bin/sh
# setup-hooks.sh — 安装 git hooks 到本地仓库
#
# 运行:  bash setup-hooks.sh
# 效果:  git config core.hooksPath .githooks

echo "安装 git hooks..."
git config core.hooksPath .githooks
echo "已配置 hooks 路径: $(git config core.hooksPath)"
echo ""
echo "下次 git commit 时自动运行以下检查:"
echo "  1. .c/.h 文件编码一致性"
echo "  2. pi_send() 状态"
echo "  3. Python 语法"
echo "  4. wiringpi 依赖检查"
echo ""
echo "如需临时跳过: git commit --no-verify"
