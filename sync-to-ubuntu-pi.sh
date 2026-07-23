#!/bin/sh
# sync-to-ubuntu-pi.sh — 将本机 drone_control/ 同步到 ubuntu-pi 上的 FJJ/
# 用法:
#   ./sync-to-ubuntu-pi.sh basic      只同步精简版 -> FJJ/basic/
#   ./sync-to-ubuntu-pi.sh original   只同步全功能版 -> FJJ/original/
#   ./sync-to-ubuntu-pi.sh all        两者都同步（默认）
#
# 只同步核心运行文件（*.py / *.txt），不递归复制整个目录，
# 避免带上 __pycache__ / .ipynb_checkpoints。

HOST="ubuntu-pi"
REMOTE="/home/sunrise/Desktop/FJJ"
LOCAL="drone_control"
TARGET="${1:-all}"

sync_basic() {
    echo "同步 basic/ -> $HOST:$REMOTE/basic/ ..."
    scp "$LOCAL/basic/"*.py "$LOCAL/basic/"*.txt "$HOST:$REMOTE/basic/" 2>/dev/null
    scp "$LOCAL/basic/Lcode/"*.py "$HOST:$REMOTE/basic/Lcode/" 2>/dev/null
}

sync_original() {
    echo "同步 全功能版 -> $HOST:$REMOTE/original/ ..."
    scp "$LOCAL/original/"*.py "$LOCAL/original/"*.txt "$HOST:$REMOTE/original/" 2>/dev/null
    scp "$LOCAL/original/Lcode/"*.py "$HOST:$REMOTE/original/Lcode/" 2>/dev/null
}

sync_competition_2026() {
    echo "同步 competition_2026/ -> $HOST:$REMOTE/competition_2026/ ..."
    ssh "$HOST" "mkdir -p $REMOTE/competition_2026/Lcode"
    scp "$LOCAL/competition_2026/"*.py "$LOCAL/competition_2026/"*.txt "$LOCAL/competition_2026/"*.json "$HOST:$REMOTE/competition_2026/" 2>/dev/null
    scp "$LOCAL/competition_2026/Lcode/"*.py "$HOST:$REMOTE/competition_2026/Lcode/" 2>/dev/null
}

case "$TARGET" in
    basic) sync_basic ;;
    original) sync_original ;;
    comp|competition) sync_competition_2026 ;;
    all) sync_basic; sync_original; sync_competition_2026 ;;
    *) echo "用法: $0 [basic|original|comp|all]"; exit 1 ;;
esac

echo "清理远程缓存并修正属主..."
ssh "$HOST" "rm -rf $REMOTE/basic/__pycache__ $REMOTE/basic/Lcode/__pycache__ $REMOTE/original/__pycache__ $REMOTE/original/Lcode/__pycache__ $REMOTE/competition_2026/__pycache__ $REMOTE/competition_2026/Lcode/__pycache__ 2>/dev/null; chown -R sunrise:sunrise $REMOTE"

echo "✅ 同步完成（记得去板子上 cd ~/Desktop/FJJ && git add -A && git commit 保存一版）"
