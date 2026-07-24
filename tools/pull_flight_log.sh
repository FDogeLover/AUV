#!/bin/bash
# 从板子拉取飞行日志并归档到本地
#
# 用法:
#   ./tools/pull_flight_log.sh                         # 拉取 basic 版本
#   ./tools/pull_flight_log.sh basic_radar             # 拉取 basic_radar 版本
#   ./tools/pull_flight_log.sh competition_2026        # 拉取备赛版
#
# 归档位置:
#   板子: ~/Desktop/FJJ/<版本>/test_data_YYYYMMDD/
#   本地: drone_control/tools/data_archive/test_data_YYYYMMDD/

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

BOARD_HOST="ubuntu-pi"
BOARD_BASE="/home/sunrise/Desktop/FJJ"
VERSION="${1:-basic}"
DATE_TAG=$(date +%Y%m%d)

ARCHIVE_DIR="$PROJECT_DIR/drone_control/tools/data_archive"
ARCHIVE_SUBDIR="test_data_${DATE_TAG}"
LOCAL_PATH="$ARCHIVE_DIR/$ARCHIVE_SUBDIR"

echo "=== 拉取 $VERSION 飞行日志 === "

# 1) 板子上先生成测试描述
BOARD_DIR="$BOARD_BASE/$VERSION"
DESC=$(ssh "$BOARD_HOST" "
    cd $BOARD_DIR
    # 从 router.txt 提取路径概要
    if [ -f router.txt ]; then
        awk -F, '{printf \"%.1f_%.1f_\", \$1, \$2}' router.txt | head -c 40
    fi
" 2>/dev/null || echo "unknown")

# 2) 板子上归档（保留副本）
echo "  ① 板子归档..."
ssh "$BOARD_HOST" "
    mkdir -p $BOARD_DIR/test_data_${DATE_TAG}
    if [ -f $BOARD_DIR/flight_data.jsonl ]; then
        cp $BOARD_DIR/flight_data.jsonl $BOARD_DIR/test_data_${DATE_TAG}/flight_data_${DESC}_${DATE_TAG}.jsonl
        cp $BOARD_DIR/router.txt $BOARD_DIR/test_data_${DATE_TAG}/router_${DATE_TAG}.txt 2>/dev/null || true
        echo 'done'
    fi
" 2>/dev/null

# 3) 拉取到本地
echo "  ② 拉取到本地..."
mkdir -p "$LOCAL_PATH"

FILENAME="flight_data_${DESC}_${DATE_TAG}.jsonl"
N=1
while [ -f "$LOCAL_PATH/$FILENAME" ]; do
    N=$((N + 1))
    FILENAME="flight_data_${DESC}_${DATE_TAG}_v${N}.jsonl"
done

scp "$BOARD_HOST:$BOARD_DIR/flight_data.jsonl" "$LOCAL_PATH/$FILENAME" 2>/dev/null
scp "$BOARD_HOST:$BOARD_DIR/router.txt" "$LOCAL_PATH/router_${DATE_TAG}.txt" 2>/dev/null || true

echo "     📁 $LOCAL_PATH/$FILENAME"
echo "     📁 $LOCAL_PATH/router_${DATE_TAG}.txt"

# 4) 清空板子工作日志（为下次准备）
echo "  ③ 清空板子工作日志..."
ssh "$BOARD_HOST" "truncate -s 0 $BOARD_DIR/flight_data.jsonl" 2>/dev/null

echo ""
echo "✅ 完成"
echo ""
echo "分析:"
echo "  python3 tools/flight_log_analyzer.py \"$LOCAL_PATH/$FILENAME\""
echo ""
echo "归档列表:"
ls -1 "$LOCAL_PATH/" 2>/dev/null
