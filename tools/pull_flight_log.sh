#!/bin/bash
# 从板子拉取飞行日志并归档到本地
#
# 用法:
#   ./tools/pull_flight_log.sh                         # 拉取 basic 版本日志
#   ./tools/pull_flight_log.sh basic_radar             # 拉取 basic_radar 版本日志
#   ./tools/pull_flight_log.sh competition_2026        # 拉取备赛版日志
#
# 归档位置:
#   drone_control/tools/data_archive/test_data_YYYYMMDD/flight_data_<描述>_YYYYMMDD.jsonl

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 配置
BOARD_HOST="ubuntu-pi"
BOARD_BASE="/home/sunrise/Desktop/FJJ"
VERSION="${1:-basic}"  # 默认 basic
ARCHIVE_DIR="$PROJECT_DIR/drone_control/tools/data_archive"

# 生成日期标签
DATE_TAG=$(date +%Y%m%d)

# 从板子拉取
echo "=== 从板子拉取 $VERSION 版本日志 ==="
DATA_DIR="$BOARD_BASE/$VERSION"

# 找最新的 flight_data.jsonl
LATEST_LOG=$(ssh "$BOARD_HOST" "ls -t $DATA_DIR/flight_data.jsonl 2>/dev/null || echo ''")
if [ -z "$LATEST_LOG" ]; then
    echo "❌ 板子上 $DATA_DIR/flight_data.jsonl 不存在"
    exit 1
fi

# 读取当前 router.txt 作为描述参考
ROUTER_DESC=""
if ROUTER=$(ssh "$BOARD_HOST" "cat $DATA_DIR/router.txt 2>/dev/null"); then
    # 取前两个航点生成简短描述
    DESC=$(echo "$ROUTER" | head -3 | tr '\n' ' ' | sed 's/ /_/g' | sed 's/,$//')
    ROUTER_DESC="_${DESC:0:30}"
fi

# 目标文件名
ARCHIVE_SUBDIR="test_data_${DATE_TAG}"
ARCHIVE_PATH="$ARCHIVE_DIR/$ARCHIVE_SUBDIR"
mkdir -p "$ARCHIVE_PATH"

# 如果今天已有归档文件，加序号
FILENAME="flight_data${ROUTER_DESC}_${DATE_TAG}.jsonl"
N=1
while [ -f "$ARCHIVE_PATH/$FILENAME" ]; do
    FILENAME="flight_data${ROUTER_DESC}_${DATE_TAG}_v${N}.jsonl"
    N=$((N + 1))
done

echo "  来源: $BOARD_HOST:$DATA_DIR/flight_data.jsonl"
echo "  归档: $ARCHIVE_PATH/$FILENAME"

# 拉取
scp "$BOARD_HOST:$DATA_DIR/flight_data.jsonl" "$ARCHIVE_PATH/$FILENAME"

# 同时拉取 router.txt 作为参考
scp "$BOARD_HOST:$DATA_DIR/router.txt" "$ARCHIVE_PATH/router_${DATE_TAG}.txt" 2>/dev/null || true

# 清理板子上的日志（可选）
echo ""
echo "  板子上的 flight_data.jsonl 是否要清空？(为下次测试准备)"
echo "  不清也不影响，下次会继续追加"
echo ""
echo "  如需清空: ssh $BOARD_HOST \"truncate -s 0 $DATA_DIR/flight_data.jsonl\""

echo ""
echo "✅ 完成: $ARCHIVE_PATH/$FILENAME"
echo ""
echo "分析:"
echo "  python3 tools/flight_log_analyzer.py \"$ARCHIVE_PATH/$FILENAME\""
