#!/usr/bin/env bash
# 现场/训练场录包（P4.3 前置）：一条命令录下所有话题 + 顺手导出 tf 树，便于赛后离线复盘。
#
#   bash tools/record_session.sh                 # 录到 /tmp/bag_<时间戳>，Ctrl-C 结束
#   bash tools/record_session.sh /data/run1      # 指定目录（必须不存在）
#
# 产出：
#   <dir>/                      ros2 bag（`ros2 bag info <dir>` 查看）
#   <dir>/frames_<时间>.pdf      tf2_tools 导出的 tf 树（赛后确认 odom->vehicle 链路是否完整）
set -eo pipefail

BAG_DIR="${1:-/tmp/bag_$(date +%m%d_%H%M%S)}"
if [ -e "$BAG_DIR" ]; then
    echo "[record] 目录已存在（ros2 bag record 不接受已存在目录）: $BAG_DIR" >&2
    exit 2
fi

source /opt/ros/kilted/setup.bash
WORKSPACE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

echo "[record] 输出目录: $BAG_DIR"
echo "[record] 磁盘剩余: $(df -h "$(dirname "$BAG_DIR")" | tail -1 | awk '{print $4}')"

ros2 bag record -a -o "$BAG_DIR" &
REC_PID=$!

# 等 bag 目录出现，再抓 tf 树快照（view_frames 采样约 5 s 后在当前目录写出 pdf）
for _ in $(seq 1 30); do
    [ -d "$BAG_DIR" ] && break
    sleep 0.5
done
( cd "$BAG_DIR" 2>/dev/null && ros2 run tf2_tools view_frames > /dev/null 2>&1 ) &

cleanup() {
    # SIGINT 让 ros2 bag 正常收尾（SIGTERM 可能留下不完整的包）
    kill -INT "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
    echo
    echo "[record] 已停止。包大小: $(du -sh "$BAG_DIR" 2>/dev/null | cut -f1)"
    echo "[record] 查看:  ros2 bag info \"$BAG_DIR\""
    echo "[record] tf 树: $(ls "$BAG_DIR"/frames_*.pdf 2>/dev/null | tail -1)"
    echo "[record] 复盘:  ros2 bag play \"$BAG_DIR\" 可离线重放"
}
trap cleanup EXIT INT TERM

wait "$REC_PID"
