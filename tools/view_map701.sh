#!/usr/bin/env bash
# 在 RViz2 里可视化 701 场地点云地图。
#
#   bash tools/view_map701.sh [map.pcd]
#
# 默认使用 src/sentry_planner/map/701_model/map.pcd：
#   1) 后台跑 tools/pcd_cloud_publisher.py，把 PCD 发成 /map701/points (PointCloud2, frame_id=map)
#   2) 前台跑 rviz2 -d tools/map701.rviz
# 退出时自动结束后台的发布进程。
# 注意：不要用 set -u，ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES）
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PCD="${1:-${REPO_ROOT}/src/sentry_planner/map/701_model/map.pcd}"
RVIZ_CONFIG="${SCRIPT_DIR}/map701.rviz"
ROS_DISTRO="${ROS_DISTRO:-kilted}"

if [[ ! -f "${PCD}" ]]; then
    echo "找不到点云文件: ${PCD}" >&2
    exit 1
fi
if [[ ! -f /opt/ros/"${ROS_DISTRO}"/setup.bash ]]; then
    echo "找不到 /opt/ros/${ROS_DISTRO}/setup.bash" >&2
    exit 1
fi

# shellcheck disable=SC1090
source /opt/ros/"${ROS_DISTRO}"/setup.bash

python3 "${SCRIPT_DIR}/pcd_cloud_publisher.py" "${PCD}" \
    --topic /map701/points --frame-id map &
PUB_PID=$!

# 只为消除 RViz "Fixed Frame [map] does not exist" 警告：给 map 一个父坐标系
ros2 run tf2_ros static_transform_publisher \
    --frame-id world --child-frame-id map &
TF_PID=$!

trap 'kill ${PUB_PID} ${TF_PID} 2>/dev/null || true' EXIT

rviz2 -d "${RVIZ_CONFIG}"
