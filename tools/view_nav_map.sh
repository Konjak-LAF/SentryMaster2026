#!/usr/bin/env bash
# 一键在 RViz2 里同时看：701 场地点云 + 导航栅格 /nav_map + 坡道掩码 /nav_map_slope。
#
#   bash tools/view_nav_map.sh [map.pcd] [map_alignment.yaml|none]
#
# 默认：
#   1) tools/pcd_cloud_publisher.py 把 PCD 发成 /map701/points（frame_id=odom）
#      若 src/sentry_planner/config/map_alignment.yaml 存在（tools/calib_align.py 现场产出），
#      自动加 --align 把点云按 p_odom = R(dyaw)*p_map + [dx,dy] 变到 odom 系，和栅格叠得上；
#      第 2 个参数可以指定别的对齐文件，写 none 表示不对齐。
#   2) tools/publish_nav_map.py 把 nav_map.yaml/pgm 发成 /nav_map 与 /nav_map_slope
#      （栅格在建图时就已经在 odom 系里，这里不需要再变换）
#   3) 前台 rviz2 -d tools/nav_map_overlay.rviz（Fixed Frame = odom，三者同帧，不需要 TF）
# 退出（Ctrl-C / 关窗口）时 trap 把两个后台进程一起收掉。
# 注意：不要用 set -u，ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES）
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

MAP_DIR="${REPO_ROOT}/src/sentry_planner/map/701_model"
PCD="${1:-${MAP_DIR}/map.pcd}"
MAP_YAML="${MAP_DIR}/nav_map.yaml"
RVIZ_CONFIG="${SCRIPT_DIR}/nav_map_overlay.rviz"
ROS_DISTRO="${ROS_DISTRO:-kilted}"
ALIGN_FILE="${2:-${REPO_ROOT}/src/sentry_planner/config/map_alignment.yaml}"

if [[ ! -f "${PCD}" ]]; then
    echo "找不到点云文件: ${PCD}" >&2
    exit 1
fi
if [[ ! -f "${MAP_YAML}" ]]; then
    echo "找不到 ${MAP_YAML}，先跑: python3 tools/build_nav_map.py" >&2
    exit 1
fi
if [[ ! -f /opt/ros/"${ROS_DISTRO}"/setup.bash ]]; then
    echo "找不到 /opt/ros/${ROS_DISTRO}/setup.bash" >&2
    exit 1
fi

ALIGN_ARGS=()
if [[ -n "${ALIGN_FILE}" && "${ALIGN_FILE}" != "none" && -f "${ALIGN_FILE}" ]]; then
    ALIGN_ARGS=(--align "${ALIGN_FILE}")
    echo "[对齐] 点云按 ${ALIGN_FILE} 变换到 odom 系"
else
    echo "[对齐] 无（点云按 PCD 原始坐标系发布；想让点云与栅格对齐就用 --align / 传第 2 个参数）"
fi

# shellcheck disable=SC1090
source /opt/ros/"${ROS_DISTRO}"/setup.bash

python3 "${SCRIPT_DIR}/pcd_cloud_publisher.py" "${PCD}" \
    --topic /map701/points --frame-id odom "${ALIGN_ARGS[@]}" &
PUB_PID=$!

python3 "${SCRIPT_DIR}/publish_nav_map.py" \
    --map-yaml "${MAP_YAML}" --topic /nav_map --slope-topic /nav_map_slope &
MAP_PID=$!

trap 'kill ${PUB_PID} ${MAP_PID} 2>/dev/null || true' EXIT

rviz2 -d "${RVIZ_CONFIG}"
