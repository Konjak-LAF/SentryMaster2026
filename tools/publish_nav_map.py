#!/usr/bin/env python3
"""把 `nav_map.yaml` / `nav_map.pgm` 发布成 `nav_msgs/OccupancyGrid`。

- 主话题（默认 `/nav_map`）：读 yaml+pgm（map_server 语义），占据=100、自由=0、未知=-1；
- `--slope-topic`（如 `/nav_map_slope`）：另外发一张坡道/台阶掩码（1 -> 100，其余 0），
  数据来自 `nav_map.bin` 的 `slope` 数组（同目录默认找得到，或用 `--map-bin` 指定）；
- QoS：RELIABLE + KEEP_LAST(1) + TRANSIENT_LOCAL（RViz 后起也能收到），并按 `--rate`
  （默认 0.2 Hz）周期重发，兜住订阅端抖动；
- `frame_id` 默认取 yaml 里的 `frame` 键，没有则 `odom`，可用 `--frame-id` 覆盖。

用法::

    python3 tools/publish_nav_map.py                                   # 0.2 Hz 常驻发布
    python3 tools/publish_nav_map.py --slope-topic /nav_map_slope
    python3 tools/publish_nav_map.py --once --linger 3                 # 只发一帧（测试用）
    python3 tools/publish_nav_map.py --print-only                      # 不 import rclpy，只打印
"""

import argparse
import os
import sys
import time

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(TOOLS_DIR)
DEFAULT_MAP_YAML = os.path.join(REPO_ROOT, "src", "sentry_planner", "map", "701_model", "nav_map.yaml")

if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)

import nav_map  # noqa: E402


def occupancy_data(m):
    """NavMap -> OccupancyGrid.data（int8，行主序，row 0 = 顶行）。"""
    data = np.zeros(m.cost.shape, dtype=np.int8)
    data[m.cost != 0] = 100
    if m.unknown is not None:
        data[np.asarray(m.unknown, dtype=bool)] = -1
    return data


def slope_data(m):
    """NavMap（来自 nav_map.bin）的 slope 掩码 -> OccupancyGrid.data（int8）。"""
    data = np.zeros(m.cost.shape, dtype=np.int8)
    data[np.asarray(m.slope, dtype=np.uint8) != 0] = 100
    return data


def _grid_msg(OccupancyGrid, Pose, Point, Quaternion, Header, m, data, frame_id, stamp):
    msg = OccupancyGrid()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.info.resolution = float(m.resolution)
    msg.info.width = m.width
    msg.info.height = m.height
    pose = Pose()
    pose.position = Point(x=float(m.origin[0]), y=float(m.origin[1]), z=0.0)
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    msg.info.origin = pose
    msg.data = data.reshape(-1).tolist()
    return msg


def describe(m, name, frame_id, topic):
    occupied = int(np.count_nonzero(m.cost))
    total = m.width * m.height
    unknown = int(np.count_nonzero(m.unknown)) if m.unknown is not None else 0
    return (
        f"{name}: topic={topic} frame={frame_id} {m.width}x{m.height} "
        f"r={m.resolution} m/cell origin=({m.origin[0]}, {m.origin[1]}) "
        f"occupied={occupied} free={total - occupied - unknown} unknown={unknown}"
    )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="把 nav_map.yaml/pgm 发布成 nav_msgs/OccupancyGrid",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--map-yaml", default=DEFAULT_MAP_YAML, help="map_server 风格 yaml")
    parser.add_argument("--map-pgm", default="", help="PGM 路径（默认由 yaml 的 image 决定）")
    parser.add_argument("--map-bin", default="", help="nav_map.bin（--slope-topic 用；默认同目录）")
    parser.add_argument("--topic", default="/nav_map", help="占据栅格话题")
    parser.add_argument("--slope-topic", default="", help="坡道/台阶掩码话题（留空=不发）")
    parser.add_argument("--frame-id", default="", help="覆盖 yaml 里的 frame")
    parser.add_argument("--rate", type=float, default=0.2, help="重发频率 Hz（0=只发一帧且不再重发）")
    parser.add_argument("--once", action="store_true", help="只发一帧，linger 后退出")
    parser.add_argument("--linger", type=float, default=2.0, help="--once 模式发完后保持多久 (s)")
    parser.add_argument("--print-only", action="store_true", help="只加载并打印摘要，不启动 ROS 节点")
    args = parser.parse_args(argv)

    if not os.path.exists(args.map_yaml):
        parser.error(f"找不到 {args.map_yaml}（先跑 tools/build_nav_map.py 生成）")
    yaml_path = args.map_yaml
    pgm_path = args.map_pgm or os.path.splitext(yaml_path)[0] + ".pgm"
    m_main = nav_map.load_pgm(pgm_path, yaml_path)
    frame_id = args.frame_id or m_main.frame or "odom"

    m_slope = None
    if args.slope_topic:
        bin_path = args.map_bin or os.path.join(os.path.dirname(os.path.abspath(yaml_path)), "nav_map.bin")
        if not os.path.exists(bin_path):
            parser.error(f"--slope-topic 需要 {bin_path}（nav_map.bin，含 slope 数组）")
        m_slope = nav_map.load(bin_path)
        shape_hw = (m_slope.height, m_slope.width)
        if shape_hw != (m_main.height, m_main.width):
            parser.error(
                f"nav_map.bin 与 pgm 尺寸不一致: bin {shape_hw} vs pgm {(m_main.height, m_main.width)}"
            )

    print(describe(m_main, "map", frame_id, args.topic))
    if m_slope is not None:
        slope_n = int(np.count_nonzero(m_slope.slope))
        print(describe(m_slope, "slope", frame_id, args.slope_topic) + f" slope_cells={slope_n}")

    if args.print_only:
        print("--print-only: 未启动 ROS 节点")
        return 0

    import rclpy
    from geometry_msgs.msg import Point, Pose, Quaternion
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Header

    qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    class NavMapPublisher(Node):
        def __init__(self):
            super().__init__("nav_map_publisher")
            self._main_data = occupancy_data(m_main)
            self._slope_data = slope_data(m_slope) if m_slope is not None else None
            self._pubs = [
                (self.create_publisher(OccupancyGrid, args.topic, qos), m_main, self._main_data),
            ]
            if m_slope is not None:
                self._pubs.append(
                    (
                        self.create_publisher(OccupancyGrid, args.slope_topic, qos),
                        m_slope,
                        self._slope_data,
                    )
                )
            self._count = 0
            self.publish_all()
            if args.once:
                self.get_logger().info(f"--once: 发了 1 帧，保持 {args.linger} s 后退出")
            elif args.rate > 0.0:
                self.create_timer(1.0 / args.rate, self.publish_all)
            else:
                self.get_logger().info("--rate 0: 只发了 1 帧（TRANSIENT_LOCAL 常驻），进程继续等待")

        def publish_all(self):
            stamp = self.get_clock().now().to_msg()
            for pub, m, data in self._pubs:
                pub.publish(
                    _grid_msg(OccupancyGrid, Pose, Point, Quaternion, Header, m, data, frame_id, stamp)
                )
            self._count += 1
            if self._count == 1:
                names = [args.topic] + ([args.slope_topic] if m_slope is not None else [])
                self.get_logger().info(f"已发布第 1 帧: topics={names} frame={frame_id}")

    rclpy.init(args=None)
    node = NavMapPublisher()
    try:
        if args.once:
            deadline = time.monotonic() + max(0.0, args.linger)
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    # Ctrl-C / SIGTERM 会被 rclpy 的信号处理转成 ExternalShutdownException，这里一起吞掉，
    # 免得现场看到一大段 traceback。
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001 - 上下文已经关掉时 destroy 会抛
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
