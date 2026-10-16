#!/usr/bin/env python3
"""把 PCD 文件发布为 sensor_msgs/PointCloud2，便于在 RViz 中可视化。

用法:
    python3 tools/pcd_cloud_publisher.py <file.pcd> [--topic /map701/points]
                                         [--frame-id map] [--rate 0.2]
                                         [--align src/sentry_planner/config/map_alignment.yaml]

实现要点:
- 自己解析 PCD 头（FIELDS/SIZE/TYPE/COUNT/WIDTH/HEIGHT/POINTS/DATA），不依赖 pcl_ros。
  支持 DATA ascii 与 DATA binary（binary_compressed 未实现，会直接报错退出）。
- 按 PCD 的字段布局构造 numpy 结构化数组，`_` 前缀字段与 COUNT>1 都按原字节布局保留，
  所以 msg.data 是内存直拷，字段偏移与 PCD 完全一致。
- QoS 用 TRANSIENT_LOCAL + RELIABLE(depth=1)：后启动的 RViz 也能拿到已发布点云，
  无需 RViz 先订阅；同时按 --rate 周期重发，避免 subscriber 抖动丢帧。
- `--align`：官方 map.pcd 在「官方录制时的 odin 起点系」里，真车本次 odom 原点未必重合。
  给了 map_alignment.yaml（tools/calib_align.py 产出）就按 p_odom = R(dyaw_deg)*p_map + [dx,dy]
  把 x/y 变换后再发布，这样 RViz 里点云与导航栅格(/nav_map) 能叠在一起。
"""

import argparse
import os
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

import nav_map  # noqa: E402  (只需要它的 map->odom 变换与 yaml 解析)

# (TYPE, SIZE) -> (numpy dtype, PointCloud2 PointField datatype)
_FIELD_TYPES = {
    ("F", 4): ("<f4", PointField.FLOAT32),
    ("F", 8): ("<f8", PointField.FLOAT64),
    ("I", 1): ("i1", PointField.INT8),
    ("I", 2): ("<i2", PointField.INT16),
    ("I", 4): ("<i4", PointField.INT32),
    ("U", 1): ("u1", PointField.UINT8),
    ("U", 2): ("<u2", PointField.UINT16),
    ("U", 4): ("<u4", PointField.UINT32),
}


def _read_header(stream):
    """读取 PCD 头，返回 (dict, data_kind, 数据起始偏移)。"""
    header = {}
    offset = 0
    with open(stream, "rb") as fh:
        for raw in fh:
            offset += len(raw)
            line = raw.decode("ascii", errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            header[key.upper()] = value.strip()
            if key.upper() == "DATA":
                break
    if "DATA" not in header:
        raise ValueError(f"{stream}: 没有找到 DATA 行，不是合法的 PCD 文件")
    return header, header["DATA"].lower(), offset


def load_pcd(path):
    """解析 PCD，返回 (结构化 numpy 数组, width, height)。"""
    header, data_kind, data_offset = _read_header(path)

    names = header["FIELDS"].split()
    sizes = [int(s) for s in header["SIZE"].split()]
    types = header["TYPE"].split()
    counts = [int(c) for c in header.get("COUNT", " ".join("1" * len(names))).split()]
    if not (len(names) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f"{path}: FIELDS/SIZE/TYPE/COUNT 长度不一致")

    width = int(header["WIDTH"])
    height = int(header.get("HEIGHT", 1))
    points = int(header.get("POINTS", width * height))

    # numpy 结构化 dtype：未命名字段(_)与 COUNT>1 都保持原始字节布局
    dtype_list = []
    fields = []
    for i, (name, size, ptype, count) in enumerate(zip(names, sizes, types, counts)):
        if not (ptype, size) in _FIELD_TYPES:
            raise ValueError(f"{path}: 暂不支持的字段类型 TYPE={ptype} SIZE={size}")
        np_type, ros_type = _FIELD_TYPES[(ptype, size)]
        if name.startswith("_"):
            dtype_list.append((f"__pad{i}", f"V{size * count}"))
            continue
        if count > 1:
            dtype_list.append((name, np_type, (count,)))
        else:
            dtype_list.append((name, np_type))
        fields.append((name, ros_type, size, count))

    dtype = np.dtype(dtype_list)

    if data_kind == "ascii":
        rows = [line.split() for line in _iter_ascii_rows(path, data_offset)]
        values = np.array(rows, dtype="<f8")
        if values.shape[0] != points:
            raise ValueError(f"{path}: 期望 {points} 行数据，实际 {values.shape[0]}")
        arr = np.zeros(points, dtype=dtype)
        col = 0
        for entry, (name, _, _, count) in zip(dtype_list, zip(names, sizes, types, counts)):
            field = entry[0]
            if field.startswith("__pad"):
                continue
            arr[field] = values[:, col] if count == 1 else values[:, col : col + count]
            col += count
    elif data_kind == "binary":
        with open(path, "rb") as fh:
            fh.seek(data_offset)
            buf = fh.read()
        expect = points * dtype.itemsize
        if len(buf) < expect:
            raise ValueError(f"{path}: 二进制数据不足，期望 {expect} 字节，实际 {len(buf)}")
        arr = np.frombuffer(buf, dtype=dtype, count=points)
    else:
        raise ValueError(f"{path}: 不支持 DATA {data_kind}（仅支持 ascii / binary）")

    arr = arr.reshape(height, width)
    return arr, width, height, fields


def _iter_ascii_rows(path, data_offset):
    with open(path, "rb") as fh:
        fh.seek(data_offset)
        for raw in fh:
            line = raw.decode("ascii", errors="replace").strip()
            if line and not line.startswith("#"):
                yield line


class PcdPublisher(Node):
    def __init__(self, path, topic, frame_id, rate, align=None):
        super().__init__("pcd_cloud_publisher")
        arr, width, height, fields = load_pcd(path)
        if align is not None:
            names = arr.dtype.names or ()
            if "x" not in names or "y" not in names:
                raise ValueError(f"{path}: --align 需要点云有 x/y 字段（实际 {names}）")
            arr = arr.copy()  # binary 路径下 arr 是文件缓冲的只读视图，必须复制再改
            ox, oy = nav_map.align_points(arr["x"], arr["y"], align)
            arr["x"] = ox.astype(arr["x"].dtype)
            arr["y"] = oy.astype(arr["y"].dtype)
        self._msg = self._build_msg(arr, width, height, fields, frame_id)
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(PointCloud2, topic, qos)
        self._publish()
        if rate > 0.0:
            self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"{path} -> {topic}  frame_id={frame_id}  "
            f"{width}x{height} 字段={[f[0] for f in fields]}  "
            f"align={nav_map.format_alignment(align)}"
        )

    def _build_msg(self, arr, width, height, fields, frame_id):
        msg = PointCloud2()
        msg.header = Header(frame_id=frame_id)
        msg.height = height
        msg.width = width
        msg.fields = [
            PointField(name=name, offset=0, datatype=ros_type, count=count)
            for name, ros_type, _, count in fields
        ]
        # PointField.offset 必须按实际内存布局填
        offset = 0
        for field, (name, _, size, count) in zip(msg.fields, fields):
            field.offset = offset
            offset += size * count
        msg.point_step = arr.dtype.itemsize
        msg.row_step = msg.point_step * width
        msg.data = np.ascontiguousarray(arr).tobytes()
        msg.is_dense = True
        for axis in ("x", "y", "z"):
            if axis in (arr.dtype.names or ()) and not np.isfinite(arr[axis]).all():
                msg.is_dense = False
                break
        return msg

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(argv=None):
    parser = argparse.ArgumentParser(description="把 PCD 发布成 PointCloud2")
    parser.add_argument("pcd", help="PCD 文件路径")
    parser.add_argument("--topic", default="/map701/points", help="发布话题名")
    parser.add_argument("--frame-id", default="map", help="PointCloud2 的 frame_id")
    parser.add_argument(
        "--rate", type=float, default=0.2, help="重发频率 Hz，0 表示只发一次"
    )
    parser.add_argument(
        "--align",
        default="",
        help="map_alignment.yaml：把点云按 p_odom = R(dyaw)*p_map + [dx,dy] 变换后再发布"
        "（默认不变换，即按 PCD 原始坐标系）",
    )
    args = parser.parse_args(argv)

    try:
        align = nav_map.resolve_alignment(args.align or None, use_default=False)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    rclpy.init()
    node = PcdPublisher(args.pcd, args.topic, args.frame_id, args.rate, align)
    try:
        rclpy.spin(node)
    # Ctrl-C / SIGTERM 会被 rclpy 的信号处理转成 ExternalShutdownException，
    # view_nav_map.sh 退出时也会走到这里，所以一起吞掉，免得现场看到一大段 traceback。
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
