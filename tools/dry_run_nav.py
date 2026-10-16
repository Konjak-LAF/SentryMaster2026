#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dry_run_nav.py —— 用**真实的** planner 无车回放任务航点序列（plan verification / 现场自检）

干什么
------
把任务表 `points:` 里的点位按「阶段顺序」（或 `--seq` 指定顺序）逐个下发给真实的
`sentry_planner_node`（发 rm_msgs/NavCommand 到 `/sentry/nav_command`），
逐段等待到位并测量：目标点 vs 实际到达点、最终距离、耗时、PASS/FAIL，最后打印汇总表；
任一段 FAIL 则退出码非 0。

- `--simulate`（`/odin1/odometry_highfreq` 没有数据时的默认）：把 planner 发的
  `/sentry/velocity_command`（车体系 mm/s + deg/s）按当前 yaw 做欧拉积分（dt 取墙上时钟），
  合成 `nav_msgs/Odometry` 发到 `/odin1/odometry_highfreq`，并发布 tf
  `odom→odin1_base_link`（动态）+ `odin1_base_link→vehicle`（静态，数值取自
  `sentry_launch/urdf/sentry_chassis.xacro`，与真机 robot_state_publisher 一致），
  于是「决策→导航→（假）底盘」整条链路可以零硬件跑通。
- `--real`（`/odin1/odometry_highfreq` 有数据时的默认）：完全不模拟，只观察真实位姿。

用法
----
    # 另一个终端先起 planner：
    #   source /opt/ros/kilted/setup.bash && source install/setup.bash
    #   ros2 run sentry_planner sentry_planner_node --ros-args \
    #       --params-file install/sentry_planner/share/sentry_planner/config/global_planning.yaml

    python3 tools/dry_run_nav.py --simulate                     # 整局序列回放
    python3 tools/dry_run_nav.py --simulate --seq supply,central_high --leg-timeout 20
    python3 tools/dry_run_nav.py --simulate --points-file /tmp/t.yaml --hold-sec 0.5
    python3 tools/dry_run_nav.py --real --leg-timeout 60        # 真车只看不模拟

现场打分点检查清单需要的行（起点/终点位姿、逐段误差、总耗时）都会打印在汇总里。

注意
----
- 本工具只发 NavCommand，**不发** `/sentry/velocity_command`，可以与其它工具并存；
  但 `--simulate` 会在 odom/tf 上顶替真机，真机在跑时不要用 `--simulate`。
- 升级后的 planner 有 `require_map` / `require_chassis_data` 两个门禁：`require_map=true` 且
  路径图 `waypoint_graph.yaml` 为空（还没现场打点）时会 **拒绝运动**。台架/仿真这样起 planner：
      ros2 run sentry_planner sentry_planner_node --ros-args \
        --params-file install/sentry_planner/share/sentry_planner/config/global_planning.yaml \
        -p require_map:=false -p require_chassis_data:=false
  （`--simulate` 下本工具会回发 `/chassis_data`，所以 `require_chassis_data` 也可以留 true。）
- 未打点（坐标 `.nan`）的点位会记为 SKIP（不计入失败，与决策节点跳过 NaN 阶段一致）。
- 点位半径取任务表的 `radius`；缺失/非法时用 `--arrive-radius`。
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_POINTS = os.path.join(REPO_ROOT, "src/sentry_decision_sys/config/task_table.yaml")
DEFAULT_URDF = os.path.join(REPO_ROOT, "src/sentry_launch/urdf/sentry_chassis.xacro")


# --------------------------------------------------------------------------- #
# 纯几何工具（不依赖 ROS，便于 --help / 单测）
# --------------------------------------------------------------------------- #
def quat_to_matrix(q):
    """四元数 (x, y, z, w) -> 3x3 旋转矩阵。"""
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def matrix_to_quat(r):
    """3x3 旋转矩阵 -> 四元数 (x, y, z, w)。"""
    m = np.asarray(r, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def rpy_to_quat(roll, pitch, yaw):
    """固定轴 RPY（ROS 约定 R = Rz*Ry*Rx）-> 四元数 (x, y, z, w)。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_to_yaw(q):
    """四元数 (x, y, z, w) -> yaw（rad）。"""
    x, y, z, w = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_to_rpy(q):
    """四元数 (x, y, z, w) -> (roll, pitch, yaw)，ROS 约定。"""
    r = quat_to_matrix(q)
    sy = math.hypot(r[0, 0], r[1, 0])
    if sy > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def make_tf(translation, quat):
    """(x,y,z) + 四元数 -> 4x4 齐次变换。"""
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(quat)
    t[:3, 3] = np.asarray(translation, dtype=float)
    return t


def inv_tf(t):
    """4x4 齐次变换求逆。"""
    out = np.eye(4)
    r = t[:3, :3]
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ t[:3, 3]
    return out


def wrap_pi(a):
    return math.remainder(a, 2.0 * math.pi)


def is_finite(v):
    return v is not None and math.isfinite(float(v))


def rclpy_ok():
    """rclpy 上下文是否还活着（Ctrl-C/关停后会变成 False）。"""
    try:
        import rclpy

        return bool(rclpy.ok())
    except Exception:
        return False


class Log:
    """日志适配：rclpy 的 logger.info(message) 不接受 %-style 参数（kilted 实测），这里自己格式化。"""

    def __init__(self, logger):
        self._logger = logger

    @staticmethod
    def _render(fmt, args):
        if not args:
            return str(fmt)
        try:
            return fmt % args
        except (TypeError, ValueError):
            return f"{fmt} {args}"

    def info(self, fmt, *args):
        self._logger.info(self._render(fmt, args))

    def warning(self, fmt, *args):
        self._logger.warning(self._render(fmt, args))

    def error(self, fmt, *args):
        self._logger.error(self._render(fmt, args))


def _num(v):
    """YAML 值 -> float（None/非法 -> nan）。"""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return float("nan")
    return f


def parse_urdf_offset(path, logger=None):
    """从 `sentry_chassis.xacro` 里取 odin1_base_link→vehicle 的固定变换；失败则用 2.md 记录值。"""
    fallback = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    try:
        with open(path, "r", encoding="utf-8") as fh:
            text = fh.read()
    except OSError:
        if logger:
            logger.warning("URDF %s 读不到，使用内置 odin 安装偏置", path)
        return fallback
    import re

    origin = None
    for m in re.finditer(r"<origin\s+([^/>]*)/?>", text):
        attrs = dict(re.findall(r'(\w+)\s*=\s*"([^"]*)"', m.group(1)))
        if "xyz" in attrs and "rpy" in attrs:
            origin = attrs
            break
    if origin is None:
        if logger:
            logger.warning("URDF %s 里没找到 <origin xyz rpy>，使用内置 odin 安装偏置", path)
        return fallback
    xyz = tuple(float(v) for v in origin["xyz"].split())
    rpy = tuple(float(v) for v in origin["rpy"].split())
    return xyz, rpy


# --------------------------------------------------------------------------- #
# 任务表解析
# --------------------------------------------------------------------------- #
def load_task_table(path):
    with open(path, "r", encoding="utf-8") as fh:
        table = yaml.safe_load(fh) or {}
    if not isinstance(table, dict):
        raise ValueError(f"{path} 顶层不是 map")
    points = {}
    for name, spec in (table.get("points") or {}).items():
        if not isinstance(spec, dict):
            continue
        points[str(name)] = {
            "x": _num(spec.get("x")),
            "y": _num(spec.get("y")),
            "yaw_deg": _num(spec.get("yaw_deg")),
            "radius": _num(spec.get("radius")),
        }
    return table, points


def point_meta(table):
    """点位 -> {task_id, stay_sec, spin}：取第一个引用它的阶段（pre_point 或 point）。"""
    ids = table.get("task_ids") or {}
    meta = {}
    for ph in table.get("phases") or []:
        if not isinstance(ph, dict):
            continue
        for key in ("pre_point", "point"):
            name = ph.get(key)
            if not isinstance(name, str) or name in meta:
                continue
            meta[name] = {
                "task_id": int(_num(ids.get(name, ph.get("task_id", 0))) or 0),
                "stay_sec": _num(ph.get("stay_sec", 0.0)),
                "spin": bool(ph.get("spin_travel") or ph.get("spin_hold")),
            }
    return meta


def default_sequence(table):
    """阶段顺序 -> 航点序列：每阶段 = pre_point + point；与上一阶段展开完全相同的阶段折叠掉。"""
    seq = []
    prev = None
    for ph in table.get("phases") or []:
        if not isinstance(ph, dict):
            continue
        expand = [n for n in (ph.get("pre_point"), ph.get("point")) if isinstance(n, str)]
        if not expand or expand == prev:
            continue
        prev = expand
        seq.extend(expand)
    return seq


# --------------------------------------------------------------------------- #
# 主逻辑
# --------------------------------------------------------------------------- #
class DryRun:
    """把 planner 下发的速度指令积分成假底盘，并按航点序列下发 NavCommand 做回放。"""

    def __init__(self, node, args, table, points, seq, logger, msgs):
        self.node = node
        self.args = args
        self.table = table
        self.points = points
        self.seq = seq
        self.log = logger
        self.msg_nav, self.msg_vel, self.msg_odom, self.msg_chassis = msgs
        self.mode_stop = self.msg_nav.MODE_STOP
        self.mode_goto = self.msg_nav.MODE_GOTO
        self.mode_hold = self.msg_nav.MODE_HOLD
        self.meta = point_meta(table)

        self.sim = None                # True / False，等发现里程计后确定
        self.pose_vehicle = None       # 4x4 odom->vehicle（sim 下为积分结果）
        self.sim_pose = make_tf((0.0, 0.0, 0.0), rpy_to_quat(0.0, 0.0, math.radians(args.start[2])))
        self.sim_pose[:3, 3] = np.asarray([args.start[0], args.start[1], 0.0])
        self.cmd = (0.0, 0.0, 0.0)     # 最近一次 planner 发出的车体系速度
        self.cmd_stamp = 0.0
        self.vel_seen = 0
        self.cmd_zero_since = None
        self.last_step = time.monotonic()
        self.odom_seen = 0
        self.odom_msg = None
        self.tf = None
        self.stopping = False          # SIGINT/SIGTERM：只置标志，不再下发指令
        self.interrupted = False
        self._send_error_logged = False

        xyz, rpy = parse_urdf_offset(args.urdf, logger)
        self.odin_offset = make_tf(xyz, rpy_to_quat(*rpy))

        self._create_interfaces()

    # ------------------------------------------------------------------ #
    def request_stop(self, signum=None, frame=None):
        """SIGINT/SIGTERM 处理：只置标志，由各循环在 ~20 ms 内优雅退出。

        这里**不发任何指令**（现场 Ctrl-C 时 rclpy 上下文随时可能被关掉，
        在信号处理里发指令正是会抛 RCLError 的写法）；planner 有 1 s 指令超时，会自行停车。
        """
        self.stopping = True
        self.interrupted = True

    def install_signal_handlers(self):
        """接管 SIGINT/SIGTERM（rclpy.init 装的那个会先关上下文，导致收尾报错）。"""
        import signal
        import threading

        if threading.current_thread() is not threading.main_thread():
            return
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, self.request_stop)
            except (ValueError, OSError):
                pass

    def _spin_once(self, timeout=0.02):
        """安全版 spin_once：中断/上下文关停时返回 False，绝不抛异常。"""
        if self.stopping:
            return False
        try:
            import rclpy

            rclpy.spin_once(self.node, timeout_sec=timeout)
            return True
        except KeyboardInterrupt:
            self.request_stop()
            return False
        except Exception as exc:
            if not self.stopping:
                self.log.warning("spin 中断（%s）：按关停处理", exc)
            self.stopping = True
            return False

    # ------------------------------------------------------------------ #
    def _create_interfaces(self):
        """订阅/发布（sim 相关的 publisher/timer 在 _setup_mode 里建）。"""
        self.pub_nav = self.node.create_publisher(self.msg_nav, self.args.nav_topic, 10)
        self.node.create_subscription(self.msg_odom, self.args.odom_topic, self._on_odom, 10)
        self.node.create_subscription(self.msg_vel, self.args.vel_topic, self._on_velocity, 10)

    def _on_odom(self, msg):
        self.odom_seen += 1
        self.odom_msg = msg

    def _on_velocity(self, msg):
        self.cmd = (float(msg.vx), float(msg.vy), float(msg.wz))
        self.cmd_stamp = time.monotonic()
        self.vel_seen += 1
        if abs(self.cmd[0]) < 1e-3 and abs(self.cmd[1]) < 1e-3 and abs(self.cmd[2]) < 1e-3:
            if self.cmd_zero_since is None:
                self.cmd_zero_since = self.cmd_stamp
        else:
            self.cmd_zero_since = None

    # ------------------------------------------------------------------ #
    def setup_mode(self):
        """等 --discovery-wait 秒看真里程计有没有数据，决定 sim / real。"""
        import rclpy

        if self.args.simulate:
            self.sim = True
        elif self.args.real:
            self.sim = False
        else:
            deadline = time.monotonic() + self.args.discovery_wait
            while time.monotonic() < deadline and self.odom_seen == 0:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            self.sim = self.odom_seen == 0
        if self.sim and self.odom_seen > 0 and self.args.simulate:
            self.log.warning(
                "%s 上已经有 %d 帧里程计（真机/另一个 sim 在跑？），--simulate 会顶替它，"
                "注意 tf 冲突", self.args.odom_topic, self.odom_seen,
            )

        if self.sim:
            from geometry_msgs.msg import TransformStamped
            from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

            self.pose_vehicle = self.sim_pose.copy()   # 起点就是 --start，先给个有效位姿
            self.pub_odom = self.node.create_publisher(self.msg_odom, self.args.odom_topic, 10)
            # 升级后的 planner 默认 require_chassis_data:=true（底盘/串口看门狗）：
            # 假底盘回发「指令值」（理想底盘，无动力学），于是不需要关掉那个检查。
            self.pub_chassis = self.node.create_publisher(
                self.msg_chassis, self.args.chassis_topic, 10)
            self.tf_pub = TransformBroadcaster(self.node)
            self.static_tf_pub = StaticTransformBroadcaster(self.node)
            self.static_tfs = []
            if self.args.tf_style == "chain":
                # 与真机一致：odin 发 odom→odin1_base_link，robot_state_publisher 发 odin1_base_link→vehicle
                st = TransformStamped()
                st.header.stamp = self.node.get_clock().now().to_msg()
                st.header.frame_id = self.args.odin_frame
                st.child_frame_id = self.args.robot_frame
                (st.transform.translation.x, st.transform.translation.y,
                 st.transform.translation.z) = self.odin_offset[:3, 3]
                q = matrix_to_quat(self.odin_offset[:3, :3])
                (st.transform.rotation.x, st.transform.rotation.y,
                 st.transform.rotation.z, st.transform.rotation.w) = q
                self.static_tfs.append(st)
            # direct：不发静态 tf（odom→vehicle 由 _step 每拍动态发，两者同时发会打架）
            if self.static_tfs:
                self.static_tf_pub.sendTransform(self.static_tfs)
            self.node.create_timer(1.0 / self.args.sim_rate, self._step)
        else:
            from tf2_ros import Buffer, TransformListener

            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self.node)

    # ------------------------------------------------------------------ #
    def _step(self):
        """sim：按当前 yaw 欧拉积分速度指令，发 odom + tf odom→odin1_base_link。"""
        from geometry_msgs.msg import TransformStamped

        now_wall = time.monotonic()
        dt = now_wall - self.last_step
        self.last_step = now_wall
        dt = max(0.0, min(dt, 0.2))
        vx, vy, wz = self.cmd
        if now_wall - self.cmd_stamp > 1.0:
            vx = vy = wz = 0.0   # planner 没在发指令：停车（与真机行为一致）
        yaw = quat_to_yaw(matrix_to_quat(self.sim_pose[:3, :3]))
        self.sim_pose[:3, 3] += dt * np.array([
            vx / 1000.0 * math.cos(yaw) - vy / 1000.0 * math.sin(yaw),
            vx / 1000.0 * math.sin(yaw) + vy / 1000.0 * math.cos(yaw),
            0.0,
        ])
        yaw = wrap_pi(yaw + math.radians(wz) * dt)
        self.sim_pose[:3, :3] = quat_to_matrix(rpy_to_quat(0.0, 0.0, yaw))
        self.pose_vehicle = self.sim_pose.copy()

        stamp = self.node.get_clock().now().to_msg()
        odom_pose = self.sim_pose @ inv_tf(self.odin_offset)   # odom→odin1_base_link
        if self.args.tf_style == "chain":
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.args.global_frame
            tf.child_frame_id = self.args.odin_frame
            tf.transform.translation.x = float(odom_pose[0, 3])
            tf.transform.translation.y = float(odom_pose[1, 3])
            tf.transform.translation.z = float(odom_pose[2, 3])
            q = matrix_to_quat(odom_pose[:3, :3])
            (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w) = q
            self.tf_pub.sendTransform(tf)
        else:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.args.global_frame
            tf.child_frame_id = self.args.robot_frame
            tf.transform.translation.x = float(self.sim_pose[0, 3])
            tf.transform.translation.y = float(self.sim_pose[1, 3])
            tf.transform.translation.z = float(self.sim_pose[2, 3])
            q = matrix_to_quat(self.sim_pose[:3, :3])
            (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w) = q
            self.tf_pub.sendTransform(tf)

        msg = self.msg_odom()
        msg.header.stamp = stamp
        msg.header.frame_id = self.args.global_frame
        child = self.args.odin_frame if self.args.tf_style == "chain" else self.args.robot_frame
        msg.child_frame_id = child
        src = odom_pose if self.args.tf_style == "chain" else self.sim_pose
        msg.pose.pose.position.x = float(src[0, 3])
        msg.pose.pose.position.y = float(src[1, 3])
        msg.pose.pose.position.z = float(src[2, 3])
        q = matrix_to_quat(src[:3, :3])
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[0], q[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[2], q[3]
        self.pub_odom.publish(msg)

        ch = self.msg_chassis()
        ch.vx, ch.vy, ch.wz = float(vx), float(vy), float(wz)
        self.pub_chassis.publish(ch)

    # ------------------------------------------------------------------ #
    def pose(self):
        """odom→vehicle 的 4x4 位姿；sim 直接用积分结果，real 先查 tf 再退到底盘里程计。"""
        if self.sim:
            return self.pose_vehicle
        if self.tf is not None:
            try:
                tr = self.tf.lookup_transform(
                    self.args.global_frame, self.args.robot_frame, self.node.get_clock().now()
                )
            except Exception:
                tr = None
            if tr is not None:
                t = tr.transform.translation
                q = tr.transform.rotation
                return make_tf((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        if self.odom_msg is None:
            return None
        pose = self.odom_msg.pose.pose
        t_odin = make_tf(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        )
        if self.odom_msg.child_frame_id == self.args.robot_frame:
            return t_odin
        if self.odom_msg.child_frame_id == self.args.odin_frame:
            return t_odin @ self.odin_offset   # 自己补 URDF 那段固定变换
        return None

    # ------------------------------------------------------------------ #
    def send(self, mode, x, y, yaw, spin_rate, stay_sec, task_id, radius=float("nan")):
        """下发 NavCommand；已中断/上下文关停时直接跳过（返回 False），失败只记一次日志。"""
        if self.stopping or not rclpy_ok():
            return False
        try:
            msg = self.msg_nav()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.args.global_frame
            msg.mode = mode
            msg.task_id = task_id
            msg.x = float(x)
            msg.y = float(y)
            msg.yaw = float(yaw)
            msg.spin_rate = float(spin_rate)
            msg.stay_sec = float(stay_sec)
            msg.radius = float(radius)
            self.pub_nav.publish(msg)
            return True
        except Exception as exc:      # RCLError / InvalidHandle：关停竞态，不升级成崩溃
            if not self._send_error_logged:
                self._send_error_logged = True
                self.log.warning("下发 NavCommand 失败（上下文关停？）：%s", exc)
            return False

    # ------------------------------------------------------------------ #
    def wait_arrival(self, x, y, radius, timeout, publish):
        """等到位（连续 settle 秒在 radius 内）或超时/中断。返回 (ok, pose, dist, elapsed)。"""
        period = 1.0 / self.args.cmd_rate
        t0 = time.monotonic()
        inside_since = None
        pose = None
        dist = float("nan")
        warned = False
        last_pub = -1e9
        while True:
            if self.stopping:
                return False, pose, dist, time.monotonic() - t0
            self._spin_once(0.02)
            now = time.monotonic()
            if now - last_pub >= period:
                publish()
                last_pub = now
            if not warned and now - t0 > 2.0:
                if self.vel_seen == 0:
                    warned = True
                    self.log.warning(
                        "还没收到任何 %s 帧：planner 起了吗？话题名对得上吗？"
                        "（升级后的 planner 在 require_map / require_chassis_data 不满足时会"
                        "[fault] 拒绝运动——台架/仿真加 -p require_map:=false -p require_chassis_data:=false；"
                        "本机 DDS 发现偶发 ~50%% 失败，重跑或设 FASTRTPS_DEFAULT_PROFILES_FILE 可解）",
                        self.args.vel_topic,
                    )
                elif now - self.cmd_stamp > 2.0:
                    warned = True
                    self.log.warning("planner 已经 %.1f s 没发 %s", now - self.cmd_stamp, self.args.vel_topic)
                elif self.cmd_zero_since is not None and now - self.cmd_zero_since > 1.0:
                    warned = True
                    self.log.warning(
                        "planner 在持续发零速（已 %.1f s）：NavCommand 可能没送到 / planner 被 "
                        "require_map 或底盘看门狗拦停 / 它自己认为已到位——看 planner 日志的 [nav] 行核对",
                        now - self.cmd_zero_since,
                    )
            pose = self.pose()
            if pose is not None:
                dist = math.hypot(float(pose[0, 3]) - x, float(pose[1, 3]) - y)
                if dist <= radius + 1e-6:
                    if inside_since is None:
                        inside_since = now
                    elif now - inside_since >= self.args.settle:
                        return True, pose, dist, now - t0
                else:
                    inside_since = None
            if now - t0 > timeout:
                return False, pose, dist, now - t0

    def hold(self, x, y, yaw, stay, spin_rate, task_id, radius=float("nan")):
        """HOLD 驻留 stay 秒，返回期间离目标点的最大偏移（中断时提前返回）。"""
        def publish():
            self.send(self.mode_hold, x, y, yaw, spin_rate, stay, task_id, radius)

        period = 1.0 / self.args.cmd_rate
        t0 = time.monotonic()
        last_pub = -1e9
        max_drift = 0.0
        while True:
            if self.stopping:
                return max_drift
            self._spin_once(0.02)
            now = time.monotonic()
            if now - last_pub >= period:
                publish()
                last_pub = now
            pose = self.pose()
            if pose is not None:
                max_drift = max(max_drift, math.hypot(float(pose[0, 3]) - x, float(pose[1, 3]) - y))
            if now - t0 >= stay:
                return max_drift

    # ------------------------------------------------------------------ #
    def fmt_pose(self, pose):
        if pose is None:
            return "n/a"
        roll, pitch, yaw = quat_to_rpy(matrix_to_quat(pose[:3, :3]))
        return f"x={pose[0, 3]:.3f} y={pose[1, 3]:.3f} yaw={math.degrees(yaw):.2f} deg"

    def run_leg(self, idx, name):
        tag = f"[leg {idx}/{len(self.seq)}]"
        pt = self.points.get(name)
        base = {"index": idx, "point": name, "result": None, "err": float("nan"),
                "time": 0.0, "hold": 0.0, "drift": float("nan"), "note": ""}
        if self.stopping:
            return dict(base, result="ABORT", note="interrupted")
        if pt is None:
            self.log.error("%s 点位 %r 不在 %s 的 points: 里 -> FAIL", tag, name, self.args.points_file)
            return dict(base, result="FAIL", note="point missing from table")
        if not (is_finite(pt["x"]) and is_finite(pt["y"])):
            self.log.warning("%s 点位 %s 坐标未打点(.nan) -> SKIP", tag, name)
            return dict(base, result="SKIP", note="unmarked (.nan)")
        x, y = float(pt["x"]), float(pt["y"])
        radius = float(pt["radius"]) if (is_finite(pt["radius"]) and pt["radius"] > 0.0) else self.args.arrive_radius
        yaw = math.radians(float(pt["yaw_deg"])) if is_finite(pt["yaw_deg"]) else float("nan")
        meta = self.meta.get(name, {})
        task_id = int(meta.get("task_id", 0))
        spin = float(self.args.spin_rate) if meta.get("spin") else 0.0
        stay = float(self.args.hold_sec) if self.args.hold_sec is not None else float(meta.get("stay_sec", 0.0))
        if not math.isfinite(stay) or stay < 0.0:
            stay = 0.0
        self.log.info(
            "%s GOTO %-16s target=(%.3f, %.3f) yaw=%s r=%.2f task_id=%d%s",
            tag, name, x, y,
            "nan" if math.isnan(yaw) else f"{math.degrees(yaw):.1f}deg",
            radius, task_id, f" spin={spin:.0f}deg/s" if spin else "",
        )

        def publish():
            self.send(self.mode_goto, x, y, yaw, spin, 0.0, task_id, radius)

        ok, pose, dist, elapsed = self.wait_arrival(x, y, radius, self.args.leg_timeout, publish)
        reached = "n/a" if pose is None else f"({float(pose[0, 3]):.3f}, {float(pose[1, 3]):.3f})"
        if self.stopping:
            result = "ABORT"
        else:
            result = "PASS" if ok else "FAIL"
        self.log.info(
            "%s %s %-16s target=(%.3f, %.3f) reached=%s dist=%.3f m elapsed=%.2f s",
            tag, result, name, x, y, reached, dist, elapsed,
        )
        row = dict(base, result=result, err=dist, time=elapsed, x=x, y=y, radius=radius,
                   pose=reached)
        if ok and stay > 0.0 and not self.stopping:
            drift = self.hold(x, y, yaw, stay, spin, task_id, radius)
            row["hold"] = stay
            row["drift"] = drift
            self.log.info("%s HOLD %.1f s：离目标最大偏移 %.3f m", tag, stay, drift)
        return row

    def wait_for_pose(self, timeout):
        """等到第一次拿到 odom->vehicle（real 模式刚起时 tf 还没到）。"""
        import rclpy

        end = time.monotonic() + max(0.0, timeout)
        pose = self.pose()
        while pose is None and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            pose = self.pose()
        return pose

    def safe_stop(self):
        """收尾急停：只在正常收尾且 rclpy 还活着时发；关停竞态一律吞掉。"""
        if self.stopping or not rclpy_ok():
            self.log.info("已中断：不再下发指令（planner 有 1 s 指令超时，会自行停车）")
            return False
        if not self.send(self.mode_stop, 0.0, 0.0, float("nan"), 0.0, 0.0, 0):
            return False
        end = time.monotonic() + 0.3
        while time.monotonic() < end and not self.stopping and rclpy_ok():
            self._spin_once(0.05)
        return True

    def run(self):
        self.install_signal_handlers()
        start_pose = self.wait_for_pose(0.0 if self.sim else self.args.discovery_wait)
        if not self.sim and start_pose is None:
            self.log.warning(
                "读不到 %s->%s 的位姿：%s 没有数据（先起 odin 驱动 / robot_state_publisher），"
                "或改用 --simulate",
                self.args.global_frame, self.args.robot_frame, self.args.odom_topic,
            )
        self.log.info(
            "模式: %s   起点 %s->%s: %s",
            "simulate（假底盘积分）" if self.sim else "real（只看真实位姿）",
            self.args.global_frame, self.args.robot_frame, self.fmt_pose(start_pose),
        )
        self.plan_table()
        rows = []
        t_all = time.monotonic()
        try:
            for idx, name in enumerate(self.seq, 1):
                if self.stopping:
                    break
                row = self.run_leg(idx, name)
                rows.append(row)
                if row["result"] == "FAIL" and self.args.stop_on_fail:
                    self.log.warning("--stop-on-fail：第 %d 段失败，提前收工", idx)
                    break
        except KeyboardInterrupt:
            self.request_stop()
        total = time.monotonic() - t_all
        try:
            self.safe_stop()
        except Exception as exc:                      # 关停竞态：绝不把 traceback 甩给现场
            self.log.warning("收尾停车失败（关停中）：%s", exc)
        end_pose = self.pose()
        if self.interrupted:
            self.log.warning("已中断（Ctrl-C/SIGTERM）：回放停在 %d/%d 段，打印已完成段落汇总",
                             len(rows), len(self.seq))
        self.print_summary(rows, start_pose, end_pose, total)
        if self.interrupted:
            return 130
        return 1 if any(r["result"] == "FAIL" for r in rows) else 0

    def plan_table(self):
        self.log.info("回放序列 (%d 段): %s", len(self.seq), " -> ".join(self.seq))
        for i, name in enumerate(self.seq, 1):
            pt = self.points.get(name)
            if pt is None:
                self.log.info("  %2d. %-16s  <缺失>", i, name)
            elif not (is_finite(pt["x"]) and is_finite(pt["y"])):
                self.log.info("  %2d. %-16s  未打点(.nan)", i, name)
            else:
                self.log.info(
                    "  %2d. %-16s (%.3f, %.3f) r=%s", i, name, pt["x"], pt["y"],
                    f"{pt['radius']:.2f}" if is_finite(pt["radius"]) else "default",
                )

    def print_summary(self, rows, start_pose, end_pose, total):
        n_pass = sum(1 for r in rows if r["result"] == "PASS")
        n_fail = sum(1 for r in rows if r["result"] == "FAIL")
        n_skip = sum(1 for r in rows if r["result"] == "SKIP")
        n_abort = sum(1 for r in rows if r["result"] == "ABORT")
        out = []
        out.append("=" * 100)
        out.append(f"回放汇总 / DRY-RUN SUMMARY   模式={'simulate' if self.sim else 'real'}   "
                   f"points={self.args.points_file}")
        out.append(f"起点 {self.args.global_frame}->{self.args.robot_frame}: {self.fmt_pose(start_pose)}")
        out.append(f"{'#':>3} {'point':<16} {'target (m)':<20} {'reached (m)':<20} "
                   f"{'err(m)':>7} {'time(s)':>8} {'hold(s)':>8} {'drift(m)':>9}  result")
        for r in rows:
            if "x" in r:
                target = f"({r['x']:.3f}, {r['y']:.3f})"
            else:
                target = "-"
            reached = r.get("pose", "-")
            out.append(
                f"{r['index']:>3} {r['point']:<16} {target:<20} {reached:<20} "
                f"{r['err']:>7.3f} {r['time']:>8.2f} {r['hold']:>8.1f} "
                f"{r['drift']:>9.3f}  {r['result']}"
            )
        out.append(f"终点 {self.args.global_frame}->{self.args.robot_frame}: {self.fmt_pose(end_pose)}")
        out.append(f"总耗时: {total:.2f} s    段数 {len(rows)} = PASS {n_pass} / FAIL {n_fail} / "
                   f"SKIP {n_skip} / ABORT {n_abort}")
        if self.interrupted:
            out.append(f"自检结论: 已中断（Ctrl-C/SIGTERM）——停在 {len(rows)}/{len(self.seq)} 段，"
                       "上面是已完成段落；planner 靠 1 s 指令超时自行停车（退出码 130）")
        elif n_fail:
            out.append("自检结论: 失败——存在超时/未到位段，见上面 FAIL 行（退出码 1）")
        elif n_pass == 0:
            out.append("自检结论: 全部 SKIP（点位未打点），本次只跑通了链路（退出码 0）")
        else:
            out.append("自检结论: 通过——每段都进入了 radius 之内（退出码 0）")
        out.append("=" * 100)
        print("\n".join(out))
        sys.stdout.flush()


# --------------------------------------------------------------------------- #
def build_parser():
    p = argparse.ArgumentParser(
        prog="dry_run_nav.py",
        description="用真实 planner 无车回放任务航点序列并逐段测量到位误差/耗时",
        epilog=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--points-file", default=DEFAULT_POINTS,
                   help=f"任务表 yaml（读 points: / phases: / task_ids:），默认 {DEFAULT_POINTS}")
    p.add_argument("--seq", default=None,
                   help="逗号分隔的点位名，覆盖默认访问顺序（默认由 phases: 推出）")
    p.add_argument("--arrive-radius", type=float, default=0.20,
                   help="点位 radius 缺失时的到位半径，m（默认 0.20）")
    p.add_argument("--leg-timeout", type=float, default=40.0, help="单段超时，s（默认 40）")
    p.add_argument("--settle", type=float, default=0.5,
                   help="需要在 radius 内连续保持的时长，s（默认 0.5）")
    p.add_argument("--hold-sec", type=float, default=None,
                   help="到位后的 HOLD 驻留时长，s（默认用任务表该点所在阶段的 stay_sec；0 = 不驻留）")
    p.add_argument("--spin-rate", type=float, default=0.0,
                   help="小陀螺角速度 deg/s（仅对任务表标了 spin 的阶段生效，默认 0=不转）")
    p.add_argument("--cmd-rate", type=float, default=10.0,
                   help="NavCommand 重发频率 Hz（planner command_timeout=1 s，必须周期重发；默认 10）")
    p.add_argument("--stop-on-fail", action="store_true", help="任一段 FAIL 立即结束回放")
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--simulate", action="store_true",
                      help="强制假底盘：积分 /sentry/velocity_command 并发布里程计/tf")
    mode.add_argument("--real", action="store_true",
                      help="强制真实模式：绝不模拟，只读真实里程计/tf")
    p.add_argument("--discovery-wait", type=float, default=2.0,
                   help="启动时等里程计出现的时间，s（默认 2.0）；超时未见数据则用 --simulate 行为")
    p.add_argument("--sim-rate", type=float, default=50.0, help="假底盘积分/发布频率 Hz（默认 50）")
    p.add_argument("--start", default="0,0,0",
                   help='sim 起点 "x,y,yaw_deg"（默认 "0,0,0"，即 odom 原点）')
    p.add_argument("--tf-style", choices=("chain", "direct"), default="chain",
                   help="chain: odom→odin1_base_link(动态) + odin1_base_link→vehicle(静态 URDF)，与真机一致；"
                        "direct: 直接发 odom→vehicle")
    p.add_argument("--urdf", default=DEFAULT_URDF,
                   help=f"取 odin1_base_link→vehicle 固定变换的 URDF/xacro，默认 {DEFAULT_URDF}")
    p.add_argument("--nav-topic", default="/sentry/nav_command", help="NavCommand 话题")
    p.add_argument("--odom-topic", default="/odin1/odometry_highfreq", help="里程计话题")
    p.add_argument("--vel-topic", default="/sentry/velocity_command", help="planner 速度指令话题（sim 输入）")
    p.add_argument("--chassis-topic", default="/chassis_data",
                   help="sim 下回发底盘反馈（ChassisData）的话题名，供 planner 的底盘看门狗使用")
    p.add_argument("--global-frame", default="odom", help="世界系（planner global_frame）")
    p.add_argument("--robot-frame", default="vehicle", help="机器人系（planner robot_frame）")
    p.add_argument("--odin-frame", default="odin1_base_link", help="odin 里程计子坐标系名")
    return p


def parse_start(text):
    parts = [s.strip() for s in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('--start 需要 "x,y,yaw_deg"')
    return tuple(float(v) for v in parts)


def main(argv=None):
    args = build_parser().parse_args(argv)
    args.start = parse_start(args.start)
    if args.sim_rate <= 0.0 or args.cmd_rate <= 0.0:
        print("--sim-rate/--cmd-rate 必须 > 0", file=sys.stderr)
        return 2
    if not os.path.isfile(args.points_file):
        print(f"任务表不存在: {args.points_file}", file=sys.stderr)
        return 2
    try:
        table, points = load_task_table(args.points_file)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"任务表解析失败: {exc}", file=sys.stderr)
        return 2

    if args.seq:
        seq = [s.strip() for s in args.seq.split(",") if s.strip()]
    else:
        seq = default_sequence(table)
    if not seq:
        print(f"回放序列为空（{args.points_file} 里既没有 phases: 也没有 --seq）", file=sys.stderr)
        return 2

    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rm_msgs.msg import ChassisData, NavCommand, VelocityCommand
    except ImportError as exc:
        print(
            f"ROS 环境不可用: {exc}\n"
            "请先 source /opt/ros/kilted/setup.bash && source install/setup.bash",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = rclpy.create_node("dry_run_nav")
    logger = Log(node.get_logger())
    runner = None
    code = 1
    try:
        runner = DryRun(node, args, table, points, seq, logger,
                        (NavCommand, VelocityCommand, Odometry, ChassisData))
        runner.setup_mode()
        code = runner.run()
    except KeyboardInterrupt:
        # 兜底：run() 内部已接管信号；万一在更早的位置收到 Ctrl-C 就不发指令、只报一句
        code = 130
        logger.warning("已中断（Ctrl-C）：不再下发指令，planner 靠 1 s 指令超时自行停车")
    except Exception as exc:      # 不把 traceback 甩给现场操作员
        code = 1
        logger.error("回放异常结束：%s", exc)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return code


if __name__ == "__main__":
    sys.exit(main())
