#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_chain.py —— 用「低速小陀螺」当刺激，逐段验证数据通路

干什么
------
在**不移动底盘**的前提下（MODE_HOLD 原地 + 小陀螺），沿整条链路打一针已知信号，
逐段核对每一跳的数据与符号：

  本工具 --NavCommand(spin)--> 导航 --VelocityCommand(wz)--> uart_driver
         --0x20 帧--> 电控 --回传--> /chassis_data
                                     底盘实际转动 --> /odin1/odometry_highfreq 的 yaw

核对项（每项都有实测值、容差与 PASS/FAIL）：

  1. 导航是否收到并转发了 spin（`/sentry/velocity_command` 的 wz ≈ 指令值，含爬升时间）
  2. 电控是否回传了同样的 wz（`/chassis_data.wz`，与指令同号同量级）
  3. 底盘是否**真的朝正方向**转（odom 的 yaw 角速度 ≈ 指令值）
  4. 停止指令后是否在超时内停下
  5. 裁判/里程计数据是否全程新鲜（数据通路没断）

再配合 `--spin -90` 跑一次反向，专抓「wz 正方向/符号」这类约定错误。

用法
----
  # 台架/无车：本工具自己模拟底盘（积分 /sentry/velocity_command），真导航照常被驱动
  python3 tools/check_chain.py --simulate --spin 90

  # 真车：车轮离地或场地空旷、人抓急停；低速（默认上限 180 deg/s）
  python3 tools/check_chain.py --spin 90
  python3 tools/check_chain.py --spin -90          # 反向，验证符号约定
  python3 tools/check_chain.py --spin 90 --json /tmp/chain.json

  # 自检工具本身能抓错：让模拟底盘反向响应，符号检查必须 FAIL
  python3 tools/check_chain.py --simulate --spin 90 --simulate-sign -1

安全 / 前提
-----------
* `--spin` 绝对值默认上限 180 deg/s（`--allow-fast` 才放开）；只发 MODE_HOLD，不产生平移指令。
* 真车上请车轮离地或留出 ≥1 m 半径；本工具结束时会发 MODE_STOP 并等转速落下来。
* 真车模式不模拟任何数据；缺话题就报缺（不会自己补数据制造假 PASS）。
* **必须独占** `/sentry/nav_command`：决策节点也在发同一话题时本工具会检测到并中止
  （先 `pkill -f sentry_decision_node`，或只起导航节点）。

退出码：0 = 全部 PASS；1 = 有 FAIL；2 = 参数/安全拒绝
"""
import argparse
import json
import math
import sys
import time


def _now() -> float:
    return time.monotonic()


def _mean(values):
    return sum(values) / len(values) if values else float("nan")


def _sign(value: float) -> int:
    return 1 if value > 0 else (-1 if value < 0 else 0)


class Sim:
    """无车模式：把 /sentry/velocity_command 积分成 odom + tf，并回发 /chassis_data。"""

    def __init__(self, node, args, state):
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rm_msgs.msg import ChassisData

        self.node = node
        self.args = args
        self.state = state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last = None
        self.pub_odom = node.create_publisher(Odometry, args.odom_topic, 10)
        self.pub_chassis = node.create_publisher(ChassisData, args.chassis_topic, 10)
        from tf2_ros import TransformBroadcaster
        self.broadcaster = TransformBroadcaster(node)
        node.create_timer(1.0 / args.sim_rate, self.tick)

    def tick(self) -> None:
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rm_msgs.msg import ChassisData

        now = _now()
        if self.last is None:
            self.last = now
        dt = min(max(0.0, now - self.last), 0.1)
        self.last = now
        vx, vy, wz = self.state["cmd"]
        if now - self.state["cmd_stamp"] > 1.0:      # 与真机一致：没指令就停
            vx = vy = wz = 0.0
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (cos_yaw * vx - sin_yaw * vy) * 1e-3 * dt
        self.y += (sin_yaw * vx + cos_yaw * vy) * 1e-3 * dt
        self.yaw += math.radians(wz * self.args.simulate_sign) * dt
        self.state["sim_pose"] = (self.x, self.y, self.yaw)

        stamp = self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.args.global_frame
        odom.child_frame_id = self.args.robot_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = math.hypot(vx, vy) * 1e-3
        odom.twist.twist.angular.z = math.radians(wz)
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.args.global_frame
        tf.child_frame_id = self.args.robot_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.broadcaster.sendTransform(tf)

        chassis = ChassisData()
        chassis.vx = float(vx)
        chassis.vy = float(vy)
        chassis.wz = float(wz)
        self.pub_chassis.publish(chassis)


def run(args) -> int:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    from rm_msgs.msg import ChassisData, NavCommand, RefereeData, VelocityCommand

    rclpy.init()
    node = Node("check_chain")
    state = {"cmd": (0.0, 0.0, 0.0), "cmd_stamp": 0.0,
             "nav_seen": [], "foreign_nav": 0,
             "chassis_wz": None, "chassis_stamp": None,
             "referee_gt": None, "referee_stamp": None,
             "odom": None, "odom_stamp": None, "sim_pose": None,
             "yaw": [], "odom_gaps": [], "chassis_gaps": []}

    pub_nav = node.create_publisher(NavCommand, args.nav_topic, 10)

    def on_nav(msg: NavCommand) -> None:
        # 只用于「有没有别人也在发」的冲突检测
        state["nav_seen"].append((_now(), msg.mode, float(msg.spin_rate)))

    def on_vel(msg: VelocityCommand) -> None:
        state["cmd"] = (msg.vx, msg.vy, msg.wz)
        state["cmd_stamp"] = _now()

    def on_chassis(msg: ChassisData) -> None:
        now = _now()
        if state["chassis_stamp"] is not None:
            state["chassis_gaps"].append(now - state["chassis_stamp"])
        state["chassis_stamp"] = now
        state["chassis_wz"] = float(msg.wz)

    def on_referee(msg: RefereeData) -> None:
        state["referee_gt"] = int(msg.game_time)
        state["referee_stamp"] = _now()

    def on_odom(msg: Odometry) -> None:
        now = _now()
        if state["odom_stamp"] is not None:
            state["odom_gaps"].append(now - state["odom_stamp"])
        state["odom_stamp"] = now
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        state["odom"] = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        state["yaw"].append((now, yaw))
        if len(state["yaw"]) > 200:
            del state["yaw"][:100]

    node.create_subscription(NavCommand, args.nav_topic, on_nav, 10)
    node.create_subscription(VelocityCommand, args.vel_topic, on_vel, 10)
    node.create_subscription(ChassisData, args.chassis_topic, on_chassis, 10)
    node.create_subscription(RefereeData, args.referee_topic, on_referee, 10)
    node.create_subscription(Odometry, args.odom_topic, on_odom, 10)

    sim = None

    def pump(seconds: float) -> None:
        end = _now() + max(0.0, seconds)
        while _now() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def yaw_rate(span: float = 0.25):
        """用最近 span 秒的 yaw 变化算角速度（deg/s），带解缠。"""
        samples = state["yaw"]
        if len(samples) < 3:
            return float("nan")
        t_end, y_end = samples[-1]
        ref = None
        for t, y in reversed(samples):
            if t_end - t >= span:
                ref = (t, y)
                break
        if ref is None:
            ref = samples[0]
        total = 0.0
        prev = ref[1]
        for t, y in samples:
            if t < ref[0]:
                continue
            delta = y - prev
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            total += delta
            prev = y
        dt = t_end - ref[0]
        return math.degrees(total) / dt if dt > 1e-3 else float("nan")

    # ---------------------------------------------------------------- 等数据
    mode_note = ""
    explicit_sim = args.simulate
    if not args.real:
        deadline = _now() + args.wait * 0.5
        while _now() < deadline and state["odom_stamp"] is None:
            pump(0.05)
        if explicit_sim or state["odom_stamp"] is None:
            sim = Sim(node, args, state)
            mode_note = "simulate（本工具模拟底盘）"
        else:
            mode_note = "real（用真实里程计）"
    else:
        mode_note = "real（--real 强制）"

    print(f"[chain] 模式: {mode_note}  指令 spin={args.spin:+.0f} deg/s  "
          f"保持 {args.duration:.0f}s，之后发 STOP", flush=True)

    # 冲突检测：决策节点也在发 nav_command 时，本工具的自检没有意义
    pump(1.0)
    own = [item for item in state["nav_seen"]
           if abs(item[2] - args.spin) < 1e-3 and item[1] == NavCommand.MODE_HOLD]
    foreign = len(state["nav_seen"]) - len(own)
    if foreign > 5:
        print(f"[chain] !! 检测到 /sentry/nav_command 上有别的发布者（1s 内 {foreign} 条与本工具不符）"
              f"—— 决策节点在跑？先 pkill -f sentry_decision_node 再跑本工具", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    # 等里程计/底盘/裁判
    deadline = _now() + args.wait
    while _now() < deadline:
        pump(0.05)
        ok_odom = state["odom_stamp"] is not None
        ok_chassis = state["chassis_stamp"] is not None
        ok_ref = state["referee_stamp"] is not None
        if ok_odom and ok_chassis and ok_ref:
            break
    missing = [name for name, flag in (("/odin1/odometry_highfreq", state["odom_stamp"]),
                                       ("/chassis_data", state["chassis_stamp"]),
                                       ("/referee_data", state["referee_stamp"]))
               if flag is None]
    if not args.simulate and missing:
        print(f"[chain] !! 等不到: {', '.join(missing)}（真车请先确认串口/odin 起来了；"
              f"无车请加 --simulate）", file=sys.stderr)

    pose = state["odom"] if state["odom"] is not None else (0.0, 0.0, 0.0)
    target = {"x": pose[0], "y": pose[1], "yaw": pose[2]}

    # ---------------------------------------------------------------- 打刺激
    samples = []
    t_spin_start = None
    end = _now() + args.duration
    while _now() < end:
        msg = NavCommand()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = args.global_frame
        msg.mode = NavCommand.MODE_HOLD
        msg.task_id = NavCommand.TASK_IDLE
        msg.x = float(target["x"])
        msg.y = float(target["y"])
        msg.yaw = float("nan")
        msg.spin_rate = float(args.spin)
        msg.stay_sec = float(args.duration)
        msg.radius = 0.02
        pub_nav.publish(msg)
        pump(1.0 / args.cmd_rate)
        if t_spin_start is None and state["cmd_stamp"] > 0.0 and abs(state["cmd"][2]) > 1.0:
            t_spin_start = _now()
        samples.append({
            "t": _now(),
            "vel_wz": float(state["cmd"][2]),
            "chassis_wz": (float(state["chassis_wz"]) if state["chassis_wz"] is not None
                           else float("nan")),
            "yaw_rate": yaw_rate(),
        })

    # ---------------------------------------------------------------- 发停止
    t_stop = _now()
    stop_deadline = t_stop + args.stop_timeout
    while _now() < stop_deadline:
        msg = NavCommand()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = args.global_frame
        msg.mode = NavCommand.MODE_STOP
        msg.radius = float("nan")
        pub_nav.publish(msg)
        pump(1.0 / args.cmd_rate)
        rate = yaw_rate()
        if not math.isnan(rate) and abs(rate) < max(5.0, abs(args.spin) * 0.1):
            break
    pump(0.4)
    t_stopped = _now()
    stop_elapsed = t_stopped - t_stop
    final_rate = yaw_rate()

    # ---------------------------------------------------------------- 统计
    steady = samples[len(samples) // 2:]
    vel_vals = [s["vel_wz"] for s in steady]
    ch_vals = [s["chassis_wz"] for s in steady if not math.isnan(s["chassis_wz"])]
    rate_vals = [s["yaw_rate"] for s in steady if not math.isnan(s["yaw_rate"])]
    vel_mean = _mean(vel_vals)
    ch_mean = _mean(ch_vals)
    rate_mean = _mean(rate_vals)
    tol = max(5.0, abs(args.spin) * 0.25)

    ramp = float("nan")
    if t_spin_start is not None:
        need = abs(args.spin) * 0.8
        for s in samples:
            if not math.isnan(s["vel_wz"]) and abs(s["vel_wz"]) >= need:
                ramp = s["t"] - t_spin_start
                break

    def _gap_max(gaps):
        return max(gaps) if gaps else 0.0

    odom_rate_hz = (len(state["odom_gaps"]) /
                    max(1e-6, sum(state["odom_gaps"]))) if state["odom_gaps"] else 0.0
    chassis_rate_hz = (len(state["chassis_gaps"]) /
                       max(1e-6, sum(state["chassis_gaps"]))) if state["chassis_gaps"] else 0.0

    checks = []

    def check(name, ok, detail, skip=False):
        checks.append({"name": name, "ok": bool(ok), "detail": detail, "skip": bool(skip)})

    # 1. 导航转发
    if not math.isnan(vel_mean) and abs(vel_mean) < 1.0 and abs(args.spin) >= 10.0:
        check("导航转发 spin", False,
              f"`/sentry/velocity_command` 的 wz≈0（收到 {vel_mean:+.1f}）：导航没转发 —— 检查是否在"
              f"禁小陀螺区（waypoint_graph.yaml 的 spin_forbidden_zones）、或导航指令冲突/看门狗拦停")
    else:
        check("导航转发 spin", abs(vel_mean - args.spin) <= tol,
              f"wz 实测 {vel_mean:+.1f} vs 指令 {args.spin:+.0f}（容差 ±{tol:.0f}）")

    # 2. 电控回传
    if sim is not None:
        check("电控回传 wz", True,
              f"[SKIP] 模拟底盘回显 {ch_mean:+.1f} —— 这一跳（串口/电控）请用 "
              f"bash tools/bench_uart.sh 验证", skip=True)
    elif not ch_vals:
        check("电控回传 wz", False, "没收到 /chassis_data（串口/电控没起来？）")
    else:
        check("电控回传 wz", abs(ch_mean - args.spin) <= tol,
              f"/chassis_data.wz 实测 {ch_mean:+.1f} vs 指令 {args.spin:+.0f}")

    # 3. 底盘真的转 + 符号
    if math.isnan(rate_mean):
        check("底盘实际转速/符号", False, "测不到 odom 的 yaw 角速度（里程计停更？）")
    else:
        sign_ok = _sign(rate_mean) == _sign(args.spin)
        mag_ok = abs(abs(rate_mean) - abs(args.spin)) <= max(tol, abs(args.spin) * 0.4)
        check("底盘实际转速/符号", sign_ok and mag_ok,
              f"odom yaw 角速度 {rate_mean:+.1f} deg/s（指令 {args.spin:+.0f}）"
              f"{'，符号正确' if sign_ok else '，**符号相反**：查 QA 口径（vx 车体正前 / wz 逆时针为正）与电控侧符号'}")

    # 4. 停得下来
    stop_ok = (not math.isnan(final_rate)) and abs(final_rate) < max(5.0, abs(args.spin) * 0.1)
    check("停止响应", stop_ok,
          f"STOP 后 {stop_elapsed:.2f}s 内 yaw 角速度 {final_rate:+.1f} deg/s"
          f"（阈值 ±{max(5.0, abs(args.spin) * 0.1):.0f}）")

    # 5. 数据新鲜
    odom_ok = mode_note.startswith("simulate") or (odom_rate_hz >= 20.0 and _gap_max(state["odom_gaps"]) <= 0.5)
    ch_ok = mode_note.startswith("simulate") or (chassis_rate_hz >= 5.0 and _gap_max(state["chassis_gaps"]) <= 0.5)
    ref_ok = state["referee_stamp"] is not None
    ref_note = "有" if ref_ok else ("未模拟（真车模式必查）" if sim is not None else "无")
    check("数据通路新鲜度", odom_ok and ch_ok and (ref_ok or sim is not None),
          f"odom {odom_rate_hz:.0f}Hz/最大间隔 {_gap_max(state['odom_gaps']):.2f}s；"
          f"chassis {chassis_rate_hz:.0f}Hz/最大间隔 {_gap_max(state['chassis_gaps']):.2f}s；"
          f"referee {ref_note}",
          skip=(sim is not None and not ref_ok))

    # ---------------------------------------------------------------- 输出
    print()
    print("=" * 96)
    print(f"数据通路自检  spin={args.spin:+.0f} deg/s  模式={mode_note}")
    print("-" * 96)
    for item in checks:
        tag = "SKIP" if item.get("skip") else ("PASS" if item["ok"] else "FAIL")
        print(f"  [{tag}] {item['name']:<16} {item['detail']}")
    print("-" * 96)
    print(f"  实测: vel_wz={vel_mean:+.1f}  chassis_wz={ch_mean:+.1f}  "
          f"yaw_rate={rate_mean:+.1f} deg/s  爬升={ramp if not math.isnan(ramp) else -1:.2f}s  "
          f"停止={stop_elapsed:.2f}s")
    print(f"  速度折算: chassis/指令={ch_mean / args.spin if args.spin else float('nan'):.2f}  "
          f"yaw/指令={rate_mean / args.spin if args.spin else float('nan'):.2f}")
    failures = [item for item in checks if not item["ok"] and not item.get("skip")]
    if failures:
        print(f"  结论: FAIL（{len(failures)} 项）—— 见上面的处置提示")
    else:
        print("  结论: PASS —— 低速小陀螺这一针在整条链路上都能看到，符号方向一致")
    print("=" * 96)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump({"spin": args.spin, "mode": mode_note, "checks": checks,
                       "measured": {"vel_wz": vel_mean, "chassis_wz": ch_mean,
                                    "yaw_rate": rate_mean, "ramp_s": ramp,
                                    "stop_s": stop_elapsed},
                       "odom_hz": odom_rate_hz, "chassis_hz": chassis_rate_hz},
                      handle, ensure_ascii=False, indent=2)
        print(f"  （结果已写入 {args.json}）")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if not failures else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        prog="check_chain.py",
        description="低速小陀螺刺激下的数据通路逐段自检（NavCommand -> 导航 -> uart -> 电控 -> odom）",
        epilog=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--simulate", action="store_true",
                      help="无车：本工具模拟底盘（无 odom 时自动进入）")
    mode.add_argument("--real", action="store_true", help="强制真车：绝不模拟")
    parser.add_argument("--spin", type=float, default=90.0,
                        help="小陀螺转速 deg/s（逆时针为正，默认 90）")
    parser.add_argument("--duration", type=float, default=10.0, help="匀速段时长 s（默认 10）")
    parser.add_argument("--cmd-rate", type=float, default=10.0, help="NavCommand 重发频率 Hz（默认 10）")
    parser.add_argument("--sim-rate", type=float, default=50.0, help="模拟底盘频率 Hz（默认 50）")
    parser.add_argument("--stop-timeout", type=float, default=4.0, help="停止后的超时 s（默认 4）")
    parser.add_argument("--wait", type=float, default=15.0, help="等话题出现的超时 s（默认 15）")
    parser.add_argument("--allow-fast", action="store_true", help="允许 |spin| > 180 deg/s")
    parser.add_argument("--simulate-sign", type=float, default=1.0,
                        help="自检用：模拟底盘的响应符号（-1 用来验证工具能抓到符号错）")
    parser.add_argument("--json", default="", help="把结果写到该 json 文件")
    parser.add_argument("--nav-topic", default="/sentry/nav_command")
    parser.add_argument("--vel-topic", default="/sentry/velocity_command")
    parser.add_argument("--odom-topic", default="/odin1/odometry_highfreq")
    parser.add_argument("--chassis-topic", default="/chassis_data")
    parser.add_argument("--referee-topic", default="/referee_data")
    parser.add_argument("--global-frame", default="odom")
    parser.add_argument("--robot-frame", default="vehicle")
    args = parser.parse_args()

    if abs(args.spin) > 180.0 and not args.allow_fast:
        print(f"拒绝：|spin|={abs(args.spin):g} > 180 deg/s。低速自检不该超过这个值；"
              f"确实要测就加 --allow-fast", file=sys.stderr)
        return 2
    if abs(args.spin) < 10.0:
        print("拒绝：--spin 太小（<10 deg/s），噪声会盖过结论；低速自检建议 60~120", file=sys.stderr)
        return 2
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
