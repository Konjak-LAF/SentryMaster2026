#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calib_speed.py —— 速度/功率标定：真车分级跑速度，读功率，产出 speed_limits.yaml

⚠ 本工具**直接驱动底盘**（自己发 `/sentry/velocity_command`），
  **绝对不能与 planner 同时运行**（`sentry_planner_node` 也在发同一个话题，两边会互相打架）。
  标定前请确认：`ros2 node list` 里没有 `sentry_planner_node`（或先把它停掉）。

做什么
------
按 `--stages` 给出的分级顺序逐个跑：每级先按 `--ramp` 秒线性升到目标速度、保持 `--dwell` 秒、
再零速停 `--rest` 秒；全程记录 `/chassis_data` 的实际 vx/vy/wz（均值/峰值），每级打印一行。
功率来源：
  1. 自动：`/chassis_data`、`/referee_data`（或 `--power-topic/--power-field`）里如果有功率字段就自动记录；
  2. 否则（或 `--no-power`）交互提示操作员输入读到的瓦数 / 输入 `c` 表示疑似 100 W 限功率触发。
一旦判定限功率触发（操作员输入 c / 读数 ≥ `--power-limit` / 自动读数 ≥ 限值）：
**立即零速整局中止**（20 s 功率锁定代价太高），并只用已确认安全的数据写 yaml，退出码 2。

最后写 `--out`（默认在运行时解析 `sentry_planner` 的 share 目录，即 launch 读取的那份
`<share>/config/speed_limits.yaml`；同时在仓库 `src/sentry_planner/config/` 里同步一份），
schema 固定为
`/sentry_planner_node: ros__parameters: {max_vx, max_vy, max_wz, max_acc_vx, max_acc_vy, max_acc_wz}`，
取值规则：
  - `max_v*`：该轴**没触发限功率的最高一级**（且实测确实能跑到该速度，≥70% 命令值）——不会超过它；
  - `max_acc_*`：该轴最高安全一级实测的升速能力（0.9×命令值 / 达到该速度的时间）；
  - 全程没有功率数据（`--no-power`、干跑、或没读数）→ 保持默认 800/800/180、1500/1500/360。

无车/可重复测试（headless：不初始化 ROS、不发任何指令、不碰底盘）
-----------------------------------------------------------------
  `--dry-run`：只打印分级表 + 将要写入的 yaml（不落盘）；
  `--script FILE`：从文件读操作员答案（每级一行：瓦数 / `-` / `cutoff`），同样 headless，
                  但会把结果写进 `--out`；两者可组合，于是整条流程零硬件、可脚本化复现。
  真车标定用交互模式（既不加 `--dry-run` 也不加 `--script`），此时才会发布速度指令。

例子
----
    # 零硬件试跑：打印分级表与将要写的 yaml（这里 --script 里给了 3 个读数）
    python3 tools/calib_speed.py --dry-run --script /tmp/calib_answers.txt

    # 真车标定（先确认 planner 没在跑）
    python3 tools/calib_speed.py --csv /tmp/calib_speed.csv
    python3 tools/calib_speed.py --stages "vx=300;vx=600;wz=120;combo=600,120" --dwell 5
"""

import argparse
import csv
import math
import os
import re
import signal
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_out_path():
    """默认输出：运行时解析 sentry_planner 的 share/config/speed_limits.yaml。

    global_planning.launch.py 就是从这个路径当第二份参数文件读它；取不到（没 source ROS、
    或包没装）时退回仓库源码树里的同一个文件。
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("sentry_planner")
        return os.path.join(share, "config", "speed_limits.yaml")
    except Exception:
        return os.path.join(REPO_ROOT, "src/sentry_planner/config/speed_limits.yaml")


DEFAULT_OUT = default_out_path()
DEFAULT_STAGES = "vx=200;vx=400;vx=600;vx=800;vx=1000;vy=200;vy=600;vy=1000;wz=60;wz=120;wz=180;wz=360;combo=600,120"
DEFAULT_LIMITS = {"max_vx": 800.0, "max_vy": 800.0, "max_wz": 180.0,
                  "max_acc_vx": 1500.0, "max_acc_vy": 1500.0, "max_acc_wz": 360.0}
POWER_FIELD_RE = re.compile(r"(power|watt|pwr|current|volt|energy)", re.IGNORECASE)


# --------------------------------------------------------------------------- #
def parse_stages(text):
    """把 `vx=200;vy=600;wz=120;combo=600,120` 解析成 stage 列表。"""
    stages = []
    for raw in re.split(r"[;]", text or ""):
        tok = raw.strip()
        if not tok:
            continue
        if "=" not in tok:
            raise ValueError(f"--stages 片段 {tok!r} 缺少 '='（形如 vx=200 / vy=600 / wz=120 / combo=600,120）")
        axis, _, value = tok.partition("=")
        axis = axis.strip().lower()
        value = value.strip()
        if axis in ("vx", "vy"):
            val = float(value)
            if val <= 0.0:
                raise ValueError(f"{tok!r}: 速度必须 > 0")
            stage = {"axis": axis, "label": f"{axis}={val:g}", "vx": 0.0, "vy": 0.0, "wz": 0.0}
            stage[axis] = val
            stages.append(stage)
        elif axis == "wz":
            val = float(value)
            if val <= 0.0:
                raise ValueError(f"{tok!r}: 角速度必须 > 0")
            stages.append({"axis": axis, "label": f"wz={val:g}", "vx": 0.0, "vy": 0.0, "wz": val})
        elif axis in ("combo", "spin+translate", "spin_translate"):
            parts = [p for p in re.split(r"[,:]", value) if p.strip()]
            if len(parts) != 2:
                raise ValueError(f"{tok!r}: combo 需要 'combo=<vx>,<wz>' 两个值")
            vx, wz = float(parts[0]), float(parts[1])
            stages.append({"axis": "combo", "label": f"combo=vx{vx:g}+wz{wz:g}",
                           "vx": vx, "vy": 0.0, "wz": wz})
        else:
            raise ValueError(f"--stages 不认识轴名 {axis!r}（可用 vx/vy/wz/combo）")
    if not stages:
        raise ValueError("--stages 解析结果为空")
    return stages


def stage_key(stage):
    """用于把测量结果映射回参数字段：vx/vy/wz 单轴，combo 记到两个轴（取最小安全值）。"""
    return stage["axis"]


# --------------------------------------------------------------------------- #
class NodeLogger:
    """rclpy logger 适配：kilted 的 logger.info(message) 不接受 %-style 参数，这里自己格式化。"""

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

    def info(self, fmt, *a):
        self._logger.info(self._render(fmt, a))

    def warning(self, fmt, *a):
        self._logger.warning(self._render(fmt, a))

    def error(self, fmt, *a):
        self._logger.error(self._render(fmt, a))


class PrintLogger:
    """headless（--dry-run/--script）时替代 rclpy logger。"""

    def _emit(self, level, fmt, *a):
        try:
            text = fmt % a if a else str(fmt)
        except (TypeError, ValueError):
            text = f"{fmt} {a}"
        print(f"[calib][{level}] {text}")

    def info(self, fmt, *a):
        self._emit("info", fmt, *a)

    def warning(self, fmt, *a):
        self._emit("WARN", fmt, *a)

    def error(self, fmt, *a):
        self._emit("ERROR", fmt, *a)


class OperatorInput:
    """操作员答案来源：交互 stdin 或 --script 文件（可脚本化、可干跑）。"""

    def __init__(self, script_path=None, logger=None):
        self.logger = logger
        self.lines = None
        self.cursor = 0
        if script_path:
            with open(script_path, "r", encoding="utf-8") as fh:
                self.lines = [ln.strip() for ln in fh]
            self.lines = [ln for ln in self.lines if not ln.startswith("#")]

    def ask(self, prompt):
        if self.lines is None:
            try:
                return input(prompt).strip()
            except EOFError:
                return ""
        if self.cursor < len(self.lines):
            answer = self.lines[self.cursor]
            self.cursor += 1
        else:
            answer = ""
        print(f"{prompt}{answer}   [script]")
        return answer

    def note(self, text):
        if self.lines is not None:
            print(text)


# --------------------------------------------------------------------------- #
class CalibRun:
    """一次标定的状态：测量收集 + 限值推导 + yaml 输出。"""

    def __init__(self, args, stages, operator, logger):
        self.args = args
        self.stages = stages
        self.op = operator
        self.log = logger
        self.rows = []
        self.cutoff = False
        self.cutoff_stage = None
        self.interrupted = False
        self.power_seen = False   # 是否有任何功率读数（自动或手输）

    # ------------------------------------------------------------------ #
    def ask_power(self, stage, auto):
        """返回 (power_mean, power_max, cutoff)。auto 为自动采集到的 (mean, max) 或 None。"""
        limit = self.args.power_limit
        if auto is not None:
            mean, mx = auto
            self.power_seen = True
            if mx >= limit:
                self.log.warning("自动功率峰值 %.1f W ≥ 限值 %.1f W → 判定限功率触发", mx, limit)
                return mean, mx, True
            return mean, mx, False
        if self.args.no_power:
            return None, None, False
        if self.args.dry_run and self.op.lines is None:
            self.op.note("  [功率] --dry-run 且没有 --script：不提示输入，按“无功率数据”处理")
            return None, None, False
        self.op.note(f"  [功率] 读取 {stage['label']} 的功率读数（回车=跳过，c=疑似限功率）")
        answer = self.op.ask("  功率读数 [W] 或 c(限功率) : ")
        if answer == "":
            self.log.info("  无功率读数（跳过该级）")
            return None, None, False
        if answer.lower() in ("c", "cutoff", "cut", "x", "限额", "限功率"):
            self.log.warning("操作员标记限功率触发 -> 立即零速中止")
            return None, None, True
        try:
            value = float(answer)
        except ValueError:
            self.log.warning("读不懂的输入 %r：按“无读数”处理", answer)
            return None, None, False
        self.power_seen = True
        if value >= limit:
            self.log.warning("读数 %.1f W ≥ 限值 %.1f W -> 判定限功率触发", value, limit)
            return value, value, True
        return value, value, False

    # ------------------------------------------------------------------ #
    def record(self, stage, cmd, achieved, power, rise, cutoff, note=""):
        row = {
            "idx": len(self.rows) + 1,
            "stage": stage["label"],
            "axis": stage["axis"],
            "cmd_vx": cmd[0], "cmd_vy": cmd[1], "cmd_wz": cmd[2],
            "ach_vx_mean": achieved["vx_mean"], "ach_vy_mean": achieved["vy_mean"],
            "ach_wz_mean": achieved["wz_mean"],
            "ach_vx_max": achieved["vx_max"], "ach_vy_max": achieved["vy_max"],
            "ach_wz_max": achieved["wz_max"],
            "power_mean": "" if power[0] is None else round(power[0], 2),
            "power_max": "" if power[1] is None else round(power[1], 2),
            "rise_vx_s": "" if rise.get("vx") is None else round(rise["vx"], 3),
            "rise_vy_s": "" if rise.get("vy") is None else round(rise["vy"], 3),
            "rise_wz_s": "" if rise.get("wz") is None else round(rise["wz"], 3),
            "cutoff": int(bool(cutoff)),
            "note": note,
        }
        self.rows.append(row)
        rises = "/".join(f"{k}{v:.2f}s" for k, v in rise.items() if v is not None)

        def m(v):
            return "  n/a " if not math.isfinite(float(v)) else f"{float(v):7.1f}"

        self.log.info(
            "[%2d/%2d] %-18s cmd=(vx %.0f, vy %.0f, wz %.0f)  "
            "实测 mean/max=(vx %s/%s vy %s/%s wz %s/%s)  "
            "power=%s  rise=%s  %s",
            row["idx"], len(self.stages), row["stage"],
            cmd[0], cmd[1], cmd[2],
            m(row["ach_vx_mean"]), m(row["ach_vx_max"]),
            m(row["ach_vy_mean"]), m(row["ach_vy_max"]),
            m(row["ach_wz_mean"]), m(row["ach_wz_max"]),
            "n/a" if row["power_mean"] == "" else f"{row['power_mean']} W",
            rises or "n/a",
            "CUTOFF" if cutoff else "OK",
        )

    # ------------------------------------------------------------------ #
    def derive_limits(self):
        """按测量数据推导 6 个限值（无功率数据则全用默认）。"""
        limits = dict(DEFAULT_LIMITS)
        if not self.power_seen:
            self.log.info("没有任何功率数据 -> 保持默认限值 %s", DEFAULT_LIMITS)
            return limits, {k: "default (no power data)" for k in DEFAULT_LIMITS}

        why = {}
        limit_field = {"vx": "max_vx", "vy": "max_vy", "wz": "max_wz"}
        # ---- 速度上限：该轴“没触发限功率的最高一级”（且实测达到 ≥70% 命令值） ----
        axis_of = {"vx": ("cmd_vx", "ach_vx_mean"), "vy": ("cmd_vy", "ach_vy_mean"),
                   "wz": ("cmd_wz", "ach_wz_mean")}
        for axis, (cmd_key, ach_key) in axis_of.items():
            field = limit_field[axis]
            best = None
            for row in self.rows:
                if row["axis"] not in (axis, "combo") or row["cutoff"]:
                    continue
                cmd = float(row[cmd_key])
                if cmd <= 0.0:
                    continue
                ach = float(row[ach_key])
                if math.isfinite(ach) and ach < 0.7 * cmd:
                    why.setdefault(field + "_hint", f"跳过 {row['stage']}：实测只到 {ach:.0f}（<70%）")
                    continue
                if best is None or cmd > best:
                    best = cmd
            if best is None:
                why[field] = "default (没有该轴安全且可达的分级)"
                continue
            limits[field] = float(best)
            why[field] = f"最高安全分级 {best:g}"
        # ---- 加速度上限：该轴最高安全一级实测的升速能力（0.9×命令值 / 达到该速度的时间） ----
        acc_of = {"vx": "max_acc_vx", "vy": "max_acc_vy", "wz": "max_acc_wz"}
        for axis, field in acc_of.items():
            cands = []
            for row in self.rows:
                if row["axis"] not in (axis, "combo") or row["cutoff"]:
                    continue
                rise = row[f"rise_{axis}_s"]
                cmd = float(row[f"cmd_{axis}"])
                if rise == "" or cmd <= 0.0 or cmd > limits[limit_field[axis]]:
                    continue
                cands.append(0.9 * cmd / max(float(rise), 1e-3))
            if not cands:
                why[field] = "default (没有可测的升速数据)"
                continue
            limits[field] = float(max(cands))
            why[field] = f"最高安全分级实测升速 {max(cands):.0f}"
        return limits, why

    # ------------------------------------------------------------------ #
    def write_yaml(self, limits, why):
        mode = ("dry-run（未落盘）" if self.args.dry_run
                else ("headless 脚本（无底盘）" if self.args.script else "真车"))
        lines = [
            "# speed_limits.yaml —— 真车速度/功率标定结果（tools/calib_speed.py 生成）",
            f"# 生成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}   模式: {mode}",
            f"# 功率上限: {self.args.power_limit:g} W   分级: {';'.join(s['label'] for s in self.stages)}",
            f"# 依据: {'; '.join(f'{k}={v}' for k, v in why.items() if not k.endswith('_hint'))}",
            *[f"# 备注: {v}" for k, v in why.items() if k.endswith("_hint")],
            f"# 判限功率触发: {'是（第 %d 级，已中止）' % self.cutoff_stage if self.cutoff else '否'}",
            "/sentry_planner_node:",
            "  ros__parameters:",
            f"    max_vx: {limits['max_vx']:.1f}",
            f"    max_vy: {limits['max_vy']:.1f}",
            f"    max_wz: {limits['max_wz']:.1f}",
            f"    max_acc_vx: {limits['max_acc_vx']:.1f}",
            f"    max_acc_vy: {limits['max_acc_vy']:.1f}",
            f"    max_acc_wz: {limits['max_acc_wz']:.1f}",
            "",
        ]
        text = "\n".join(lines)
        if self.args.dry_run:
            print("\n---- 将要写入 %s ----" % self.args.out)
            print(text)
            return None
        targets = [self.args.out]
        mirror = self._source_tree_mirror()
        if mirror:
            targets.append(mirror)
        for path in targets:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(text)
            print(f"[calib] 已写入 {path}")
        print(text)
        return self.args.out

    def _source_tree_mirror(self):
        """默认输出落在 install/share 下（launch 从这里读）时，也在仓库源码树里留同一份。"""
        src = os.path.join(REPO_ROOT, "src/sentry_planner/config/speed_limits.yaml")
        out = os.path.abspath(self.args.out)
        if not getattr(self.args, "mirror_source", False) or os.path.abspath(src) == out:
            return None
        if os.path.islink(out) and os.path.realpath(out) == os.path.abspath(src):
            return None   # share 下是软链（symlink-install），写它就是写源码树
        return src

    def write_csv(self, path):
        cols = ["idx", "stage", "axis", "cmd_vx", "cmd_vy", "cmd_wz",
                "ach_vx_mean", "ach_vy_mean", "ach_wz_mean",
                "ach_vx_max", "ach_vy_max", "ach_wz_max",
                "power_mean", "power_max", "rise_vx_s", "rise_vy_s", "rise_wz_s",
                "cutoff", "note"]
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=cols)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(row)
        print(f"[calib] 已写入 {path}")


# --------------------------------------------------------------------------- #
class ChassisIO:
    """真车 I/O：发布速度指令、订阅底盘/裁判/功率。"""

    def __init__(self, node, args, msg_classes, logger):
        self.node = node
        self.args = args
        self.log = logger
        self.VelocityCommand, self.ChassisData, self.RefereeData = msg_classes
        self.pub = node.create_publisher(self.VelocityCommand, args.vel_topic, 10)
        self.chassis = None
        self.chassis_seen = 0
        self.foreign = 0
        self.last_pub = (0.0, 0.0, 0.0)
        node.create_subscription(self.ChassisData, args.chassis_topic, self._on_chassis, 10)
        node.create_subscription(self.VelocityCommand, args.vel_topic, self._on_velocity, 10)
        self.power_topic = args.power_topic
        self.power_field = args.power_field
        self.power_values = []
        self.referee_seen = 0
        self.detect_power_source()

    # ---- 自动功率探测：msg 里有没有功率字段 ----
    def detect_power_source(self):
        if self.args.no_power:
            return
        if self.power_topic and self.power_field:
            self._subscribe_power(self.power_topic, self.power_field)
            return
        for topic, cls in ((self.args.chassis_topic, self.ChassisData),
                           (self.args.referee_topic, self.RefereeData)):
            try:
                fields = cls.get_fields_and_field_types()
            except Exception:
                continue
            hit = next((f for f in fields if POWER_FIELD_RE.search(f)), None)
            if hit:
                self.log.info("自动功率源: %s 的字段 %s", topic, hit)
                self._subscribe_power(topic, hit)
                return
        self.log.info("%s / %s 里都没有功率字段 → 只能由操作员手输读数",
                      self.args.chassis_topic, self.args.referee_topic)

    def _subscribe_power(self, topic, field):
        if topic == self.args.chassis_topic:
            # 功率字段就在底盘反馈里：复用已有订阅，不再重复订阅同一话题
            self.power_field = field
            return
        if topic == self.args.referee_topic:
            cls = self.RefereeData
            self.node.create_subscription(cls, topic, lambda m, f=field: self._on_power(m, f), 10)
            self.power_field = field
            return
        self.log.warning("--power-topic %s 既不是 %s 也不是 %s，本工具不认识其类型，改为操作员手输",
                         topic, self.args.chassis_topic, self.args.referee_topic)

    def _on_power(self, msg, field):
        try:
            self.power_values.append(abs(float(getattr(msg, field))))
        except (AttributeError, TypeError, ValueError):
            pass

    def _on_chassis(self, msg):
        self.chassis = msg
        self.chassis_seen += 1
        if self.power_field:
            self._on_power(msg, self.power_field)

    def _on_velocity(self, msg):
        # 自己发的指令会回环；数值对不上说明还有别的节点在发（planner？）
        if (abs(msg.vx - self.last_pub[0]) > 1e-3
                or abs(msg.vy - self.last_pub[1]) > 1e-3
                or abs(msg.wz - self.last_pub[2]) > 1e-3):
            self.foreign += 1

    def check_no_planner(self):
        """标定期间若发现别人也在发速度指令 -> 大声告警（并且这是危险状态）。"""
        if self.foreign > 0:
            self.log.error(
                "%s 上收到 %d 条**不是本工具发出**的指令：planner 很可能也在跑！"
                "请先停掉 sentry_planner_node（两边抢同一个话题会打架）",
                self.args.vel_topic, self.foreign,
            )
            return False
        return True

    def publish(self, vx, vy, wz):
        """发速度指令；上下文已关则返回 False 而不是抛异常。"""
        try:
            import rclpy

            if not rclpy.ok():
                return False
            msg = self.VelocityCommand()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.vx = float(vx)
            msg.vy = float(vy)
            msg.wz = float(wz)
            self.pub.publish(msg)
            self.last_pub = (float(vx), float(vy), float(wz))
            return True
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            self.log.warning("发速度指令失败（%s）", exc)
            return False

    def zero(self, seconds=0.0):
        """急停：尽力发零速；任何关停竞态都不许冒出来（返回是否至少发出一次）。"""
        import rclpy

        sent = False
        try:
            if not rclpy.ok():
                return False
            self.publish(0.0, 0.0, 0.0)
            sent = True
            end = time.monotonic() + seconds
            while time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.02)
                self.publish(0.0, 0.0, 0.0)
        except KeyboardInterrupt:
            raise
        except Exception as exc:      # RCLError / InvalidHandle：上下文已关
            self.log.warning("发零速失败（%s）", exc)
            return sent
        return sent

    def _spin(self, timeout=0.0):
        """安全版 spin_once：中断/上下文关停时返回 False，不抛异常。"""
        try:
            import rclpy

            if not rclpy.ok():
                return False
            rclpy.spin_once(self.node, timeout_sec=timeout)
            return True
        except KeyboardInterrupt:
            raise
        except Exception:
            return False

    # ------------------------------------------------------------------ #
    def run_stage(self, stage, args):
        """跑一级：ramp 升速 -> dwell 保持（同时采样）-> 返回测量字典。"""
        period = 1.0 / args.rate
        t0 = time.monotonic()
        total = args.ramp + args.dwell
        target = (stage["vx"], stage["vy"], stage["wz"])
        samples = {"vx": [], "vy": [], "wz": []}
        rise = {}
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= total:
                break
            alpha = 1.0 if args.ramp <= 0.0 else min(1.0, elapsed / args.ramp)
            self.publish(target[0] * alpha, target[1] * alpha, target[2] * alpha)
            self._spin(0.0)
            if self.chassis is not None:
                for key, cmd in (("vx", target[0]), ("vy", target[1]), ("wz", target[2])):
                    if cmd > 0.0:
                        samples[key].append(abs(float(getattr(self.chassis, key))))
                if elapsed >= args.ramp:
                    for key, cmd in (("vx", target[0]), ("vy", target[1]), ("wz", target[2])):
                        if (cmd > 0.0 and key not in rise
                                and abs(float(getattr(self.chassis, key))) >= 0.9 * cmd):
                            rise[key] = round(elapsed - args.ramp, 3)
            sleep = period - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

        achieved = {}
        for key in ("vx", "vy", "wz"):
            vals = samples[key]
            achieved[f"{key}_mean"] = sum(vals) / len(vals) if vals else float("nan")
            achieved[f"{key}_max"] = max(vals) if vals else float("nan")
        return achieved, rise


# --------------------------------------------------------------------------- #
def build_parser():
    p = argparse.ArgumentParser(
        prog="calib_speed.py",
        description="真车分级速度/功率标定，产出 src/sentry_planner/config/speed_limits.yaml",
        epilog=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--stages", default=DEFAULT_STAGES,
                   help="分级表，';' 分隔：vx=800 / vy=600 / wz=120 / combo=<vx>,<wz>；"
                        f"默认 {DEFAULT_STAGES}")
    p.add_argument("--dwell", type=float, default=4.0, help="每级保持时长 s（默认 4）")
    p.add_argument("--ramp", type=float, default=0.5, help="每级升速用时 s（默认 0.5；0 = 直接跳变）")
    p.add_argument("--rest", type=float, default=1.0, help="级间零速间隔 s（默认 1.0，便于读功率/散热）")
    p.add_argument("--rate", type=float, default=50.0, help="指令发布频率 Hz（默认 50）")
    p.add_argument("--power-limit", type=float, default=100.0,
                   help="功率上限 W（默认 100，官方功率锁定门限）")
    p.add_argument("--no-power", action="store_true",
                   help="完全不测功率（不提示、不自动读），限值保持默认")
    p.add_argument("--power-topic", default=None, help="功率字段所在话题（默认自动探测 chassis/referee）")
    p.add_argument("--power-field", default=None, help="功率字段名（与 --power-topic 一起用）")
    p.add_argument("--csv", default=None, help="把每级测量写成 CSV")
    p.add_argument("--out", default=None,
                   help=f"输出参数文件；默认运行时时解析 sentry_planner 的 share 目录"
                        f"（形如 {DEFAULT_OUT}），并在源码树里同步留一份")
    p.add_argument("--dry-run", action="store_true",
                   help="headless 且不落盘：不初始化 ROS、不发指令、不碰底盘，只打印分级表与将要写入的 yaml")
    p.add_argument("--script", default=None,
                   help="headless：操作员答案改从文件读（每级一行：瓦数 / '-' / 'cutoff'），"
                        "结果仍写入 --out；真车请改用交互模式（不加此参数）")
    p.add_argument("--vel-topic", default="/sentry/velocity_command", help="速度指令话题")
    p.add_argument("--chassis-topic", default="/chassis_data", help="底盘反馈话题")
    p.add_argument("--referee-topic", default="/referee_data", help="裁判系统话题")
    p.add_argument("--chassis-wait", type=float, default=2.0,
                   help="启动时等底盘反馈的时间 s（默认 2.0，没有也能跑）")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    args.mirror_source = args.out is None
    if args.out is None:
        args.out = default_out_path()
    if args.dwell <= 0.0 or args.rate <= 0.0 or args.ramp < 0.0 or args.rest < 0.0:
        print("--dwell/--rate 必须 > 0，--ramp/--rest 必须 >= 0", file=sys.stderr)
        return 3
    try:
        stages = parse_stages(args.stages)
    except ValueError as exc:
        print(f"--stages 解析失败: {exc}", file=sys.stderr)
        return 3
    if bool(args.power_topic) != bool(args.power_field):
        print("--power-topic 与 --power-field 必须同时给", file=sys.stderr)
        return 3

    logger = PrintLogger()
    node = None
    io = None
    headless = bool(args.dry_run or args.script)
    if not headless:
        try:
            import rclpy
            from rm_msgs.msg import ChassisData, RefereeData, VelocityCommand
        except ImportError as exc:
            print(f"ROS 环境不可用: {exc}\n"
                  "请先 source /opt/ros/kilted/setup.bash && source install/setup.bash",
                  file=sys.stderr)
            return 3
    try:
        if not headless:
            import rclpy
            from rm_msgs.msg import ChassisData, RefereeData, VelocityCommand

            rclpy.init()
            node = rclpy.create_node("calib_speed")
            logger = NodeLogger(node.get_logger())
            # 接管 SIGINT/SIGTERM：rclpy.init 装的处理会先关掉上下文，导致「尽力发零速」失败。
            # 这里让信号转成 KeyboardInterrupt，于是收尾还能真的把零速发出去。
            def _interrupt(signum, frame):
                raise KeyboardInterrupt

            for _sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    signal.signal(_sig, _interrupt)
                except (ValueError, OSError):
                    pass
            logger.warning("标定直接驱动底盘：请确认 sentry_planner_node 没有在跑（两边都发 %s）",
                           args.vel_topic)
            io = ChassisIO(node, args, (VelocityCommand, ChassisData, RefereeData), logger)
            io.zero(0.0)
            deadline = time.monotonic() + args.chassis_wait
            while time.monotonic() < deadline and io.chassis is None:
                rclpy.spin_once(node, timeout_sec=0.05)
            if io.chassis is None:
                logger.warning("%s 没有反馈（假底盘/无线？）：实测列会是 nan",
                               args.chassis_topic)
        else:
            print("[calib] 无底盘模式：不初始化 ROS、不发任何指令、不碰底盘"
                  f"（{'--dry-run：结果只打印不落盘' if args.dry_run else '--script：结果会写入 --out'}）")

        operator = OperatorInput(args.script, logger)
        if args.script:
            print(f"[calib] 操作员答案来自 {args.script}")
        run = CalibRun(args, stages, operator, logger)
        print_stage_table(args, stages)

        try:
            for stage in stages:
                if io is not None and not io.check_no_planner():
                    print_backend_reason(io, args)
                    run.cutoff = True
                    run.cutoff_stage = len(run.rows) + 1
                    break
                if io is not None:
                    achieved, rise = io.run_stage(stage, args)
                    io.zero(args.rest)
                    power_values = list(io.power_values)
                    io.power_values.clear()
                    auto = None
                    if io.power_field and power_values:
                        auto = (sum(power_values) / len(power_values), max(power_values))
                else:
                    # 干跑：没有底盘，实测列 nan；升速时间不可测
                    achieved = {k: float("nan") for k in
                                ("vx_mean", "vy_mean", "wz_mean", "vx_max", "vy_max", "wz_max")}
                    rise = {}
                    auto = None
                cmd = (stage["vx"], stage["vy"], stage["wz"])
                power_mean, power_max, cutoff = run.ask_power(stage, auto)
                run.record(stage, cmd, achieved, (power_mean, power_max), rise, cutoff)
                if cutoff:
                    run.cutoff = True
                    run.cutoff_stage = len(run.rows)
                    if io is not None:
                        io.zero(0.2)
                        logger.warning("已发零速，整局中止（20 s 功率锁定代价高，不再试探）")
                    break
        except KeyboardInterrupt:
            zero_ok = False
            if io is not None:
                try:
                    zero_ok = bool(io.zero(0.2))
                except KeyboardInterrupt:
                    zero_ok = False
                except Exception as exc:
                    logger.warning("发零速失败（%s）", exc)
            print("[calib] 已中止：已发零速（随后写出的限值只用了已完成的分级）" if zero_ok
                  else "[calib] 已中止：未能发送零速（rclpy 已关停/无底盘），限值只用了已完成的分级")
            run.interrupted = True
            run.cutoff_stage = len(run.rows) + 1

        limits, why = run.derive_limits()
        if args.csv:
            run.write_csv(args.csv)
        written = run.write_yaml(limits, why)
        if written:
            try:
                verify_limits_file(written)
            except Exception as exc:
                print(f"[calib] 自检失败: {exc}", file=sys.stderr)
                return 1
        if run.interrupted:
            print("[calib] 运行被中断（Ctrl-C/SIGTERM）：限值只用了已完成的分级，退出码 130")
            return 130
        if run.cutoff:
            print("[calib] 限功率触发过：以上限值只用了已确认安全的分级，退出码 2")
            return 2
        return 0
    except ImportError as exc:
        print(f"ROS 环境不可用: {exc}\n"
              "请先 source /opt/ros/kilted/setup.bash && source install/setup.bash",
              file=sys.stderr)
        return 3
    finally:
        if node is not None:
            node.destroy_node()
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except ImportError:
            pass


def verify_limits_file(path):
    """回读自己写的参数文件：必须是合法 yaml，且 6 个键都在、都是数字。"""
    import yaml

    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    params = (data or {}).get("/sentry_planner_node", {}).get("ros__parameters", {})
    missing = [k for k in DEFAULT_LIMITS if k not in params]
    if missing:
        raise ValueError(f"{path} 缺少参数 {missing}")
    for k in DEFAULT_LIMITS:
        float(params[k])
    print(f"[calib] 自检: {path} 解析通过，6 个参数齐全 -> "
          + ", ".join(f"{k}={float(params[k]):g}" for k in DEFAULT_LIMITS))
    return params


def print_backend_reason(io, args):
    print(f"[calib] 检测到 {args.vel_topic} 上还有别的发布者（{io.foreign} 条异己指令）："
          "先停掉 planner 再标定，本次不跑任何分级")


def print_stage_table(args, stages):
    print("[calib] 分级表（ramp %.2f s + dwell %.2f s，级间零速 %.2f s，功率上限 %.0f W%s）:"
          % (args.ramp, args.dwell, args.rest, args.power_limit, "，不测功率" if args.no_power else ""))
    for i, s in enumerate(stages, 1):
        parts = []
        if s["vx"]:
            parts.append(f"vx={s['vx']:>7.0f} mm/s")
        if s["vy"]:
            parts.append(f"vy={s['vy']:>7.0f} mm/s")
        if s["wz"]:
            parts.append(f"wz={s['wz']:>7.0f} deg/s")
        print(f"  {i:2d}. {s['label']:<20} " + "  ".join(parts))


if __name__ == "__main__":
    sys.exit(main())
