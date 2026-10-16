#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""replay_game.py —— 整局无车回放：**真决策 + 真导航** 对 **模拟裁判/底盘/里程计**，并自检排程

干什么
------
把「整局比赛」在没有硬件的情况下跑通，并**核对任务表的排程与场上实际发生的是否一致**：

  * 模拟裁判：`/referee_data`，`game_time` 从 420 递减（官方 v1.4 §6.6：开始 420、结束 120、
    裁判终止归 0），速率为 `--game-duration` 秒对应 420 -> 420-时长；到点后发 0 = 已终止，
    决策节点随即待机。
  * 模拟底盘：把导航发到 `/sentry/velocity_command` 的**车体系**速度（mm/s、deg/s）
    按当前 yaw 欧拉积分成世界位姿（与 `tools/dry_run_nav.py --simulate` 同一套做法），
    发 `/odin1/odometry_highfreq` + tf `odom->odin1_base_link`（动态）+
    `odin1_base_link->vehicle`（静态，数值取 `sentry_launch/urdf/sentry_chassis.xacro`），
    并把「指令值」回发到 `/chassis_data`（理想底盘，无动力学）——于是
    `require_chassis_data:=true` 的看门狗、两端的 tf 查询都正常，不需要关掉任何门禁。
  * 起**真节点**做子进程：`sentry_decision_sys`（`task_table_file` 指向本工具给的任务表）
    与 `sentry_planner`，stdout/stderr 全部落盘 + 实时解析。
  * 时间线：≥2 Hz（默认 5 Hz）记录「最近一条 NavCommand（mode/task_id/target）+ 世界位姿 +
    决策的 `[fsm]`/`[status]` 事实」，退出时打印**阶段报告**与**自检结论**。

三种用法
--------
    # 1) 台架/CI 一把梭：自带任务表（5 个真实点位名 + 合成开阔场 + 压缩时间窗），
    #    不需要地图/路径图（require_map:=false，退化为直线跟踪）
    python3 tools/replay_game.py --bench

    # 2) 现场：用打点后的真任务表，按真实时长跑满（**注意：game-duration=300 就是 300 s 墙上时间**）
    python3 tools/replay_game.py --task-table src/sentry_decision_sys/config/task_table.yaml --game-duration 300

    # 3) 生成一份「真坐标量级」的示例任务表，然后用它做一次中等时长回放
    python3 tools/replay_game.py --write-example-table /tmp/replay_real.yaml
    python3 tools/replay_game.py --task-table /tmp/replay_real.yaml --game-duration 180

数据来源（报告里逐列标注）
--------------------------
  * 「log」列（进入时刻 / 到达时刻 / 停留完成值 / 是否记分 / 累计预计得分）：全部来自
    **决策节点自己的日志**（`[fsm]` / `[status]`）。注意 `[fsm]` 是 1 Hz 一次性刷出的，
    因此这些时刻有 ≤1 s 的量化误差。
  * 「meas」列（实测到位距离、实测停留时长、实测进入/到达）：来自**本工具的 5 Hz 时间线**，
    底盘是理想积分，所以这就是场上「真值」。「实测到位」= 本阶段先离开过该点 `radius`
    再回到半径内的时刻（否则开局就站在目标点上会被误判成到位）。

自检（`--expect`，默认开）
--------------------------
逐阶段比对任务表（t_start/t_end/stay_sec/score）与实际发生的事，打印 PASS/FAIL/SKIP + 汇总，
出现以下任一情况则退出码 1：
  * 必跑阶段从未进入（被跳过 / 时间窗过了）；
  * 到位时实测离目标点超过该点 `radius`（或整段都没进过半径）；
  * 停留没有完成 / 实测停留时长明显不足 `stay_sec`；
  * 得分与得分表对不上（记分的项、分数与 `score:` 不符；或决策累计分与各次加分之和对不上）；
  * 子进程日志里出现故障行（`[fault]` / `串口数据中断` / tf、odom、chassis 超时 / 卡住 / 无路可走）。
坐标是 `.nan`（未打点，诚实状态）或阶段窗口超出本次 `--game-duration` 的阶段记为 SKIP，不算失败。
全部 SKIP（例如未打点的真任务表）退出码 0。

隔离（必看）
------------
所有子进程与本节点都用 `--domain-id`（默认 `ROS_DOMAIN_ID`，没有就用 7）与
`FASTDDS_BUILTIN_TRANSPORTS=SHM`（本机 UDP 发现不可用，同机进程必须走共享内存）启动，
不会干扰别的回放/台架。子进程用独立进程组，收工只杀自己起的进程组。

日志与临时文件
--------------
子进程日志落在 `--log-dir`（默认 `mkdtemp()`），收工时打印路径，默认保留（`--rm-logs` 可删）。
"""

import argparse
import math
import os
import queue
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np
import yaml

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(TOOLS_DIR)
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)

import dry_run_nav as drn            # noqa: E402  复用几何/URDF/任务表解析（同一套约定）

DEFAULT_URDF = drn.DEFAULT_URDF
DEFAULT_DECISION_YAML = os.path.join(
    REPO_ROOT, "install/sentry_decision_sys/share/sentry_decision_sys/config/decision.yaml")
DEFAULT_PLANNER_YAML = os.path.join(
    REPO_ROOT, "install/sentry_planner/share/sentry_planner/config/global_planning.yaml")

MODE_NAMES = {0: "STOP", 1: "GOTO", 2: "HOLD"}

# 故障行（任一出现 -> 自检 FAIL）。字符串按子进程 stdout 原文匹配。
FAULT_PATTERNS = [
    ("[fault]", "子进程 [fault]"),
    ("串口数据中断", "决策: 串口(裁判/底盘)数据中断降级"),
    ("odometry timeout", "导航: 里程计超时停车"),
    ("tf odom -> vehicle unavailable", "导航: tf odom->vehicle 不可用"),
    ("tf odom->vehicle unavailable", "决策: tf odom->vehicle 不可用"),
    ("nav command timeout", "导航: 决策指令超时停车"),
    ("robot may be stuck", "导航: 疑似卡住"),
    ("no path to (", "导航: 无可行路径"),
    ("map/graph unavailable", "导航: 地图/路径图不可用"),
]


# --------------------------------------------------------------------------- #
# 任务表：台架合成表 / 示例表
# --------------------------------------------------------------------------- #
# 合成开阔场：~10x10 m（±5 m），全部点位都在场内且互不遮挡；离补给点都 ≥ 6 m，
# 于是 ① 的「先离开再进入」(leave_supply_dist=6 m) 天然满足。
BENCH_POINTS = {
    "supply":         {"x": -3.5, "y": -3.5, "radius": 0.25},
    "supply_pullout": {"x": 3.5, "y": 3.5, "radius": 0.30},
    "central_high":   {"x": -3.5, "y": 3.5, "radius": 0.15},
    "trapezoid_high": {"x": 3.5, "y": -3.5, "radius": 0.15},
    "bunker":         {"x": 4.5, "y": 4.5, "radius": 0.15},
}
BENCH_PHASES = [
    # name, t_start, t_end, point, pre_point, stay_sec, counts_as, spin_travel, spin_hold
    ("supply_m1", 0.0, 40.0, "supply", "supply_pullout", 3.0, "supply", False, False),
    ("central_high", 20.0, 70.0, "central_high", None, 10.0, "central_high", True, False),
    ("trapezoid_high", 45.0, 92.0, "trapezoid_high", None, 10.0, "trapezoid_high", True, False),
    ("bunker", 65.0, 108.0, "bunker", None, 10.0, "bunker", True, False),
    ("supply_m2", 88.0, 116.0, "supply", "supply_pullout", 3.0, "supply", False, False),
]
BENCH_SCORE = {"supply": 30, "central_high": 150, "trapezoid_high": 75, "bunker": 120,
               "spin_bonus": 30}
BENCH_TASK_IDS = {"supply": 1, "central_high": 2, "trapezoid_high": 3, "bunker": 4}

# 现场示例表：「真坐标量级」（~20x20 m 的场地，点位间距 5~15 m），阶段/停留/得分沿用
# 真任务表的数字（含 pre_wait_minute 的分钟同步），可以直接拿来做中等时长回放。
EXAMPLE_POINTS = {
    "supply":         {"x": -5.0, "y": -5.0, "radius": 0.25},
    # 拉出点必须离 supply ≥ leave_supply_dist（默认 6 m）：hypot(4, 6) = 7.21 m
    "supply_pullout": {"x": -1.0, "y": 1.0, "radius": 0.30},
    "central_high":   {"x": 4.0, "y": 5.0, "radius": 0.15},
    "trapezoid_high": {"x": -4.0, "y": 5.0, "radius": 0.15},
    "bunker":         {"x": 5.0, "y": -5.0, "radius": 0.15},
}
EXAMPLE_TABLE = {
    "points": EXAMPLE_POINTS,
    "score": {"supply": 30, "central_high": 150, "trapezoid_high": 75, "bunker": 120,
              "spin_bonus": 30},
    "task_ids": {"supply": 1, "central_high": 2, "trapezoid_high": 3, "bunker": 4},
    "phases": [
        {"name": "supply_m1", "task_id": 1, "t_start": 0.0, "t_end": 55.0,
         "pre_point": "supply_pullout", "point": "supply", "stay_sec": 5.0,
         "posture": "score", "counts_as": "supply"},
        {"name": "supply_m2", "task_id": 1, "t_start": 50.0, "t_end": 120.0,
         "pre_point": "supply_pullout", "pre_wait_minute": True, "point": "supply",
         "stay_sec": 5.0, "posture": "score", "counts_as": "supply"},
        {"name": "central_high", "task_id": 2, "t_start": 55.0, "t_end": 178.0,
         "point": "central_high", "stay_sec": 20.0, "posture": "score",
         "spin_travel": True, "counts_as": "central_high"},
        {"name": "supply_m3", "task_id": 1, "t_start": 110.0, "t_end": 180.0,
         "pre_point": "supply_pullout", "pre_wait_minute": True, "point": "supply",
         "stay_sec": 5.0, "posture": "score", "counts_as": "supply"},
        {"name": "trapezoid_high", "task_id": 3, "t_start": 120.0, "t_end": 178.0,
         "point": "trapezoid_high", "stay_sec": 20.0, "posture": "score",
         "spin_travel": True, "counts_as": "trapezoid_high"},
        {"name": "supply_m4", "task_id": 1, "t_start": 165.0, "t_end": 240.0,
         "pre_point": "supply_pullout", "pre_wait_minute": True, "point": "supply",
         "stay_sec": 5.0, "posture": "score", "counts_as": "supply"},
        {"name": "bunker", "task_id": 4, "t_start": 190.0, "t_end": 297.0,
         "point": "bunker", "stay_sec": 20.0, "posture": "score", "spin_travel": True,
         "spin_hold": True, "counts_as": "bunker"},
        {"name": "supply_m5", "task_id": 1, "t_start": 240.0, "t_end": 299.0,
         "pre_point": "supply_pullout", "point": "supply", "stay_sec": 5.0,
         "posture": "score", "counts_as": "supply"},
    ],
}


def _nan_point(px, py, radius):
    return {"x": px, "y": py, "yaw_deg": None, "radius": radius}


def build_bench_table(stay_scale=1.0):
    """台架任务表：5 个真实点位名 + 合成开阔场 + 压缩时间窗（最后 t_end=116 s）。

    `stay_scale` 只用于**反向对照**（放大停留时长，让阶段在窗口内根本停不完 -> 必须 FAIL）。
    """
    points = {name: dict(spec, yaw_deg=None) for name, spec in BENCH_POINTS.items()}
    phases = []
    for (name, t0, t1, point, pre, stay, counts, spin_travel, spin_hold) in BENCH_PHASES:
        phase = {"name": name, "t_start": t0, "t_end": t1, "point": point,
                 "stay_sec": round(stay * stay_scale, 3), "posture": "score",
                 "counts_as": counts}
        if pre:
            phase["pre_point"] = pre
        if spin_travel:
            phase["spin_travel"] = True
        if spin_hold:
            phase["spin_hold"] = True
        phases.append(phase)
    return {"points": points, "score": dict(BENCH_SCORE), "task_ids": dict(BENCH_TASK_IDS),
            "phases": phases}


def bench_span(table):
    return max(float(ph.get("t_end", 0.0)) for ph in table["phases"])


def write_table(path, table):
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("# 由 tools/replay_game.py 生成（合成任务表，仅供无车回放/自检用）\n")
        yaml.safe_dump(table, handle, allow_unicode=True, sort_keys=False, default_flow_style=False)


# --------------------------------------------------------------------------- #
# 任务表 -> 归一化阶段
# --------------------------------------------------------------------------- #
def load_phases(table):
    """任务表 -> [dict]，字段已做类型归一（与 scheduler.Phase 的字段一一对应）。"""
    ids = {str(k): int(v) for k, v in (table.get("task_ids") or {}).items()}
    phases = []
    for raw in (table.get("phases") or []):
        if not isinstance(raw, dict):
            continue
        counts = raw.get("counts_as")
        phases.append({
            "name": str(raw.get("name", "phase")),
            "t_start": drn._num(raw.get("t_start", 0.0)),
            "t_end": drn._num(raw.get("t_end", 300.0)),
            "point": str(raw.get("point", "")),
            "pre_point": (str(raw["pre_point"]) if raw.get("pre_point") not in (None, "") else None),
            "stay_sec": drn._num(raw.get("stay_sec", 0.0)),
            "counts_as": (None if counts in (None, "") else str(counts)),
            "spin_travel": bool(raw.get("spin_travel", False)),
            "spin_hold": bool(raw.get("spin_hold", False)),
            "pre_wait_minute": bool(raw.get("pre_wait_minute", False)),
            "task_id": int(raw.get("task_id", 0) or ids.get(str(counts), 0)),
        })
    return phases


# --------------------------------------------------------------------------- #
# 子进程：stdout 线程读 + 落盘 + 时间戳入队
# --------------------------------------------------------------------------- #
class Child:
    def __init__(self, name, cmd, env, log_path, echo=False):
        self.name = name
        self.cmd = list(cmd)
        self.log_path = log_path
        self.echo = echo
        self.lines = queue.Queue()
        self.tail = []
        self._log = open(log_path, "w", encoding="utf-8")
        self.proc = subprocess.Popen(
            self.cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, encoding="utf-8", errors="replace", bufsize=1,
            env=env, start_new_session=True,
        )
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        try:
            for line in self.proc.stdout:
                stamp = time.monotonic()
                text = line.rstrip("\n")
                self._log.write(line)
                self._log.flush()
                self.tail.append(text)
                if len(self.tail) > 40:
                    del self.tail[:len(self.tail) - 40]
                self.lines.put((stamp, text))
                if self.echo:
                    print(f"[{self.name}] {text}", flush=True)
        finally:
            self._log.flush()

    def drain(self):
        out = []
        while True:
            try:
                out.append(self.lines.get_nowait())
            except queue.Empty:
                return out

    def poll(self):
        return self.proc.poll()

    def stop(self, timeout=6.0):
        """只杀自己起的进程组：SIGINT -> 等 -> SIGKILL。"""
        try:
            pgid = os.getpgid(self.proc.pid)
        except OSError:
            pgid = None
        for sig in (signal.SIGINT, signal.SIGTERM):
            if self.proc.poll() is not None:
                break
            try:
                if pgid is not None:
                    os.killpg(pgid, sig)
                else:
                    self.proc.send_signal(sig)
            except OSError:
                break
            end = time.monotonic() + timeout / 2.0
            while time.monotonic() < end:
                if self.proc.poll() is not None:
                    break
                time.sleep(0.05)
        if self.proc.poll() is None:
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGKILL)
                else:
                    self.proc.kill()
            except OSError:
                pass
            try:
                self.proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
        try:
            self._log.close()
        except OSError:
            pass
        return self.proc.poll()


# --------------------------------------------------------------------------- #
# 日志行解析
# --------------------------------------------------------------------------- #
ROS_LINE_RE = re.compile(r"^\[(\w+)\] \[([\d.]+)\] \[([^\]]+)\]: (.*)$")


def split_ros_line(line):
    """`[INFO] [1700.0] [node]: msg` -> (level, node, msg)；不是 ROS 日志行 -> (None, None, line)。"""
    match = ROS_LINE_RE.match(line)
    if match is None:
        return None, None, line
    return match.group(1), match.group(3), match.group(4)


RE_ENTER = re.compile(r"^\[fsm\] enter (\S+) \((\S+)\)")
RE_SKIP = re.compile(r"^\[fsm\] skip (\S+): (.*)$")
RE_ABORT = re.compile(r"^\[fsm\] abort (\S+): (.*)$")
RE_ARRIVE = re.compile(r"^\[fsm\] (\S+): 到达(?:前置航点 )?([^，,（(]+)(.*)$")
RE_HOLD_DONE = re.compile(r"^\[fsm\] (\S+): 停留完成 ([\d.]+)s >= ([\d.]+)s")
RE_AWARD = re.compile(r"^\[fsm\] 得分 (\S+) \+([\d.]+)（累计 ([\d.]+)）")
RE_SPIN_BONUS = re.compile(r"^\[fsm\] ⑤ 小陀螺加分 \+([\d.]+)（累计 ([\d.]+)）")
RE_LEAVE_OK = re.compile(r"^\[fsm\] ①有效：最远离补给点 ([\d.]+) m")
RE_LEAVE_BAD = re.compile(r"^\[fsm\] ①无效：本分钟最远离补给点 ([\d.]+) m < ([\d.]+) m")
RE_MINUTE = re.compile(r"^\[fsm\] (\S+): 分钟窗口开启")
RE_RESET = re.compile(r"^\[fsm\] (\S+): 离开停留半径，停留计时清零")
RE_STATUS_T = re.compile(r"t=\s*([\d.]+)s")
RE_STATUS_POSE = re.compile(r"pose=\(([-\d.]+), ([-\d.]+), ([-\d.]+)deg\)")
RE_STATUS_PHASE = re.compile(r"phase=(\S+) sub=(\S+) posture=(\d+) hold=([\d.]+)s")
RE_STATUS_SCORE = re.compile(r"score=([\d.eE+-]+)")
RE_STATUS_AWARDS = re.compile(r"awards=(\{[^}]*\})")
RE_STATUS_GT = re.compile(r"game_time=(\d+)\(up=(\w+)\) frames=(\d+)")


def parse_decision_line(msg):
    """决策节点的一行日志 -> event dict（无法识别的返回 None）。"""
    match = RE_ENTER.match(msg)
    if match:
        return {"kind": "enter", "phase": match.group(1), "sub": match.group(2)}
    match = RE_SKIP.match(msg)
    if match:
        return {"kind": "skip", "phase": match.group(1), "reason": match.group(2)}
    match = RE_ABORT.match(msg)
    if match:
        return {"kind": "abort", "phase": match.group(1), "reason": match.group(2)}
    match = RE_ARRIVE.match(msg)
    if match:
        return {"kind": "arrive", "phase": match.group(1), "point": match.group(2).strip(),
                "rest": match.group(3)}
    match = RE_HOLD_DONE.match(msg)
    if match:
        return {"kind": "hold_done", "phase": match.group(1),
                "hold": float(match.group(2)), "need": float(match.group(3))}
    match = RE_AWARD.match(msg)
    if match:
        return {"kind": "award", "item": match.group(1), "points": float(match.group(2)),
                "cumulative": float(match.group(3))}
    match = RE_SPIN_BONUS.match(msg)
    if match:
        return {"kind": "spin_bonus", "item": "spin_bonus", "points": float(match.group(1)),
                "cumulative": float(match.group(2))}
    match = RE_LEAVE_OK.match(msg)
    if match:
        return {"kind": "leave_ok", "dist": float(match.group(1))}
    match = RE_LEAVE_BAD.match(msg)
    if match:
        return {"kind": "leave_bad", "dist": float(match.group(1))}
    match = RE_MINUTE.match(msg)
    if match:
        return {"kind": "minute", "phase": match.group(1)}
    match = RE_RESET.match(msg)
    if match:
        return {"kind": "hold_reset", "phase": match.group(1)}
    if msg.startswith("[status]"):
        event = {"kind": "status"}
        match = RE_STATUS_T.search(msg)
        if match:
            event["t"] = float(match.group(1))
        match = RE_STATUS_POSE.search(msg)
        if match:
            event["pose"] = (float(match.group(1)), float(match.group(2)), float(match.group(3)))
        match = RE_STATUS_PHASE.search(msg)
        if match:
            event["phase"] = match.group(1)
            event["sub"] = match.group(2)
            event["posture"] = int(match.group(3))
            event["hold"] = float(match.group(4))
        match = RE_STATUS_SCORE.search(msg)
        if match:
            event["score"] = float(match.group(1))
        match = RE_STATUS_AWARDS.search(msg)
        if match:
            event["awards"] = match.group(1)
        match = RE_STATUS_GT.search(msg)
        if match:
            event["game_time"] = int(match.group(1))
            event["count_up"] = match.group(2)
            event["frames"] = int(match.group(3))
        return event
    return None


# --------------------------------------------------------------------------- #
# 阶段记录
# --------------------------------------------------------------------------- #
class PhaseRecord:
    def __init__(self, phase):
        self.phase = phase
        self.name = phase["name"]
        self.entered = False
        self.t_enter = None
        self.t_arrive_pre = None
        self.t_arrive = None          # 到达「阶段目标点」（log）
        self.t_hold_done = None
        self.hold_log = None
        self.hold_need = float(phase["stay_sec"])
        self.skip_reason = None
        self.abort_reason = None
        self.awards = []              # [(item, points, cumulative)]
        self.leave_dist = None
        self.leave_ok = None
        self.hold_resets = 0
        # 本工具时间线实测
        self.t_enter_meas = None
        self.t_arrive_meas = None
        self.meas_left = False        # 本阶段内是否先离开过目标点（否则「在位」不是「到位」）
        self.hold_meas = 0.0
        self.min_dist = float("inf")
        self.min_dist_t = None
        self.dist_at_arrival_log = float("nan")
        self.ticks = 0

    @property
    def awarded(self):
        return bool(self.awards)

    @property
    def point(self):
        return self.phase["point"]

    def award_points(self):
        return sum(points for _, points, _ in self.awards)

    def status_text(self):
        if self.skip_reason is not None and not self.entered:
            return "SKIP"
        if self.abort_reason is not None:
            return "ABORT"
        if not self.entered:
            return "NEVER"
        if self.t_hold_done is not None or self.awarded:
            return "DONE"
        return "PARTIAL"


# --------------------------------------------------------------------------- #
# 回放主体
# --------------------------------------------------------------------------- #
class GameReplay:
    """一个 rclpy 节点：既当模拟世界（裁判/底盘/里程计/tf），又当回放控制与自检。"""

    def __init__(self, node, args, table, points, phases, msgs):
        self.node = node
        self.args = args
        self.table = table
        self.points = points
        self.phases = phases
        self.msg_referee, self.msg_chassis, self.msg_odom, self.msg_nav, self.msg_vel, \
            self.msg_posture = msgs

        self.records = {ph["name"]: PhaseRecord(ph) for ph in phases}
        self.events = []              # 解析出的日志事件（按收到顺序）
        self.ticks = []               # 5 Hz 时间线
        self.faults = []              # (t, label, line)
        self.warmup_faults = 0
        self.expand_awards = []       # 末段策略(expand_margin_N)的加分
        self.current_phase = None     # 决策日志里「最近 enter 的阶段名」
        self.final_score = 0.0
        self.decision_status = {}
        self.child_exit = {}

        xyz, rpy = drn.parse_urdf_offset(args.urdf)
        self.odin_offset = drn.make_tf(xyz, drn.rpy_to_quat(*rpy))

        start = args.start
        self.sim_pose = drn.make_tf((start[0], start[1], 0.0),
                                    drn.rpy_to_quat(0.0, 0.0, math.radians(start[2])))
        self.pose = self.sim_pose.copy()
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_stamp = 0.0
        self.posture_reported = 3
        self.last_step = None
        self.last_tick = None
        self.nav_seen = 0
        self.vel_seen = 0
        self.nav_msg = None
        self.game_state = "warmup"    # warmup -> running -> tail -> done
        self.game_start = None
        self.game_elapsed = 0.0
        self.last_put = 0.0
        self.children = []

        self._create_interfaces()

    # ------------------------------------------------------------------ ROS 接口
    def _create_interfaces(self):
        node = self.node
        self.pub_referee = node.create_publisher(self.msg_referee, self.args.referee_topic, 10)
        self.pub_chassis = node.create_publisher(self.msg_chassis, self.args.chassis_topic, 10)
        self.pub_odom = node.create_publisher(self.msg_odom, self.args.odom_topic, 10)
        node.create_subscription(self.msg_vel, self.args.vel_topic, self._on_velocity, 10)
        node.create_subscription(self.msg_nav, self.args.nav_topic, self._on_nav, 10)
        node.create_subscription(self.msg_posture, self.args.posture_topic, self._on_posture, 10)

        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
        self.tf_pub = TransformBroadcaster(node)
        self.static_tf_pub = StaticTransformBroadcaster(node)
        stat = TransformStamped()
        stat.header.stamp = node.get_clock().now().to_msg()
        stat.header.frame_id = self.args.odin_frame
        stat.child_frame_id = self.args.robot_frame
        (stat.transform.translation.x, stat.transform.translation.y,
         stat.transform.translation.z) = (float(self.odin_offset[0, 3]),
                                          float(self.odin_offset[1, 3]),
                                          float(self.odin_offset[2, 3]))
        quat = drn.matrix_to_quat(self.odin_offset[:3, :3])
        (stat.transform.rotation.x, stat.transform.rotation.y,
         stat.transform.rotation.z, stat.transform.rotation.w) = quat
        self.static_tf_pub.sendTransform(stat)

        node.create_timer(1.0 / self.args.sim_rate, self._integrate)
        node.create_timer(1.0 / self.args.referee_rate, self._publish_referee)
        node.create_timer(1.0 / self.args.tick_rate, self._tick)

    def _on_velocity(self, msg):
        self.cmd = (float(msg.vx), float(msg.vy), float(msg.wz))
        self.cmd_stamp = time.monotonic()
        self.vel_seen += 1

    def _on_nav(self, msg):
        self.nav_msg = msg
        self.nav_seen += 1

    def _on_posture(self, msg):
        self.posture_reported = int(msg.posture)   # 电控回报：这里假设指令即到位

    # ------------------------------------------------------------------ 世界模拟
    def _integrate(self):
        from geometry_msgs.msg import TransformStamped
        now = time.monotonic()
        if self.last_step is None:
            self.last_step = now
            return
        dt = min(max(0.0, now - self.last_step), 0.2)
        self.last_step = now

        vx, vy, wz = self.cmd
        if now - self.cmd_stamp > 1.0:
            vx = vy = wz = 0.0            # 导航停发指令 -> 停（与真机一致）
        yaw = drn.quat_to_yaw(drn.matrix_to_quat(self.sim_pose[:3, :3]))
        self.sim_pose[:3, 3] += dt * np.array([
            vx / 1000.0 * math.cos(yaw) - vy / 1000.0 * math.sin(yaw),
            vx / 1000.0 * math.sin(yaw) + vy / 1000.0 * math.cos(yaw),
            0.0,
        ])
        yaw = drn.wrap_pi(yaw + math.radians(wz) * dt)
        self.sim_pose[:3, :3] = drn.quat_to_matrix(drn.rpy_to_quat(0.0, 0.0, yaw))
        self.pose = self.sim_pose.copy()

        stamp = self.node.get_clock().now().to_msg()
        odom_pose = self.sim_pose @ drn.inv_tf(self.odin_offset)     # odom -> odin1_base_link
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.args.global_frame
        tf.child_frame_id = self.args.odin_frame
        (tf.transform.translation.x, tf.transform.translation.y,
         tf.transform.translation.z) = (float(odom_pose[0, 3]), float(odom_pose[1, 3]),
                                        float(odom_pose[2, 3]))
        quat = drn.matrix_to_quat(odom_pose[:3, :3])
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = quat
        self.tf_pub.sendTransform(tf)

        odom = self.msg_odom()
        odom.header.stamp = stamp
        odom.header.frame_id = self.args.global_frame
        odom.child_frame_id = self.args.odin_frame
        odom.pose.pose.position.x = float(odom_pose[0, 3])
        odom.pose.pose.position.y = float(odom_pose[1, 3])
        odom.pose.pose.position.z = float(odom_pose[2, 3])
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = quat
        odom.twist.twist.linear.x = float(vx) / 1000.0               # 车体系 m/s
        odom.twist.twist.linear.y = float(vy) / 1000.0
        odom.twist.twist.angular.z = math.radians(float(wz))
        self.pub_odom.publish(odom)

        chassis = self.msg_chassis()
        chassis.vx = float(vx)          # 理想底盘：回发指令值（无动力学）
        chassis.vy = float(vy)
        chassis.wz = float(wz)
        self.pub_chassis.publish(chassis)

    # ------------------------------------------------------------------ 裁判
    def _publish_referee(self):
        if self.game_state == "warmup":          # 未开始：game_time=0 -> 决策待机
            game_time = 0
        elif self.game_state == "running":
            game_time = int(round(max(0.0, self.args.game_time_at_start - self.game_elapsed)))
        else:                                    # tail：裁判终止 -> 归 0
            game_time = 0
        msg = self.msg_referee()
        msg.game_time = int(game_time)
        msg.sentry_posture = int(self.posture_reported)
        self.pub_referee.publish(msg)

    # ------------------------------------------------------------------ 时间线
    def _tick(self):
        now = time.monotonic()
        dt = 0.0 if self.last_tick is None else (now - self.last_tick)
        self.last_tick = now
        if self.game_state == "warmup":
            return
        nav = self.nav_msg
        row = {
            "t": self.game_elapsed,
            "wall": now - (self.game_start or now),
            "dt": dt,
            "mode": int(nav.mode) if nav is not None else -1,
            "task": int(nav.task_id) if nav is not None else -1,
            "tx": float(nav.x) if nav is not None else float("nan"),
            "ty": float(nav.y) if nav is not None else float("nan"),
            "stay": float(nav.stay_sec) if nav is not None else float("nan"),
            "spin": float(nav.spin_rate) if nav is not None else float("nan"),
            "x": float(self.pose[0, 3]),
            "y": float(self.pose[1, 3]),
            "yaw": math.degrees(drn.quat_to_yaw(drn.matrix_to_quat(self.pose[:3, :3]))),
            "phase": self.current_phase or (self.decision_status.get("phase")),
            "sub": self.decision_status.get("sub"),
            "score": self.decision_status.get("score", 0.0),
        }
        self.ticks.append(row)
        self._attribute_tick(row)

    def _attribute_tick(self, row):
        """按「决策日志里的当前阶段」把这一拍算到对应阶段的实测统计里。"""
        name = row["phase"]
        if name is None:
            return
        rec = self.records.get(name)
        if rec is None:
            return
        if rec.t_enter_meas is None:
            rec.t_enter_meas = row["t"]
        point = self.points.get(rec.point)
        if point is None or not (drn.is_finite(point["x"]) and drn.is_finite(point["y"])):
            return
        dist = math.hypot(row["x"] - point["x"], row["y"] - point["y"])
        rec.ticks += 1
        if dist < rec.min_dist:
            rec.min_dist = dist
            rec.min_dist_t = row["t"]
        radius = point["radius"] if drn.is_finite(point["radius"]) else 0.15
        if dist > radius + 1e-6:
            rec.meas_left = True
        elif rec.meas_left and rec.t_arrive_meas is None:
            # 「到位」= 本阶段先出过半径、再回到半径内（否则开局就站在目标点上会误判到位）
            rec.t_arrive_meas = row["t"]
            rec.dist_at_arrival_log = dist
        if (row["mode"] == 2 and drn.is_finite(row["tx"])
                and math.hypot(row["tx"] - point["x"], row["ty"] - point["y"]) <= radius * 1.5):
            rec.hold_meas += row["dt"]

    # ------------------------------------------------------------------ 日志事件
    def _consume_lines(self, child, lines):
        for stamp, line in lines:
            level, node_name, msg = split_ros_line(line)
            game_t = (stamp - self.game_start) if self.game_start is not None else -1.0
            for pattern, label in FAULT_PATTERNS:
                if pattern in line:
                    if self.game_state in ("warmup",):
                        self.warmup_faults += 1
                    else:
                        self.faults.append((game_t, label, node_name or child.name, line.strip()))
                    break
            if level in ("ERROR", "WARN", "WARNING") and self.args.verbose:
                print(f"[{child.name}] {line.strip()}", flush=True)
            elif level == "ERROR" and "串口数据中断" in msg:
                print(f"[{child.name}] {line.strip()}", flush=True)
            event = parse_decision_line(msg) if node_name == "sentry_decision_node" else None
            if event is None:
                continue
            event["t"] = game_t
            event["wall"] = stamp
            self.events.append(event)
            self._apply_event(child, event)

    def _apply_event(self, child, event):
        kind = event["kind"]
        if kind == "status":
            for key in ("phase", "sub", "posture", "hold", "score", "awards", "game_time",
                        "frames", "pose"):
                if key in event:
                    self.decision_status[key] = event[key]
            if "score" in event:
                self.final_score = float(event["score"])
            return
        if kind == "enter":
            name = event["phase"]
            self.current_phase = name
            rec = self.records.get(name)
            if rec is not None and not rec.entered:
                rec.entered = True
                rec.t_enter = event["t"]
            return
        if kind == "skip":
            rec = self.records.get(event["phase"])
            if rec is not None:
                rec.skip_reason = event["reason"]
            self.current_phase = event["phase"]
            return
        if kind == "abort":
            rec = self.records.get(event["phase"])
            if rec is not None:
                rec.abort_reason = event["reason"]
            return
        if kind == "arrive":
            rec = self.records.get(event["phase"])
            if rec is not None:
                if event["point"] == rec.phase["pre_point"]:
                    rec.t_arrive_pre = event["t"]
                elif event["point"] == rec.point:
                    if rec.t_arrive is None:
                        rec.t_arrive = event["t"]
            return
        if kind == "hold_done":
            rec = self.records.get(event["phase"])
            if rec is not None:
                rec.t_hold_done = event["t"]
                rec.hold_log = event["hold"]
                rec.hold_need = event["need"]
            return
        if kind == "hold_reset":
            rec = self.records.get(event["phase"])
            if rec is not None:
                rec.hold_resets += 1
            return
        if kind == "leave_ok":
            rec = self.records.get(self.current_phase)
            if rec is not None:
                rec.leave_ok = True
                rec.leave_dist = event["dist"]
            return
        if kind == "leave_bad":
            rec = self.records.get(self.current_phase)
            if rec is not None:
                rec.leave_ok = False
                rec.leave_dist = event["dist"]
            return
        if kind in ("award", "spin_bonus"):
            rec = self.records.get(self.current_phase)
            entry = (event["item"], event["points"], event["cumulative"])
            if rec is not None:
                rec.awards.append(entry)
            else:
                self.expand_awards.append((self.current_phase, entry))
            return

    # ------------------------------------------------------------------ 子进程
    def _env(self):
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = str(self.args.domain_id)
        env["FASTDDS_BUILTIN_TRANSPORTS"] = "SHM"
        env.setdefault("RCUTILS_LOGGING_BUFFERED_STREAM", "0")
        return env

    def _start_children(self, table_path):
        nomap = os.path.join(self.args.log_dir, "no_map_placeholder.bin")
        planner_params = {"require_map": "false", "require_chassis_data": "true",
                          "nav_map_file": nomap, "waypoint_graph_file": nomap}
        if self.args.require_map:
            planner_params = {"require_map": "true", "require_chassis_data": "true"}
        decision_params = {"task_table_file": table_path}
        planner_extra = self._extra_tokens(self.args.planner_arg, planner_params)
        decision_extra = self._extra_tokens(self.args.decision_arg, decision_params)

        decision_cmd = ["ros2", "run", "sentry_decision_sys", "sentry_decision_node", "--ros-args"]
        if os.path.isfile(self.args.decision_yaml):
            decision_cmd += ["--params-file", self.args.decision_yaml]
        decision_cmd += self._param_tokens(decision_params) + decision_extra

        planner_cmd = ["ros2", "run", "sentry_planner", "sentry_planner_node", "--ros-args"]
        if os.path.isfile(self.args.planner_yaml):
            planner_cmd += ["--params-file", self.args.planner_yaml]
        planner_cmd += self._param_tokens(planner_params) + planner_extra

        env = self._env()
        print(f"[replay] 起决策节点: {' '.join(shlex.quote(t) for t in decision_cmd)}", flush=True)
        print(f"[replay] 起导航节点: {' '.join(shlex.quote(t) for t in planner_cmd)}", flush=True)
        self.children = [
            Child("decision", decision_cmd, env,
                  os.path.join(self.args.log_dir, "decision_node.log"), self.args.verbose),
            Child("planner", planner_cmd, env,
                  os.path.join(self.args.log_dir, "planner_node.log"), self.args.verbose),
        ]

    @staticmethod
    def _param_tokens(params):
        tokens = []
        for key, value in params.items():
            tokens += ["-p", f"{key}:={value}"]
        return tokens

    @staticmethod
    def _extra_tokens(extra, params):
        """把用户 --planner-arg/--decision-arg 里的 `-p k:=v` 从默认覆盖里摘掉（用户优先）。"""
        tokens = []
        for chunk in (extra or []):
            tokens += shlex.split(chunk)
        overridden = set()
        for i, token in enumerate(tokens):
            if token in ("-p", "--param", "--parameter") and i + 1 < len(tokens) \
                    and ":=" in tokens[i + 1]:
                overridden.add(tokens[i + 1].split(":=")[0])
        for key in overridden:
            params.pop(key, None)
        return tokens

    def _spin(self, seconds, drain=True):
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            self._spin_once(0.02)
            if drain:
                self._drain()
            for child in self.children:
                code = child.poll()
                if code is not None and child.name not in self.child_exit:
                    self.child_exit[child.name] = code
                    print(f"[replay] !! {child.name} 进程提前退出，退出码 {code}", flush=True)
                    print(f"[replay]    最后几行：\n" + "\n".join(child.tail[-8:]), flush=True)
                    return

    def _spin_once(self, timeout=0.02):
        import rclpy
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout)
        except KeyboardInterrupt:
            raise
        except Exception as exc:                       # 关停竞态
            print(f"[replay] spin 中断（{exc}）", flush=True)

    def _drain(self):
        for child in self.children:
            lines = child.drain()
            if lines:
                lines.sort(key=lambda item: item[0])
                self._consume_lines(child, lines)

    # ------------------------------------------------------------------ 主流程
    def run(self, table_path):
        interrupted = False
        try:
            print("[replay] 发布模拟世界（裁判/底盘/里程计/tf）…", flush=True)
            self._spin(1.0)
            self._start_children(table_path)
            ready_deadline = time.monotonic() + self.args.ready_timeout
            while time.monotonic() < ready_deadline:
                self._spin(0.1)
                if self.child_exit:
                    return 2
                if self.vel_seen > 0 and self.nav_seen > 0:
                    break
            else:
                print("[replay] !! 等不到「决策的 NavCommand + 导航的 VelocityCommand」："
                      "DDS 发现失败或节点起不来（本机必须 FASTDDS_BUILTIN_TRANSPORTS=SHM）。",
                      flush=True)
                self._dump_tails()
                return 2
            print(f"[replay] 两个真节点就绪（NavCommand {self.nav_seen} 帧 / "
                  f"VelocityCommand {self.vel_seen} 帧），预热 {self.args.warmup_sec:.1f}s",
                  flush=True)
            self._spin(self.args.warmup_sec)
            if self.child_exit:
                return 2
            self.game_start = time.monotonic()
            self.last_tick = None
            self.game_state = "running"
            print(f"[replay] === 比赛开始 t=0  game_time={int(self.args.game_time_at_start)} "
                  f"时长 {self.args.game_duration:.0f}s ===", flush=True)
            deadline = self.game_start + self.args.game_duration
            next_beat = self.game_start + self.args.heartbeat_sec
            while time.monotonic() < deadline:
                self._spin(0.05)
                if self.child_exit:
                    return 2
                now = time.monotonic()
                self.game_elapsed = now - self.game_start
                if now >= next_beat:
                    next_beat = now + self.args.heartbeat_sec
                    self._heartbeat()
            self.game_elapsed = self.args.game_duration
            self.game_state = "tail"
            print(f"[replay] === 时长到 game_time -> 0（裁判终止），尾部 {self.args.tail_sec:.1f}s "
                  f"观察降级 ===", flush=True)
            self._spin(self.args.tail_sec)
            self.game_state = "done"
        except KeyboardInterrupt:
            interrupted = True
            print("[replay] 收到中断（Ctrl-C）：收尾", flush=True)
        finally:
            self._drain()
            for child in self.children:
                code = child.stop()
                self.child_exit[child.name] = code
            self._drain()
        return 130 if interrupted else 0

    def _heartbeat(self):
        row = {
            "mode": int(self.nav_msg.mode) if self.nav_msg is not None else -1,
            "tx": float(self.nav_msg.x) if self.nav_msg is not None else float("nan"),
            "ty": float(self.nav_msg.y) if self.nav_msg is not None else float("nan"),
        }
        mode = MODE_NAMES.get(row["mode"], "-")
        gt = int(max(0.0, self.args.game_time_at_start - self.game_elapsed))
        print(
            f"[replay] t={self.game_elapsed:6.1f}s gt={gt:3d} "
            f"pose=({self.pose[0, 3]:6.2f},{self.pose[1, 3]:6.2f},"
            f"{math.degrees(drn.quat_to_yaw(drn.matrix_to_quat(self.pose[:3, :3]))):6.0f}deg) "
            f"cmd={mode} tgt=({row['tx']:6.2f},{row['ty']:6.2f}) "
            f"phase={self.decision_status.get('phase', 'n/a')} "
            f"sub={self.decision_status.get('sub', 'n/a')} "
            f"score={self.final_score:g}",
            flush=True,
        )

    def _dump_tails(self):
        for child in self.children:
            print(f"[replay] --- {child.name} 最后 12 行 ---", flush=True)
            for line in child.tail[-12:]:
                print(f"    {line}", flush=True)

    # ------------------------------------------------------------------ 报告
    def timeline_rows(self, limit):
        rows = []
        # 1) 导航指令/决策阶段变化（时间线主体）
        prev = None
        for tick in self.ticks:
            key = (tick["mode"], round(tick["tx"], 2), round(tick["ty"], 2), tick["task"],
                   round(tick["stay"], 1), round(tick["spin"], 0), tick["phase"], tick["sub"])
            if key != prev:
                prev = key
                rows.append({
                    "t": tick["t"], "kind": "NAV",
                    "text": (f"mode={MODE_NAMES.get(tick['mode'], '-'):<4} task={tick['task']} "
                             f"tgt=({tick['tx']:6.2f},{tick['ty']:6.2f}) "
                             f"stay={tick['stay']:5.1f} spin={tick['spin']:5.0f}"),
                    "pose": f"({tick['x']:6.2f},{tick['y']:6.2f},"
                            f"{tick['yaw']:6.0f}deg)",
                    "phase": f"{tick['phase'] or 'n/a'}/{tick['sub'] or 'n/a'}",
                    "score": tick["score"],
                })
        # 2) 决策日志事件
        labels = {
            "enter": lambda ev: f"[fsm] enter {ev['phase']} ({ev.get('sub', '')})",
            "arrive": lambda ev: f"[fsm] {ev['phase']}: 到达 {ev['point']}",
            "hold_done": lambda ev: f"[fsm] {ev['phase']}: 停留完成 "
                                    f"{ev['hold']:.1f}s >= {ev['need']:.1f}s",
            "award": lambda ev: f"[fsm] 得分 {ev['item']} +{ev['points']:g}"
                                f"（累计 {ev['cumulative']:g}）",
            "spin_bonus": lambda ev: f"[fsm] ⑤ 小陀螺加分 +{ev['points']:g}"
                                     f"（累计 {ev['cumulative']:g}）",
            "abort": lambda ev: f"[fsm] abort {ev['phase']}: {ev['reason']}",
            "skip": lambda ev: f"[fsm] skip {ev['phase']}: {ev['reason']}",
            "leave_bad": lambda ev: f"[fsm] ①无效：最远离补给点 {ev['dist']:.2f} m",
            "leave_ok": lambda ev: f"[fsm] ①有效：最远离补给点 {ev['dist']:.2f} m",
            "minute": lambda ev: f"[fsm] {ev['phase']}: 分钟窗口开启",
            "hold_reset": lambda ev: f"[fsm] {ev['phase']}: 离开停留半径，停留计时清零",
        }
        running = 0.0
        for ev in self.events:
            if ev["kind"] in ("award", "spin_bonus"):
                running = float(ev["cumulative"])
            label = labels.get(ev["kind"])
            if label is None:
                continue
            rows.append({"t": ev["t"], "kind": "LOG", "text": label(ev), "pose": "-", "phase": "-",
                         "score": running})
        rows.sort(key=lambda item: item["t"])
        return rows[:limit], len(rows)

    def phase_report(self, wall, table_path):
        out = []
        out.append("=" * 132)
        out.append(f"整局回放 / PHASE REPORT   模式={'bench' if self.args.bench else 'task-table'}"
                   f"   任务表={table_path}")
        out.append(f"game_duration={self.args.game_duration:.0f}s   墙上时间={wall:.1f}s   "
                   f"起点={self.args.start}   模拟={self.args.sim_rate:.0f}Hz   时间线="
                   f"{self.args.tick_rate:.0f}Hz（{len(self.ticks)} 拍）")
        out.append("数据来源：进入/到达/停留（log）、是否记分、累计预计得分 = **决策节点日志**"
                   "（[fsm]/[status]，1 Hz 刷出，时刻量化 ≤1s）；")
        out.append("          实测到达/实测停留/最小距离 = **本工具时间线**（假底盘理想积分 = 场上真值；"
                   "实测到位 = 本阶段先离开过该点半径、再回到半径内的时刻）。")
        out.append("-" * 132)
        header = (f"{'#':>2} {'阶段':<15} {'点位':<15} {'进入时刻':>8} {'到达时刻':>8} "
                  f"{'到位耗时':>8} {'停留时长(log/实测)':>19} {'是否记分':>10} "
                  f"{'累计预计得分':>12} {'结果':>7}")
        out.append(header)
        for idx, ph in enumerate(self.phases, 1):
            rec = self.records[ph["name"]]
            t_enter = f"{rec.t_enter:6.1f}s" if rec.t_enter is not None else "     -"
            t_arrive = f"{rec.t_arrive:6.1f}s" if rec.t_arrive is not None else "     -"
            if rec.t_enter is not None and rec.t_arrive is not None:
                travel = f"{rec.t_arrive - rec.t_enter:6.1f}s"
            else:
                travel = "     -"
            hold = "-"
            if rec.hold_log is not None or rec.hold_meas > 0.0:
                log_text = f"{rec.hold_log:.1f}" if rec.hold_log is not None else "-"
                hold = f"{log_text:>6}/{rec.hold_meas:5.1f}s"
            if rec.awarded:
                scored = "是 " + "+".join(f"{p:g}" for _, p, _ in rec.awards)
                cumulative = f"{rec.awards[-1][2]:g}"
            elif rec.entered and rec.leave_ok is False:
                scored = "否(①无效)"
                cumulative = "-"
            elif rec.entered:
                scored = "否"
                cumulative = "-"
            else:
                scored = "-"
                cumulative = "-"
            out.append(f"{idx:>2} {ph['name']:<15} {ph['point']:<15} {t_enter:>8} {t_arrive:>8} "
                       f"{travel:>8} {hold:>19} {scored:>10} {cumulative:>12} "
                       f"{rec.status_text():>7}")
            notes = []
            if rec.phase["pre_point"]:
                pre = "到达" if rec.t_arrive_pre is not None else "未到达"
                notes.append(f"前置航点 {rec.phase['pre_point']} {pre}"
                             + (f" @{rec.t_arrive_pre:.1f}s" if rec.t_arrive_pre is not None else ""))
            if rec.t_arrive_meas is not None:
                notes.append(f"实测到位 @{rec.t_arrive_meas:.1f}s（最小距离 "
                             f"{rec.min_dist:.3f}m）")
            elif rec.min_dist < float("inf"):
                notes.append(f"实测最小距离 {rec.min_dist:.3f}m（t={rec.min_dist_t:.1f}s）")
            if rec.hold_resets:
                notes.append(f"停留计时清零 {rec.hold_resets} 次")
            if rec.leave_dist is not None:
                notes.append(f"离补给点最远 {rec.leave_dist:.2f}m"
                             f"({'≥' if rec.leave_ok else '<'} leave_supply_dist)")
            if rec.skip_reason:
                notes.append(f"skip: {rec.skip_reason}")
            if rec.abort_reason:
                notes.append(f"abort: {rec.abort_reason}")
            if notes:
                out.append(f"      └ {' | '.join(notes)}")
        if self.expand_awards:
            total = sum(entry[1] for _, entry in self.expand_awards)
            out.append(f"  末段策略(expand_margin_N) 额外加分 {len(self.expand_awards)} 次，"
                       f"合计 +{total:g}（不在任务表阶段内）")
        out.append(f"决策节点累计预计得分(日志最后一条 [status]) = {self.final_score:g}")
        out.append("=" * 132)
        return "\n".join(out)

    # ------------------------------------------------------------------ 自检
    def self_check(self):
        rows = []
        fails = []
        passes = []
        skips = []
        awarded_table = 0.0

        for ph in self.phases:
            rec = self.records[ph["name"]]
            point = self.points.get(ph["point"])
            pre = self.points.get(ph["pre_point"]) if ph["pre_point"] else None
            reasons = []

            # 1) 是否要求在本次回放里跑：未打点 / 时间窗超出本局 -> SKIP（诚实状态）
            unmarked = []
            if point is None or not (drn.is_finite(point["x"]) and drn.is_finite(point["y"])):
                unmarked.append(ph["point"])
            if ph["pre_point"] and (pre is None or not (drn.is_finite(pre["x"])
                                                        and drn.is_finite(pre["y"]))):
                unmarked.append(ph["pre_point"])
            skip_reason = None
            if unmarked:
                skip_reason = f"点位未打点(.nan): {', '.join(unmarked)}"
            elif drn.is_finite(ph["t_end"]) and ph["t_end"] > self.args.game_duration + 1e-6:
                skip_reason = (f"阶段窗口 t_end={ph['t_end']:.0f}s 超出 --game-duration="
                               f"{self.args.game_duration:.0f}s（本局跑不到，不计入）")
            if skip_reason:
                skips.append((ph["name"], skip_reason))
                rows.append(("SKIP", ph["name"], skip_reason))
                continue

            # 2) 必跑阶段：进入
            if not rec.entered:
                if rec.skip_reason:
                    reasons.append(f"从未进入（决策跳过：{rec.skip_reason}）")
                else:
                    reasons.append("从未进入（决策日志里没有 enter）")
            else:
                # 3) 到位（实测真值）
                radius = point["radius"] if drn.is_finite(point["radius"]) else 0.15
                tol = self.args.arrive_tol
                if rec.t_arrive is None:
                    reasons.append("决策没有报「到达目标点」")
                if rec.min_dist > radius + tol:
                    reasons.append(f"整段实测最小距离 {rec.min_dist:.3f}m > radius {radius:.2f}m"
                                   f"(+{tol:.2f}m 容差)")
                elif rec.dist_at_arrival_log > radius + tol:
                    reasons.append(f"决策报到达时实测距离 {rec.dist_at_arrival_log:.3f}m "
                                   f"> radius {radius:.2f}m")
                # 4) 停留
                if rec.hold_need > 0.0:
                    if rec.t_hold_done is None:
                        if rec.abort_reason:
                            reasons.append(f"停留未完成（阶段超时放弃：{rec.abort_reason}）")
                        else:
                            reasons.append(f"停留未完成（需要 {rec.hold_need:.1f}s，"
                                           f"日志无「停留完成」）")
                    if rec.hold_meas + 0.5 < rec.hold_need:
                        reasons.append(f"实测在点位上的 HOLD 时长 {rec.hold_meas:.1f}s "
                                       f"< stay_sec {rec.hold_need:.1f}s")
                # 5) 记分 / 得分表
                if ph["counts_as"]:
                    item = ph["counts_as"]
                    want = float((self.table.get("score") or {}).get(item, 0.0))
                    if rec.awarded:
                        got_item = rec.awards[0][0]
                        got_points = rec.award_points()
                        want_total = want
                        if ph["spin_travel"] and item in ("central_high", "trapezoid_high",
                                                          "bunker"):
                            want_total += float((self.table.get("score") or {})
                                                .get("spin_bonus", 0.0))
                        if got_item != item:
                            reasons.append(f"记分的项对不上：日志记的是 {got_item}，"
                                           f"任务表写的是 {item}")
                        if abs(got_points - want_total) > 1e-6:
                            reasons.append(f"得分与得分表不一致：日志 +{got_points:g}，"
                                           f"任务表应为 +{want_total:g}")
                        awarded_table += got_points
                    elif rec.t_hold_done is not None or rec.t_arrive is not None:
                        if rec.leave_ok is False:
                            reasons.append(f"完成到位/停留但未记分：①无效"
                                           f"（最远离补给点 {rec.leave_dist:.2f}m）")
                        else:
                            reasons.append("完成到位/停留但日志里没有加分（未记分）")
            if reasons:
                fails.append((ph["name"], reasons))
                rows.append(("FAIL", ph["name"], "；".join(reasons)))
            else:
                passes.append(ph["name"])
                rows.append(("PASS", ph["name"], ""))

        # 6) 得分总和自洽（决策自己的账 vs 加分之和）
        note_sum = sum(ev["points"] for ev in self.events
                       if ev["kind"] in ("award", "spin_bonus"))
        global_fails = []
        if abs(note_sum - self.final_score) > 1e-6:
            global_fails.append(f"得分总和对不上：加分之和 {note_sum:g} != 决策累计 "
                                f"{self.final_score:g}")
        # 7) 阶段顺序
        order = [(ph["name"], self.records[ph["name"]].t_enter) for ph in self.phases
                 if self.records[ph["name"]].t_enter is not None]
        for (name_a, t_a), (name_b, t_b) in zip(order, order[1:]):
            if t_b + 1e-6 < t_a:
                global_fails.append(f"阶段顺序错乱：{name_b} ({t_b:.1f}s) 早于 {name_a} "
                                    f"({t_a:.1f}s)")
        # 8) 故障行
        if self.faults:
            summary = {}
            for _t, label, who, _line in self.faults:
                summary[label] = summary.get(label, 0) + 1
            detail = "；".join(f"{label}×{count}" for label, count in sorted(summary.items()))
            global_fails.append(f"出现故障行：{detail}")
        return rows, passes, fails, skips, global_fails, note_sum

    def print_check(self, rows, passes, fails, skips, global_fails, note_sum, exit_code):
        out = []
        out.append("=" * 132)
        out.append("排程自检 / SELF-CHECK（--expect）  逐阶段比对任务表 (t_start/t_end/stay_sec/score)")
        out.append("-" * 132)
        for status, name, detail in rows:
            line = f"  [{status}] {name}"
            if detail:
                line += f": {detail}"
            out.append(line)
        out.append("-" * 132)
        if self.faults:
            out.append("  故障行（前 8 条）：")
            for game_t, label, who, line in self.faults[:8]:
                out.append(f"    t={game_t:6.1f}s [{who}] {label} :: {line[:110]}")
            if len(self.faults) > 8:
                out.append(f"    … 共 {len(self.faults)} 条")
        if self.warmup_faults:
            out.append(f"  （启动预热期另有 {self.warmup_faults} 条故障行，未计入 FAIL）")
        for text in global_fails:
            out.append(f"  [FAIL] 全局: {text}")
        expected_total = sum(float((self.table.get("score") or {}).get(ph["counts_as"], 0.0))
                             + (float((self.table.get("score") or {}).get("spin_bonus", 0.0))
                                if ph["spin_travel"] and ph["counts_as"] in
                                ("central_high", "trapezoid_high", "bunker") else 0.0)
                             for ph in self.phases if ph["counts_as"]
                             and self.records[ph["name"]].awarded)
        out.append(f"  自检汇总: PASS {len(passes)} / FAIL {len(fails)} / SKIP {len(skips)}"
                   f"   已记分阶段合计 {note_sum:g}（任务表按其 score 应为 {expected_total:g}）")
        if not fails and not global_fails:
            if passes:
                out.append(f"  自检结论: PASS —— 所有必跑阶段都进入了 radius、停留达标、"
                           f"得分与得分表一致，且没有故障行（退出码 {exit_code}）")
            else:
                out.append(f"  自检结论: 全部 SKIP（点位未打点或窗口超出本局），链路跑通，"
                           f"无排程可判（退出码 {exit_code}）")
        else:
            out.append(f"  自检结论: FAIL —— 见上面的 [FAIL] 行（退出码 {exit_code}）")
        out.append("=" * 132)
        print("\n".join(out), flush=True)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def build_parser():
    p = argparse.ArgumentParser(
        prog="replay_game.py",
        description="整局无车回放：真决策 + 真导航 + 模拟裁判/底盘/里程计，并自检任务表排程",
        epilog=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--bench", action="store_true",
                      help="自包含台架：自动写合成任务表（5 个真实点位名 + 开阔场 + 压缩窗口），"
                           "require_map:=false，跑完自检并打印 BENCH 结论")
    mode.add_argument("--task-table", default=None,
                      help="任务表 yaml（现场用打点后的真表）；不填则用 --bench 合成表或包内默认表")
    p.add_argument("--write-example-table", default=None, metavar="PATH",
                   help="把「真坐标量级」的示例任务表写到 PATH 后退出（可直接用它做中等时长回放）")
    p.add_argument("--game-duration", type=float, default=None,
                   help="本局秒数（墙上时间）：game_time 从 420 递减到 420-该值，然后归 0。"
                        "默认：--bench 用「最后阶段 t_end + 8」；--task-table 用 300（=现场真实时长，"
                        "注意就是 300 s 墙上时间）")
    p.add_argument("--bench-stay-scale", type=float, default=1.0,
                   help="反向对照用：把 --bench 合成表里的 stay_sec 乘上该系数（默认 1.0）。"
                        "设成 3~6 可以让阶段在窗口内根本停不完 -> 应当 FAIL 且退出码非 0")
    p.add_argument("--expect", dest="expect", action="store_true", default=True,
                   help="跑完做排程自检并打印 PASS/FAIL/SKIP（默认开）")
    p.add_argument("--no-expect", dest="expect", action="store_false",
                   help="只回放 + 打印阶段报告，不判 PASS/FAIL")
    p.add_argument("--domain-id", default=None,
                   help="ROS_DOMAIN_ID（默认取环境变量，没有则 7）——隔离用，别跟别的回放/台架撞")
    p.add_argument("--start", default="0,0,0", help='假底盘起点 "x,y,yaw_deg"（默认 "0,0,0"）')
    p.add_argument("--require-map", action="store_true",
                   help="导航用真 701 地图 + 路径图（现场）；默认关：require_map:=false + "
                        "地图/路径图置为不存在，退化为直线跟踪（台架/仿真）")
    p.add_argument("--sim-rate", type=float, default=50.0, help="假底盘积分/发布频率 Hz（默认 50）")
    p.add_argument("--referee-rate", type=float, default=10.0, help="裁判帧频率 Hz（默认 10）")
    p.add_argument("--tick-rate", type=float, default=5.0, help="时间线记录频率 Hz（默认 5，≥2）")
    p.add_argument("--heartbeat-sec", type=float, default=10.0,
                   help="跑动期间打印一行心跳的间隔 s（默认 10）")
    p.add_argument("--warmup-sec", type=float, default=2.0,
                   help="两节点就绪后再预热多久才开始计时（默认 2）")
    p.add_argument("--ready-timeout", type=float, default=25.0,
                   help="等「决策 NavCommand + 导航 VelocityCommand」的时限 s（默认 25）")
    p.add_argument("--tail-sec", type=float, default=3.0,
                   help="比赛时长到点后、裁判发 0 的观察时长 s（默认 3）")
    p.add_argument("--arrive-tol", type=float, default=0.10,
                   help="到位自检容差 m（默认 0.10，加在点位 radius 上）")
    p.add_argument("--timeline-rows", type=int, default=200,
                   help="最多打印多少行时间线（默认 200）")
    p.add_argument("--planner-arg", action="append", default=None, metavar="'ARGS'",
                   help="追加给导航节点的 ros 参数，可重复，如 --planner-arg '-p require_map:=true'")
    p.add_argument("--decision-arg", action="append", default=None, metavar="'ARGS'",
                   help="追加给决策节点的 ros 参数，可重复，如 --decision-arg '-p serial_fault_timeout:=1.0'")
    p.add_argument("--decision-yaml", default=DEFAULT_DECISION_YAML,
                   help=f"决策节点参数文件（默认 {DEFAULT_DECISION_YAML}）")
    p.add_argument("--planner-yaml", default=DEFAULT_PLANNER_YAML,
                   help=f"导航节点参数文件（默认 {DEFAULT_PLANNER_YAML}）")
    p.add_argument("--log-dir", default=None,
                   help="子进程日志/合成任务表目录（默认 mkdtemp）")
    p.add_argument("--rm-logs", action="store_true", help="收工删掉 --log-dir")
    p.add_argument("--verbose", action="store_true", help="把子进程的 WARN/ERROR 行实时打出来")
    # 话题（与仓库约定一致）
    p.add_argument("--referee-topic", default="/referee_data")
    p.add_argument("--chassis-topic", default="/chassis_data")
    p.add_argument("--odom-topic", default="/odin1/odometry_highfreq")
    p.add_argument("--nav-topic", default="/sentry/nav_command")
    p.add_argument("--vel-topic", default="/sentry/velocity_command")
    p.add_argument("--posture-topic", default="/sentry/posture_command")
    p.add_argument("--global-frame", default="odom")
    p.add_argument("--robot-frame", default="vehicle")
    p.add_argument("--odin-frame", default="odin1_base_link")
    p.add_argument("--urdf", default=DEFAULT_URDF)
    p.add_argument("--game-time-at-start", type=float, default=420.0,
                   help="比赛开始时裁判 game_time（官方 v1.4 §6.6 = 420）")
    return p


def main(argv=None):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass
    args = build_parser().parse_args(argv)

    if args.write_example_table:
        write_table(args.write_example_table, EXAMPLE_TABLE)
        print(f"[replay] 已写出示例任务表（真坐标量级）：{args.write_example_table}")
        print(f"[replay] 用法：python3 tools/replay_game.py --task-table "
              f"{args.write_example_table} --game-duration 180")
        return 0

    if args.domain_id is None:
        args.domain_id = os.environ.get("ROS_DOMAIN_ID") or "7"
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "SHM"   # 本机 UDP 发现不可用，必须走共享内存

    if args.sim_rate <= 0.0 or args.tick_rate < 2.0 or args.referee_rate <= 0.0:
        print("--sim-rate/--referee-rate 必须 > 0，--tick-rate 必须 ≥ 2（时间线要求 ≥2 Hz）",
              file=sys.stderr)
        return 2
    try:
        args.start = drn.parse_start(args.start)
    except Exception as exc:
        print(f"--start 解析失败：{exc}", file=sys.stderr)
        return 2

    log_dir = args.log_dir or tempfile.mkdtemp(prefix="replay_game_")
    os.makedirs(log_dir, exist_ok=True)
    args.log_dir = log_dir

    # 任务表来源
    if args.bench:
        table = build_bench_table(args.bench_stay_scale)
        table_path = os.path.join(log_dir, "bench_task_table.yaml")
        write_table(table_path, table)
        if args.game_duration is None:
            args.game_duration = bench_span(table) + 8.0
    elif args.task_table:
        table_path = os.path.abspath(args.task_table)
        if not os.path.isfile(table_path):
            print(f"任务表不存在: {table_path}", file=sys.stderr)
            return 2
        if args.game_duration is None:
            args.game_duration = 300.0
    else:
        table_path = os.path.join(
            REPO_ROOT, "install/sentry_decision_sys/share/sentry_decision_sys/config/"
                       "task_table.yaml")
        if not os.path.isfile(table_path):
            table_path = drn.DEFAULT_POINTS
        if args.game_duration is None:
            args.game_duration = 300.0

    try:
        table, points = drn.load_task_table(table_path)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"任务表解析失败: {exc}", file=sys.stderr)
        return 2
    phases = load_phases(table)
    if not phases:
        print(f"任务表 {table_path} 里没有 phases:", file=sys.stderr)
        return 2

    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rm_msgs.msg import ChassisData, NavCommand, PostureCommand, RefereeData, \
            VelocityCommand
    except ImportError as exc:
        print(f"ROS 环境不可用: {exc}\n请先 source /opt/ros/kilted/setup.bash && "
              f"source install/setup.bash", file=sys.stderr)
        return 2

    print(f"[replay] ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}  "
          f"FASTDDS_BUILTIN_TRANSPORTS=SHM  日志目录 {log_dir}", flush=True)

    rclpy.init()
    node = rclpy.create_node("game_replay")
    code = 1
    replay = None
    wall = 0.0
    try:
        replay = GameReplay(node, args, table, points, phases,
                            (RefereeData, ChassisData, Odometry, NavCommand, VelocityCommand,
                             PostureCommand))
        t0 = time.monotonic()
        code = replay.run(table_path)
        wall = time.monotonic() - t0
        interrupted = code == 130
        if code == 2:
            return code
        rows = replay.timeline_rows(args.timeline_rows)
        shown, total = rows
        print("=" * 132, flush=True)
        print(f"时间线 / TIMELINE（{shown and '前' or ''}{len(shown)} / {total} 行；"
              f"NAV=导航指令变化，LOG=决策日志事件）", flush=True)
        print(f"{'t(s)':>7} {'来源':<4} {'事件':<62} {'世界位姿(实测)':<24} "
              f"{'阶段/子状态':<22} {'累计分':>6}", flush=True)
        for row in shown:
            print(f"{row['t']:7.1f} {row['kind']:<4} {row['text']:<62} {row['pose']:<24} "
                  f"{row['phase']:<22} {row['score']:>6g}", flush=True)
        print(replay.phase_report(wall, table_path), flush=True)
        if args.expect:
            rows, passes, fails, skips, global_fails, note_sum = replay.self_check()
            exit_code = 1 if (fails or global_fails or (args.bench and skips)) else 0
            if args.bench and skips and not (fails or global_fails):
                global_fails = [f"--bench 要求全部阶段都跑完，但有 {len(skips)} 个 SKIP"]
            replay.print_check(rows, passes, fails, skips, global_fails, note_sum, exit_code)
            if args.bench:
                print(f"[replay] BENCH {'PASS' if exit_code == 0 else 'FAIL'}"
                      f"（PASS {len(passes)} / FAIL {len(fails)} / SKIP {len(skips)}，"
                      f"已记分 {note_sum:g}）", flush=True)
            code = 130 if interrupted else exit_code
        else:
            print("[replay] --no-expect：只回放，不判 PASS/FAIL（退出码 0）", flush=True)
            code = 130 if interrupted else 0
    except KeyboardInterrupt:
        print("[replay] 已中断（Ctrl-C）", flush=True)
        code = 130
    except Exception as exc:
        print(f"[replay] 异常结束：{type(exc).__name__}: {exc}", flush=True)
        import traceback
        traceback.print_exc()
        code = 1
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
        if replay is not None:
            alive = [name for name, rc in replay.child_exit.items() if rc is None]
            if alive:
                print(f"[replay] 警告：子进程未正常退出 {alive}", flush=True)
        if args.rm_logs:
            import shutil
            shutil.rmtree(log_dir, ignore_errors=True)
        else:
            print(f"[replay] 子进程日志保留在 {log_dir}", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
