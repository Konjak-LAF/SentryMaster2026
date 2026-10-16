#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图(map) <-> 里程计(odom) 的 2D 刚体对齐标定（Umeyama/SVD，**不含缩放**）。

现场流程（每个地标一对点）：
  1. 把车中心开到地标上（例如场地上的柱子/角点），按 **Enter** → 记录 tf 的 odom->vehicle 位姿；
  2. 在 RViz 里用「Publish Point」点**同一个地标**（Fixed Frame 必须是 map，点云见 tools/view_map701.sh）
     → 自动与该 odom 位姿配对；每配一对都会打印当前估计与 RMS。

结果写成 `src/sentry_planner/config/map_alignment.yaml`：
  p_odom = R(dyaw_deg) * p_map + [dx, dy]     （逆：p_map = R(-dyaw_deg) * (p_odom - [dx, dy])）
`n < 3` 或 `rms > 0.30 m` 时**拒绝写入**并打印原因（参数 --min-n / --max-rms）。

离线重算 / 自检（都不需要 ROS）：
  python3 tools/calib_align.py --from-csv pairs.csv     # 列：map_x,map_y,odom_x,odom_y[,name]
  python3 tools/calib_align.py --self-test              # 用已知 (dx,dy,dyaw) 造点，断言恢复精度
"""

from __future__ import annotations

import argparse
import csv
import shutil
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import marking_core as core  # noqa: E402

HELP = """\
命令（stdin 逐行；`#` 开头为注释）：

  <Enter>（空行）            记录当前 tf odom->vehicle 位姿，等 RViz 点击同一个地标
                            （先点地标再按 Enter 也行：点击在 --click-hold 秒内有效）
  pair <map_x> <map_y> <odom_x> <odom_y> [name]
                            手工直接加一对地标（map 在前、odom 在后，与 --from-csv 列序一致）
  name <name>               给下一对地标起名（默认 L1、L2…）
  show                      打印已采地标、当前估计与残差
  del <name|序号>           删掉一对（序号从 1 开始）
  undo                      撤销上一条改动
  wait <sec>                等若干秒（等 tf / 等 RViz 点击 / 等 DDS 发现）
  save                      解算并写 map_alignment.yaml（n<3 或 rms>阈值时拒绝写入）
                            每标一对都会打印逐地标残差，残差 > --landmark-warn 单独提示
  help / quit               打印本帮助 / 退出
"""

HEADER_TEMPLATE = (
    "# 地图(map.pcd / RViz 点云，frame=map) 与 里程计(odom) 的 2D 刚体对齐参数。\n"
    "# 由 tools/calib_align.py 现场采集 %d 组同名地标点估计（Umeyama/SVD，只含旋转+平移，无缩放）。\n"
    "# 方向：p_odom = R(dyaw_deg) * p_map + [dx, dy]（角度逆时针为正，单位 deg/m）。\n"
    "#   逆变换：p_map = R(-dyaw_deg) * (p_odom - [dx, dy])。\n"
    "# rms_m = 地标残差 RMS（m）；n = 地标组数；landmarks 为原始配对及各自残差（m）。\n"
)


class CalibSession:
    """地标配对采集 + 解算 + 落盘。"""

    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.out = Path(args.out)
        self.lock = threading.RLock()
        self.pairs: List[Dict[str, Any]] = []
        self.pending_odom: Optional[Tuple[float, float]] = None
        self.pending_click: Optional[Tuple[float, float, float]] = None
        self._click_warned_at = 0.0
        self._blank_warned = False
        self.next_name: Optional[str] = None
        self.auto_index = 0
        self.errors = 0
        self.last_rc = 0
        self._undo: List[tuple] = []
        self.tf_provider = None
        self.clicks = None
        self._ros = None

    # -- ROS ----------------------------------------------------------------

    def _setup_ros(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from tf2_ros import Buffer, TransformListener

        args = self.args
        rclpy.init(args=[])
        node = rclpy.create_node("calib_align")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, name="calib_align_spin", daemon=True)
        spin_thread.start()
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        self.tf_provider = core.TfPoseProvider(buffer, args.odom_frame, args.robot_frame, args.tf_timeout)
        self.clicks = core.ClickedPointListener(
            args.map_frame, core.make_tf_xy_lookup(buffer, args.map_frame), on_point=self._on_click
        )
        self.clicks.attach(node, args.clicked_topic)
        self._ros = (rclpy, executor, node, listener, spin_thread)
        print("[源] ROS：tf %s->%s、点击 %s（frame=%s）"
              % (args.odom_frame, args.robot_frame, args.clicked_topic, args.map_frame))

    def _shutdown(self) -> None:
        if self._ros is None:
            return
        rclpy, executor, node, listener, spin_thread = self._ros
        self._ros = None
        if self.clicks is not None:
            self.clicks.detach()
            self.clicks = None
        self.tf_provider = None
        try:
            executor.shutdown()
            spin_thread.join(timeout=3.0)
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.try_shutdown()

    # -- 采集 ----------------------------------------------------------------

    def _auto_name(self) -> str:
        while True:
            self.auto_index += 1
            name = "L%d" % self.auto_index
            if all(p["name"] != name for p in self.pairs):
                return name

    def _add_pair(self, odom: Tuple[float, float], map_pt: Tuple[float, float], name: Optional[str] = None,
                  via: str = "") -> Dict[str, Any]:
        with self.lock:
            names = {p["name"] for p in self.pairs}
            if name is None:
                name = self.next_name or self._auto_name()
            if name in names:
                raise core.CommandError("地标名重复：%s" % name)
            self._undo.append((list(self.pairs), self.pending_odom, self.next_name))
            self.next_name = None
            pair = {"name": str(name),
                    "map": (float(map_pt[0]), float(map_pt[1])),
                    "odom": (float(odom[0]), float(odom[1]))}
            self.pairs.append(pair)
            print("[配对] %s map=(%.3f, %.3f) odom=(%.3f, %.3f)%s —— 共 %d 组"
                  % (pair["name"], pair["map"][0], pair["map"][1], pair["odom"][0], pair["odom"][1],
                     ("（%s）" % via) if via else "", len(self.pairs)))
            self._print_fit(show_residuals=True)
            return pair

    def _on_click(self, x: float, y: float) -> None:
        """RViz 点击回调（ROS spin 线程里执行）。"""
        with self.lock:
            if self.pending_odom is None:
                self.pending_click = (x, y, time.time())
                now = time.time()
                if now - self._click_warned_at > 5.0:  # 点击是 5 Hz 连续发的，别刷屏
                    self._click_warned_at = now
                    print("[点击] 已记住 map=(%.3f, %.3f)：把车开到该地标上按 Enter 记录 odom 位姿，"
                          "或先按 Enter 再点地标（%gs 内有效）" % (x, y, self.args.click_hold))
                return
            odom, self.pending_odom = self.pending_odom, None
            try:
                self._add_pair(odom, (x, y), via="先按 Enter 后点地标")
            except core.CommandError as exc:
                self.pending_odom = odom
                print("ERR: %s" % exc)

    def _record_odom(self) -> None:
        x, y, yaw = self.tf_provider.get_pose()
        with self.lock:
            self.pending_odom = (x, y)
        print("[odom] x=%.3f y=%.3f yaw=%.1f deg —— 现在去 RViz 点同一个地标（Fixed Frame=map）"
              % (x, y, yaw))
        self._consume_clicked()

    def _consume_clicked(self) -> None:
        """Enter 之前如果已经点过地标（且在 --click-hold 有效期内），直接配对。"""
        with self.lock:
            if self.pending_click is None or self.pending_odom is None:
                return
            cx, cy, stamp = self.pending_click
            if time.time() - stamp > self.args.click_hold:
                self.pending_click = None
                print("[点击] 之前那次点击已超过 %g s，丢弃（请重新点地标）" % self.args.click_hold)
                return
            self.pending_click = None
            odom, self.pending_odom = self.pending_odom, None
        try:
            self._add_pair(odom, (cx, cy), via="先点地标后按 Enter")
        except core.CommandError as exc:
            with self.lock:
                self.pending_odom = odom
            print("ERR: %s" % exc)

    # -- 解算与打印 ----------------------------------------------------------

    def _fit(self):
        if len(self.pairs) < 2:
            return None
        src = [p["map"] for p in self.pairs]
        dst = [p["odom"] for p in self.pairs]
        return core.rigid_align_2d(src, dst)

    def _print_fit(self, show_residuals: bool = False) -> None:
        try:
            fit = self._fit()
        except core.CommandError as exc:
            print("[估计] 解算失败：%s" % exc)
            return
        if fit is None:
            print("[估计] 至少 2 组同名点才能解算（当前 %d 组）" % len(self.pairs))
            return
        print("[估计] n=%d dx=%.4f m dy=%.4f m dyaw=%.4f deg rms=%.4f m"
              % (fit.n, fit.dx, fit.dy, fit.dyaw_deg, fit.rms))
        if show_residuals:
            self._print_residual_flags(fit)

    def _print_residual_flags(self, fit) -> None:
        """逐地标打印残差；残差 > --landmark-warn 的单独提示（大概率点错了位置）。"""
        warn = self.args.landmark_warn
        over = []
        for pair, residual in zip(self.pairs, fit.residuals):
            residual = float(residual)
            flagged = residual > warn
            if flagged:
                over.append((pair["name"], residual))
            print("      残差 %-10s %7.3f m%s" % (pair["name"], residual, "   <== 偏大" if flagged else ""))
        if over:
            print("[提示] %d 个地标残差 > %.2f m：%s —— 大概率没点/没走到同一个物理点，"
                  "建议 `del <名字>` 后重标" % (len(over), warn,
                                               "、".join("%s(%.3f m)" % (n, r) for n, r in over)))
            newest_name, newest_residual = over[-1]
            if newest_name == self.pairs[-1]["name"]:
                print("[提示] 刚标的 %s 残差 %.3f m 偏大 —— 优先重标它" % (newest_name, newest_residual))

    def _print_residuals(self, fit) -> None:
        warn = self.args.landmark_warn
        print("[残差] %-8s %10s %10s %10s %10s %12s %s"
              % ("name", "map_x", "map_y", "odom_x", "odom_y", "residual_m", "标记"))
        over = []
        for pair, residual in zip(self.pairs, fit.residuals):
            residual = float(residual)
            flagged = residual > warn
            if flagged:
                over.append((pair["name"], residual))
            print("       %-8s %10.3f %10.3f %10.3f %10.3f %12.4f %s"
                  % (pair["name"], pair["map"][0], pair["map"][1],
                     pair["odom"][0], pair["odom"][1], residual, "偏大!" if flagged else ""))
        print("[解算] p_odom = R(%.4f deg) * p_map + [%.4f, %.4f]   （n=%d，rms=%.4f m）"
              % (fit.dyaw_deg, fit.dx, fit.dy, fit.n, fit.rms))
        if over:
            print("[提示] %d 个地标残差 > %.2f m：%s —— 建议重标这几个点再看 rms"
                  % (len(over), warn, "、".join("%s(%.3f m)" % (n, r) for n, r in over)))

    def _finish(self) -> int:
        if len(self.pairs) < 2:
            print("[拒绝写入] 只有 %d 组地标：至少要 2 组同名点才能解算，至少要 %d 组才允许写入"
                  % (len(self.pairs), self.args.min_n))
            return 1
        try:
            fit = self._fit()
        except core.CommandError as exc:
            print("[拒绝写入] 解算失败：%s" % exc)
            return 1
        self._print_residuals(fit)
        reason = core.refusal_reason(fit.n, fit.rms, self.args.min_n, self.args.max_rms)
        if reason:
            print("[拒绝写入] %s" % reason)
            return 1
        text = render_map_alignment(fit, self.pairs)
        if self.args.dry_run:
            print("[dry-run] 不写 %s；将写入：" % self.out)
            print(text, end="")
            return 0
        bak = None
        if self.out.exists():
            bak = self.out.with_name(self.out.name + ".bak")
            shutil.copy2(self.out, bak)
        self.out.parent.mkdir(parents=True, exist_ok=True)
        self.out.write_bytes(text.encode("utf-8"))
        print("[写入] %s（%d 组地标，rms=%.4f m，备份 %s）"
              % (self.out, fit.n, fit.rms, bak if bak else "无（此前无文件）"))
        example = fit.transform(*self.pairs[0]["map"])
        print("[用法] 目标点 p_map=(%.4f, %.4f) → 下发给规划器 p_odom=(%.4f, %.4f)，"
              "朝向 odom_yaw = map_yaw + %.4f deg"
              % (self.pairs[0]["map"][0], self.pairs[0]["map"][1],
                 example[0], example[1], fit.dyaw_deg))
        return 0

    # -- 命令 ----------------------------------------------------------------

    def _cmd_help(self, tokens: List[str]) -> None:
        print(HELP, end="")

    def _cmd_show(self, tokens: List[str]) -> None:
        print("[待配对 odom 位姿] %s"
              % ("(%.3f, %.3f)" % self.pending_odom if self.pending_odom else "无"))
        if self.pending_click is not None:
            age = time.time() - self.pending_click[2]
            print("[已记住的点击] map=(%.3f, %.3f)（%.1f s 前，%.0f s 内有效）"
                  % (self.pending_click[0], self.pending_click[1], age, self.args.click_hold))
        print("[下一对名字] %s" % (self.next_name or "（自动 L<n>）"))
        if not self.pairs:
            print("[地标] 空")
        else:
            print("[地标] %-8s %10s %10s %10s %10s" % ("name", "map_x", "map_y", "odom_x", "odom_y"))
            for pair in self.pairs:
                print("       %-8s %10.3f %10.3f %10.3f %10.3f"
                      % (pair["name"], pair["map"][0], pair["map"][1], pair["odom"][0], pair["odom"][1]))
        self._print_fit(show_residuals=True)

    def _cmd_pair(self, tokens: List[str]) -> None:
        if len(tokens) not in (5, 6):
            raise core.CommandError("用法：pair <map_x> <map_y> <odom_x> <odom_y> [name]")
        try:
            mx, my, ox, oy = (float(v) for v in tokens[1:5])
        except ValueError as exc:
            raise core.CommandError("pair 需要 4 个数值：%r" % (tokens[1:5],)) from exc
        name = tokens[5] if len(tokens) == 6 else None
        self._add_pair((ox, oy), (mx, my), name)

    def _cmd_name(self, tokens: List[str]) -> None:
        if len(tokens) != 2:
            raise core.CommandError("用法：name <name>")
        if any(p["name"] == tokens[1] for p in self.pairs):
            raise core.CommandError("地标名重复：%s" % tokens[1])
        self.next_name = tokens[1]
        print("[name] 下一对地标命名为 %s" % tokens[1])

    def _cmd_del(self, tokens: List[str]) -> None:
        if len(tokens) != 2:
            raise core.CommandError("用法：del <name|序号>")
        target = tokens[1]
        index = None
        if target.isdigit():
            index = int(target) - 1
            if not (0 <= index < len(self.pairs)):
                raise core.CommandError("序号超出范围：%s（共 %d 组）" % (target, len(self.pairs)))
        else:
            for i, pair in enumerate(self.pairs):
                if pair["name"] == target:
                    index = i
                    break
            if index is None:
                raise core.CommandError("没有叫 %s 的地标" % target)
        with self.lock:
            self._undo.append((list(self.pairs), self.pending_odom, self.next_name))
            removed = self.pairs.pop(index)
        print("[del] 删掉 %s map=(%.3f, %.3f) odom=(%.3f, %.3f)"
              % (removed["name"], removed["map"][0], removed["map"][1],
                 removed["odom"][0], removed["odom"][1]))
        self._print_fit()

    def _cmd_undo(self, tokens: List[str]) -> None:
        if not self._undo:
            raise core.CommandError("没有可撤销的改动")
        self.pairs, self.pending_odom, self.next_name = self._undo.pop()
        print("[undo] 已回退到上一条改动之前（当前 %d 组）" % len(self.pairs))

    def _cmd_wait(self, tokens: List[str]) -> None:
        if len(tokens) != 2:
            raise core.CommandError("用法：wait <sec>")
        try:
            seconds = float(tokens[1])
        except ValueError as exc:
            raise core.CommandError("wait 需要秒数：%r" % tokens[1]) from exc
        time.sleep(seconds)
        print("[wait] %.3f s" % seconds)

    def _cmd_save(self, tokens: List[str]) -> None:
        self.last_rc = self._finish()

    # -- 主循环 --------------------------------------------------------------

    def _dispatch(self, text: str) -> Optional[str]:
        tokens = text.split()
        cmd = tokens[0]
        if cmd == "quit":
            print("[quit] 退出")
            return "quit"
        handlers = {
            "help": self._cmd_help,
            "show": self._cmd_show,
            "pair": self._cmd_pair,
            "name": self._cmd_name,
            "del": self._cmd_del,
            "undo": self._cmd_undo,
            "wait": self._cmd_wait,
            "save": self._cmd_save,
        }
        handler = handlers.get(cmd)
        if handler is None:
            print("ERR: 未知命令 %r（输入 help 看命令表）" % cmd)
            self.errors += 1
            return None
        self.last_rc = 0
        try:
            handler(tokens)
        except core.CommandError as exc:
            print("ERR: %s" % exc)
            self.errors += 1
        else:
            if cmd == "save" and self.last_rc:
                self.errors += 1
        return None

    @staticmethod
    def _strip_comment(line: str) -> str:
        out = []
        for i, ch in enumerate(line):
            if ch == "#" and (i == 0 or line[i - 1].isspace()):
                break
            out.append(ch)
        return "".join(out).strip()

    def run_interactive(self) -> int:
        if not self.args.no_ros:
            self._setup_ros()
        else:
            print("[源] --no-ros：只能手工配对（pair / --from-csv），不初始化 ROS")
        is_tty = sys.stdin.isatty()
        print("[交互] Enter=记录 odom 位姿；RViz 点击自动配对；help 看命令；save 写文件；quit 退出")
        self._cmd_show([])
        try:
            while True:
                if is_tty:
                    print("calib> ", end="", flush=True)
                line = sys.stdin.readline()
                if not line:
                    if is_tty:
                        print()
                    break
                text = self._strip_comment(line)
                if not text:
                    if self.args.no_ros:
                        if not self._blank_warned:
                            self._blank_warned = True
                            print("[提示] --no-ros 下没有 tf，空行无法记录位姿；请用 pair 命令"
                                  "（后面再出现空行/注释行会静默跳过）")
                        continue
                    try:
                        self._record_odom()
                    except core.CommandError as exc:
                        print("ERR: %s" % exc)
                        self.errors += 1
                    continue
                if self._dispatch(text) == "quit":
                    break
        finally:
            self._shutdown()
        return 1 if self.errors else 0

    def run_csv(self) -> int:
        pairs = read_csv_pairs(self.args.from_csv)
        for pair in pairs:
            print("[CSV] %s map=(%.3f, %.3f) odom=(%.3f, %.3f)"
                  % (pair["name"], pair["map"][0], pair["map"][1], pair["odom"][0], pair["odom"][1]))
        self.pairs = pairs
        return self._finish()

    def run_self_test(self) -> int:
        return self_test()


def render_map_alignment(fit: core.RigidFit, pairs: Sequence[Dict[str, Any]]) -> str:
    """把解算结果渲染成 map_alignment.yaml 文本（键名/结构与约定一致）。"""
    out = [HEADER_TEMPLATE % fit.n]
    out.append("dx: %.4f\n" % fit.dx)
    out.append("dy: %.4f\n" % fit.dy)
    out.append("dyaw_deg: %.4f\n" % fit.dyaw_deg)
    out.append("rms_m: %.4f\n" % fit.rms)
    out.append("n: %d\n" % fit.n)
    if pairs:
        out.append("landmarks:\n")
        for pair, residual in zip(pairs, fit.residuals):
            out.append("  - {name: %s, map: [%.4f, %.4f], odom: [%.4f, %.4f], residual_m: %.4f}\n"
                       % (core.yaml_quote(pair["name"]), pair["map"][0], pair["map"][1],
                          pair["odom"][0], pair["odom"][1], float(residual)))
    else:
        out.append("landmarks: []\n")
    return "".join(out)


def read_csv_pairs(path) -> List[Dict[str, Any]]:
    p = Path(path)
    if not p.is_file():
        raise core.CommandError("CSV 不存在：%s" % p)
    pairs: List[Dict[str, Any]] = []
    with p.open("r", encoding="utf-8", newline="") as handle:
        for lineno, row in enumerate(csv.reader(handle), 1):
            cells = [c.strip() for c in row]
            if not cells or all(not c or c.startswith("#") for c in cells):
                continue
            try:
                values = [float(c) for c in cells[:4]]
            except (ValueError, IndexError):
                if lineno == 1 and any(cells):
                    continue  # 表头
                raise core.CommandError("%s:%d 前 4 列必须是 map_x,map_y,odom_x,odom_y 数值：%r"
                                        % (p, lineno, row))
            if len(values) < 4:
                raise core.CommandError("%s:%d 只有 %d 列，需要 4 列（第 5 列可选 name）"
                                        % (p, lineno, len(values)))
            name = cells[4] if len(cells) > 4 and cells[4] else "L%d" % (len(pairs) + 1)
            pairs.append({"name": name, "map": (values[0], values[1]), "odom": (values[2], values[3])})
    if not pairs:
        raise core.CommandError("%s 里没有任何地标行" % p)
    return pairs


def self_test() -> int:
    failures: List[str] = []

    def check(ok: bool, label: str, detail: str = "") -> None:
        print("[%s] %s%s" % ("PASS" if ok else "FAIL", label, ("  " + detail) if detail else ""))
        if not ok:
            failures.append(label)

    map_pts = [(2.0, 3.0), (4.5, 8.25), (7.0, 2.5), (9.75, 11.0), (-1.5, 6.5)]
    cases = [(0.0, 0.0, 0.0), (1.25, -0.75, 30.0), (-2.5, 3.5, -47.5), (0.5, 0.5, 170.0), (3.0, -1.0, -179.9)]
    last_fit = None
    for dx, dy, dyaw in cases:
        rotation = core.rotation_2d(dyaw)
        offset = np.array([dx, dy])
        odom_pts = [rotation @ np.array(pt) + offset for pt in map_pts]
        fit = core.rigid_align_2d(map_pts, odom_pts)
        err = max(abs(fit.dx - dx), abs(fit.dy - dy), abs(core.wrap_deg_diff(fit.dyaw_deg, dyaw)))
        check(err < 1e-9, "已知变换恢复 dx=%.2f dy=%.2f dyaw=%.2f" % (dx, dy, dyaw),
              "最大误差=%.3e m/deg，rms=%.3e m" % (err, fit.rms))
        check(fit.n == len(map_pts), "n 记录正确（%d）" % fit.n)
        check(fit.rms < 1e-9, "无噪声数据残差 RMS≈0", "rms=%.3e m" % fit.rms)
        last_fit = fit

    # 正/逆变换互逆
    fit = last_fit
    px, py = map_pts[2]
    bx, by = fit.inverse_transform(*fit.transform(px, py))
    check(abs(bx - px) < 1e-9 and abs(by - py) < 1e-9,
          "transform / inverse_transform 互逆", "回代误差=%.3e m" % max(abs(bx - px), abs(by - py)))

    # 反向对齐（odom -> map）应得到相反旋转
    rot30 = core.rotation_2d(30.0)
    off30 = np.array([1.25, -0.75])
    odom30 = [rot30 @ np.array(pt) + off30 for pt in map_pts]
    inv = core.rigid_align_2d(odom30, map_pts)
    back = [inv.transform(*q) for q in odom30]
    check(abs(core.wrap_deg_diff(inv.dyaw_deg, -30.0)) < 1e-9
          and max(abs(b[0] - m[0]) for b, m in zip(back, map_pts)) < 1e-9,
          "反向对齐得到相反旋转并把点还原", "dyaw=%.3e deg 期望 -30" % core.wrap_deg_diff(inv.dyaw_deg, -30.0))

    # 阈值/拒绝逻辑
    check(core.refusal_reason(5, 0.10) is None, "n=5 rms=0.10 → 允许写入")
    check(core.refusal_reason(2, 0.01) is not None, "n=2 → 拒绝写入（少于 3 组）",
          (core.refusal_reason(2, 0.01) or ""))
    check(core.refusal_reason(3, 0.40) is not None, "rms=0.40 > 0.30 → 拒绝写入",
          (core.refusal_reason(3, 0.40) or ""))

    # 噪声数据：残差 RMS 应该变大且被阈值拦住
    rotation = core.rotation_2d(15.0)
    clean = [rotation @ np.array(pt) + np.array([0.3, 0.4]) for pt in map_pts[:3]]
    noisy = [list(q) for q in clean]
    noisy[2][0] += 1.0
    fit_noisy = core.rigid_align_2d(map_pts[:3], noisy)
    check(fit_noisy.rms > 0.30, "一组点偏 1 m → rms 超阈值", "rms=%.4f m" % fit_noisy.rms)
    check(core.refusal_reason(fit_noisy.n, fit_noisy.rms) is not None, "该组数据被拒绝写入")

    # 渲染结果能被 yaml.safe_load 读回，且字段齐全
    import yaml

    pairs = [{"name": "L%d" % (i + 1), "map": map_pts[i],
              "odom": tuple(core.rotation_2d(30.0) @ np.array(map_pts[i]) + np.array([1.25, -0.75]))}
             for i in range(len(map_pts))]
    fit = core.rigid_align_2d([p["map"] for p in pairs], [p["odom"] for p in pairs])
    text = render_map_alignment(fit, pairs)
    data = yaml.safe_load(text)
    ok = (isinstance(data, dict) and abs(data["dx"] - 1.25) < 1e-9 and abs(data["dy"] + 0.75) < 1e-9
          and abs(data["dyaw_deg"] - 30.0) < 1e-9 and data["n"] == 5
          and set(data) == {"dx", "dy", "dyaw_deg", "rms_m", "n", "landmarks"})
    check(ok, "渲染的 yaml 可被 yaml.safe_load 解析且字段齐全", "keys=%s" % sorted(yaml.safe_load(text)))
    ok = all(set(item) == {"name", "map", "odom", "residual_m"} and len(item["map"]) == 2
             for item in data["landmarks"])
    check(ok, "landmarks 每项含 name/map/odom/residual_m")

    # 退化输入
    try:
        core.rigid_align_2d([(1.0, 1.0)] * 3, [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)])
        check(False, "map 侧点重合时应报错")
    except core.CommandError as exc:
        check(True, "map 侧点重合时报错", str(exc))
    try:
        core.rigid_align_2d(map_pts[:2], [(1.0, 1.0)])
        check(False, "两侧数量不一致时应报错")
    except core.CommandError as exc:
        check(True, "两侧数量不一致时报错", str(exc))

    if failures:
        print("[结果] self-test 失败 %d 项：%s" % (len(failures), "; ".join(failures)))
        return 1
    print("[结果] self-test 全部通过")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="calib_align.py",
        description="地图(map) <-> 里程计(odom) 的 2D 刚体对齐标定（Umeyama/SVD，无缩放）",
        epilog=HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--out", default=str(core.DEFAULT_ALIGNMENT_FILE),
                        help="输出 yaml 路径，默认 %(default)s")
    parser.add_argument("--from-csv", metavar="FILE",
                        help="离线重算：列 map_x,map_y,odom_x,odom_y[,name]（可选表头）")
    parser.add_argument("--self-test", action="store_true", help="无 ROS 自检：合成数据断言恢复精度")
    parser.add_argument("--dry-run", action="store_true", help="解算+打印，但不写 yaml")
    parser.add_argument("--no-ros", action="store_true",
                        help="不初始化 ROS：交互里只能用 pair 命令手工配对")
    parser.add_argument("--min-n", type=int, default=3, help="允许写入的最少地标组数，默认 %(default)s")
    parser.add_argument("--max-rms", type=float, default=0.30, help="允许写入的最大残差 RMS（m），默认 %(default)s")
    parser.add_argument("--map-frame", default="map", help="地图/点云 frame，默认 %(default)s")
    parser.add_argument("--odom-frame", default="odom", help="里程计 frame，默认 %(default)s")
    parser.add_argument("--robot-frame", default="vehicle", help="车体 frame，默认 %(default)s")
    parser.add_argument("--clicked-topic", default="/clicked_point",
                        help="RViz「Publish Point」话题，默认 %(default)s")
    parser.add_argument("--tf-timeout", type=float, default=3.0, help="等 tf 的超时（s），默认 %(default)s")
    parser.add_argument("--landmark-warn", type=float, default=0.15,
                        help="单个地标残差超过它就单独提示（m），默认 %(default)s")
    parser.add_argument("--click-hold", type=float, default=20.0,
                        help="先点地标后按 Enter 时，点击保持有效的秒数，默认 %(default)s")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    session = CalibSession(args)
    try:
        if args.self_test:
            return session.run_self_test()
        if args.from_csv:
            return session.run_csv()
        return session.run_interactive()
    except core.CommandError as exc:
        print("ERR: %s" % exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n[中断] 退出")
        return 130
    finally:
        session._shutdown()


if __name__ == "__main__":
    sys.exit(main())
