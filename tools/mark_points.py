#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""现场打点：把 5 个任务点 + 路径图（节点/边/禁小陀螺区）标出来并写回 yaml。

现场用法（两个终端）：
    终端 A: bash tools/view_map701.sh                 # RViz 里看 701 场地点云（Fixed Frame = map）
    终端 B: python3 tools/mark_points.py              # 位姿取 tf(odom->vehicle)，也可用 RViz 的 Publish Point

写文件规则（--dry-run 时只打印不改）：
  * `src/sentry_decision_sys/config/task_table.yaml`：**只重写被标记过的那几行点位行**，
    行首缩进/键名/对齐空格与行尾注释逐字保留，其余字节一律不动；写前先存 `.bak`。
  * `src/sentry_planner/config/waypoint_graph.yaml`：整体重写（nodes / edges / spin_forbidden_zones）。

命令见下方 HELP 文本；`--script FILE` 可把命令写进文件回放（现场 15 分钟标定也可以脚本化），
`--no-ros` 用 `--pose-stdin` 的坐标行做纯离线冒烟测试，完全不碰 ROS/tf。
"""

from __future__ import annotations

import argparse
import copy
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

import marking_core as core  # noqa: E402

HELP = """\
命令（stdin 逐行；`#` 开头为注释，脚本文件同样语法）：

  show                              打印当前状态（位姿源/点击/已标点位/路径图）
  next <point_name>                 选中要点名的点位（task_table.yaml 里已有的点位之一）
  set [name]                        把当前位姿（或最近一次点击）记到点位 [name]
  yaw <deg>                         设置当前点位朝向（deg；`yaw nan` 取消朝向约束）
  radius <m>                        设置当前点位的到达半径
  node <name>                       在当前位置新增路径图节点（名字唯一）
  link <a> <b>                      连一条无向边（两个节点都要已存在）
  zone <name> [x_min y_min x_max y_max]
                                    新增禁小陀螺矩形；省略 4 个坐标时用「当前位姿 ± 当前 radius」
  del <name>                        点位→清空为 .nan；节点→连它的边一起删；区域→删
  undo                              撤销上一条改动
  wait <sec>                        等若干秒（等 tf / 等 RViz 点击 / 等 DDS 发现）
  save                              写文件（点位文件只改点位那几行 + 路径图整体重写）
                                    然后打印「点位汇总」+「地图自检」（每个已标点在
                                    nav_map.bin 里的安全边距，< path_margin 会显著警告；
                                    nav_map.bin 不存在则跳过并提示先跑 tools/build_nav_map.py）
  help / quit                       打印本帮助 / 退出（不自动保存）

位姿来源 --source：
  pose    只用 tf 的 odom->vehicle：每次 `set`/`node` 取当前位姿
  click   只用 RViz「Publish Point」发出的 /clicked_point
  either  默认：有未消费的点击就用点击，否则用位姿（点击优先于位姿）
"""


class MarkSession:
    """一次打点会话：状态 + 命令派发 + 落盘。"""

    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.points_file = Path(args.points_file)
        self.graph_file = Path(args.graph_file)
        self.errors = 0
        self._undo: List[tuple] = []
        self.current: Optional[str] = None
        self.pose_provider = None
        self.tf_provider = None
        self.clicks: Optional[core.ClickedPointListener] = None
        self._ros = None
        self._nav_map = None
        self._nav_map_mod = None
        self._nav_map_checked = False
        self._nav_map_error = ""
        self._nav_map_missing = False

        values = core.load_points_values(self.points_file)
        self.point_order = list(values.keys())
        self.points: Dict[str, Dict[str, Any]] = {}
        for name, row in values.items():
            self.points[name] = {
                "x": row["x"],
                "y": row["y"],
                "yaw_deg": row["yaw_deg"],
                "radius": row["radius"],
                "touched": False,
                "src": core.SRC_FILE,
            }
        loaded = core.load_waypoint_graph(self.graph_file)
        if loaded is None:
            self.nodes: List[Dict[str, Any]] = []
            self.edges: List[List[str]] = []
            self.zones: List[Dict[str, Any]] = []
            self.graph_loaded = False
        else:
            self.nodes, self.edges, self.zones = loaded
            self.graph_loaded = True

    # -- 位姿源 --------------------------------------------------------------

    def _setup_sources(self) -> None:
        args = self.args
        if args.no_ros:
            if args.source == core.SRC_CLICK:
                raise core.CommandError("--no-ros 下没有 ROS 点击源，请用 --source pose")
            if not args.pose_stdin:
                raise core.CommandError("--no-ros 需要 --pose-stdin FILE（每行 `x y [yaw_deg]`）")
            self.pose_provider = core.ScriptedPoseProvider(args.pose_stdin, where="--pose-stdin")
            print("[源] --no-ros：位姿来自 %s（脚本位姿剩余 %s 行）"
                  % (args.pose_stdin, self.pose_provider.remaining()))
            return

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from tf2_ros import Buffer, TransformListener

        rclpy.init(args=[])
        node = rclpy.create_node("mark_points")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, name="mark_points_spin", daemon=True)
        spin_thread.start()
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        self.tf_provider = core.TfPoseProvider(buffer, args.odom_frame, args.robot_frame, args.tf_timeout)
        self.pose_provider = self.tf_provider
        if args.source in (core.SRC_CLICK, "either"):
            self.clicks = core.ClickedPointListener(
                args.odom_frame, core.make_tf_xy_lookup(buffer, args.odom_frame)
            )
            self.clicks.attach(node, args.clicked_topic)
        self._ros = (rclpy, executor, node, listener, spin_thread)
        print("[源] ROS：tf %s->%s、点击 %s、source=%s"
              % (args.odom_frame, args.robot_frame, args.clicked_topic, args.source))

    def _shutdown(self) -> None:
        if self._ros is None:
            return
        rclpy, executor, node, listener, spin_thread = self._ros
        self._ros = None
        # 先放开订阅回调的引用，再停 spin 线程，避免在对象销毁时还有回调在跑
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

    def _resolve_position(self):
        """→ (x, y, yaw_deg | None, 来源标签)；点击优先于位姿。"""
        if self.clicks is not None:
            click = self.clicks.take()
            if click is not None:
                return (click[0], click[1], None, core.SRC_CLICK)
            if self.args.source == core.SRC_CLICK:
                status = self.clicks.status()
                hint = ""
                if status["last_raw_frame"]:
                    hint = "（最后一次点击 frame=%s）" % status["last_raw_frame"]
                raise core.CommandError(
                    "还没有收到 %s 的点击%s：在 RViz 里用工具栏 Publish Point 点一下，或改用 --source pose"
                    % (self.args.clicked_topic, hint)
                )
        x, y, yaw = self.pose_provider.get_pose()
        return (x, y, yaw, core.SRC_POSE)

    # -- 状态与撤销 ----------------------------------------------------------

    def _snapshot(self) -> None:
        self._undo.append((
            copy.deepcopy(self.points),
            copy.deepcopy(self.nodes),
            copy.deepcopy(self.edges),
            copy.deepcopy(self.zones),
            self.current,
        ))

    def _table(self) -> str:
        rows = ["  %-16s %10s %10s %10s %10s  %-6s %s"
                % ("name", "x", "y", "yaw_deg", "radius", "src", "已改")]
        for name in self.point_order:
            p = self.points[name]
            rows.append("  %-16s %10s %10s %10s %10s  %-6s %s"
                        % (name, core.format_m(p["x"]), core.format_m(p["y"]),
                           core.format_m(p["yaw_deg"], 1), core.format_m(p["radius"]),
                           p["src"], "*" if p["touched"] else ""))
        return "\n".join(rows)

    def _graph_line(self) -> str:
        nodes = ", ".join("%s(%.2f,%.2f)" % (n["name"], n["x"], n["y"]) for n in self.nodes) or "—"
        edges = ", ".join("%s-%s" % (e[0], e[1]) for e in self.edges) or "—"
        zones = ", ".join("%s x[%.2f,%.2f] y[%.2f,%.2f]"
                          % (z["name"], z["x_min"], z["x_max"], z["y_min"], z["y_max"])
                          for z in self.zones) or "—"
        return "[路径图] 节点: %s\n         边: %s\n         禁陀螺区: %s" % (nodes, edges, zones)

    def _require_current(self, cmd: str) -> str:
        if self.current is None:
            raise core.CommandError("`%s` 需要先 `next <point_name>` 选中点位（可用：%s）"
                                    % (cmd, ", ".join(self.point_order)))
        return self.current

    def _load_nav_map(self):
        """惰性载入 nav_map.bin（用 slice 1 的 tools/nav_map.py），失败返回 None。"""
        if self._nav_map_checked:
            return self._nav_map
        self._nav_map_checked = True
        path = Path(self.args.nav_map)
        if not path.is_file():
            self._nav_map_missing = True
            self._nav_map_error = "未找到 %s" % path
            return None
        try:
            import nav_map as nav_map_mod

            self._nav_map_mod = nav_map_mod
            self._nav_map = nav_map_mod.load(str(path))
        except Exception as exc:  # noqa: BLE001 - 报给操作员即可
            self._nav_map_error = "载入 %s 失败：%s" % (path, exc)
            self._nav_map = None
        return self._nav_map

    def _print_map_check(self) -> None:
        """保存后自检：每个已标点在 nav_map.bin 里的安全边距（一眼看出有没有标到墙里）。"""
        nav = self._load_nav_map()
        if nav is None:
            hint = ("先跑 `python3 tools/build_nav_map.py` 生成 nav_map.bin"
                    if self._nav_map_missing else
                    "地图自检需要可用的 nav_map.bin / tools/nav_map.py")
            print("[地图自检] 跳过：%s（%s）" % (self._nav_map_error, hint))
            return
        mod = self._nav_map_mod
        margin = self.args.path_margin
        print("[地图自检] %s（%dx%d，r=%.3f m，origin=[%.2f, %.2f]，frame=%s），path_margin=%.2f m"
              % (self.args.nav_map, nav.width, nav.height, nav.resolution,
                 nav.origin[0], nav.origin[1], nav.frame, margin))
        print("  %-16s %9s %9s %-12s %-5s %7s  %s"
              % ("name", "x", "y", "cell(col,row)", "cost", "clear", "判定"))
        bad = []
        for name in self.point_order:
            point = self.points[name]
            if not (core.is_finite(point["x"]) and core.is_finite(point["y"])):
                print("  %-16s %9s %9s %-12s %-5s %7s  %s"
                      % (name, "nan", "nan", "-", "-", "-", "未打点"))
                continue
            col, row = mod.world_to_cell(nav, point["x"], point["y"])
            inside = mod.in_bounds(nav, col, row)
            occupied = bool(nav.cost[row, col] != 0) if inside else True
            dist = mod.clearance(nav, point["x"], point["y"])
            if not inside:
                verdict, flagged = "越界!（不在栅格内）", True
            elif occupied:
                verdict, flagged = "占据格!（标到墙里了）", True
            elif dist < margin:
                verdict, flagged = "边距不足!", True
            else:
                verdict, flagged = "OK", False
            if flagged:
                bad.append((name, dist, verdict))
            print("  %-16s %9.3f %9.3f %-12s %-5s %7.3f  %s"
                  % (name, point["x"], point["y"], "(%d,%d)" % (col, row),
                     "free" if not occupied else "occ", dist, verdict))
        if bad:
            print("[警告] %d 个点位的安全边距 < %.2f m（或落在占据格/界外），规划时会绕开它们：%s"
                  % (len(bad), margin,
                     "、".join("%s(%.2f m %s)" % (n, d, v) for n, d, v in bad)))
            print("       请现场挪点，或核对 path_margin / nav_map.bin 是否与场地一致")
        else:
            print("[地图自检] 全部已标点位安全边距 >= %.2f m —— OK" % margin)

    def _supply_distance(self) -> Optional[float]:
        names = ("supply", "supply_pullout")
        if any(n not in self.points for n in names):
            return None
        a, b = (self.points[n] for n in names)
        if not all(core.is_finite(v) for v in (a["x"], a["y"], b["x"], b["y"])):
            return None
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"])

    def _print_summary(self) -> None:
        print("[点位汇总]")
        print(self._table())
        print(self._graph_line())
        dist = self._supply_distance()
        if dist is None:
            print("[距离] supply <-> supply_pullout：坐标不全，无法判断是否够远")
            return
        thr = self.args.min_leave_dist
        if dist < thr:
            print("[警告] supply <-> supply_pullout 只有 %.2f m < %.2f m：规则要求返回补给区前"
                  "先离开 >= %.1f m（decision.yaml 的 leave_supply_dist 默认也是 %.1f m），"
                  "请把拉出点挪远（否则那一分钟的补给不计分）"
                  % (dist, thr, self.args.rule_leave_dist, thr))
        else:
            print("[距离] supply <-> supply_pullout = %.2f m >= %.2f m —— OK" % (dist, thr))

    # -- 命令 ----------------------------------------------------------------

    def _cmd_show(self, tokens: List[str]) -> None:
        args = self.args
        if args.no_ros:
            remaining = self.pose_provider.remaining()
            print("[源] --no-ros：source=%s，脚本位姿剩余 %s 行" % (args.source, remaining))
        else:
            state = "可用" if self.tf_provider.available() else "不可用"
            print("[源] tf %s->%s=%s，source=%s" % (args.odom_frame, args.robot_frame, state, args.source))
            if self.clicks is not None:
                st = self.clicks.status()
                pending = st["pending"]
                print("[点击] %s：共 %d 次，待用=%s"
                      % (args.clicked_topic, st["count"],
                         ("(%.3f, %.3f)" % (pending[0], pending[1])) if pending else "无"))
        print("[当前点位] %s" % (self.current or "（未选，用 next <point_name>）"))
        print(self._table())
        print(self._graph_line())
        if self.graph_loaded:
            print("         来源: 已从 %s 载入现有路径图（save 时整体重写）" % self.graph_file)
        else:
            print("         来源: %s 不存在，从空图开始（save 时创建）" % self.graph_file)

    def _cmd_next(self, tokens: List[str]) -> None:
        if len(tokens) != 2:
            raise core.CommandError("用法：next <point_name>（可用：%s）" % ", ".join(self.point_order))
        name = tokens[1]
        if name not in self.points:
            raise core.CommandError("点位名不存在：%s（可用：%s）" % (name, ", ".join(self.point_order)))
        self.current = name
        p = self.points[name]
        print("[next] 当前点位 %s（x=%s y=%s yaw=%s radius=%s）"
              % (name, core.format_m(p["x"]), core.format_m(p["y"]),
                 core.format_m(p["yaw_deg"], 1), core.format_m(p["radius"])))

    def _cmd_set(self, tokens: List[str]) -> None:
        name = tokens[1] if len(tokens) > 1 else self.current
        if name is None:
            raise core.CommandError("先用 `next <point_name>` 或 `set <point_name>` 指定点位（可用：%s）"
                                    % ", ".join(self.point_order))
        if name not in self.points:
            raise core.CommandError("点位名不存在：%s（可用：%s）" % (name, ", ".join(self.point_order)))
        x, y, yaw, src = self._resolve_position()
        self._snapshot()
        p = self.points[name]
        p["x"], p["y"], p["src"], p["touched"] = float(x), float(y), src, True
        if core.is_finite(yaw):
            p["yaw_deg"] = float(yaw)
        self.current = name
        yaw_text = "%.1f deg" % p["yaw_deg"] if core.is_finite(p["yaw_deg"]) else "保持（未约束）"
        print("[set] %s <- %s x=%.3f y=%.3f yaw=%s" % (name, src, x, y, yaw_text))

    def _cmd_yaw(self, tokens: List[str]) -> None:
        if len(tokens) != 2:
            raise core.CommandError("用法：yaw <deg>（`yaw nan` 取消朝向约束）")
        name = self._require_current("yaw")
        try:
            value = float("nan") if tokens[1].lower() in ("nan", ".nan") else float(tokens[1])
        except ValueError as exc:
            raise core.CommandError("yaw 需要数值：%r" % tokens[1]) from exc
        self._snapshot()
        self.points[name]["yaw_deg"] = value
        self.points[name]["touched"] = True
        print("[yaw] %s yaw_deg=%s" % (name, core.format_number(value)))

    def _cmd_radius(self, tokens: List[str]) -> None:
        if len(tokens) != 2:
            raise core.CommandError("用法：radius <m>")
        name = self._require_current("radius")
        try:
            value = float(tokens[1])
        except ValueError as exc:
            raise core.CommandError("radius 需要数值：%r" % tokens[1]) from exc
        if not (value > 0.0):
            raise core.CommandError("radius 必须 > 0")
        self._snapshot()
        self.points[name]["radius"] = value
        self.points[name]["touched"] = True
        print("[radius] %s radius=%.3f m" % (name, value))

    def _cmd_node(self, tokens: List[str]) -> None:
        if len(tokens) != 2:
            raise core.CommandError("用法：node <name>")
        name = tokens[1]
        x, y, _yaw, src = self._resolve_position()
        existing = next((n for n in self.nodes if n["name"] == name), None)
        if existing is not None:
            if abs(existing["x"] - x) < 1e-9 and abs(existing["y"] - y) < 1e-9:
                print("[node] %s 已存在且坐标相同 (%.3f, %.3f)，跳过" % (name, x, y))
                return
            raise core.CommandError(
                "路径图节点 %s 已存在但坐标不同：已有 (%.3f, %.3f)，当前 %s (%.3f, %.3f)；"
                "先 `del %s` 再建" % (name, existing["x"], existing["y"], src, x, y, name))
        self._snapshot()
        self.nodes.append(core.node_entry(name, x, y))
        print("[node] %s <- %s (%.3f, %.3f)" % (name, src, x, y))

    def _cmd_link(self, tokens: List[str]) -> None:
        if len(tokens) != 3:
            raise core.CommandError("用法：link <a> <b>（无向边）")
        a, b = tokens[1], tokens[2]
        names = {n["name"] for n in self.nodes}
        missing = [n for n in (a, b) if n not in names]
        if missing:
            raise core.CommandError("节点不存在：%s（先用 node <name> 建点）" % ", ".join(missing))
        if a == b:
            raise core.CommandError("不能自连：%s" % a)
        if [a, b] in self.edges or [b, a] in self.edges:
            print("[link] %s - %s 已存在，跳过" % (a, b))
            return
        self._snapshot()
        self.edges.append([a, b])
        print("[link] %s - %s" % (a, b))

    def _cmd_zone(self, tokens: List[str]) -> None:
        if len(tokens) < 2:
            raise core.CommandError("用法：zone <name> [x_min y_min x_max y_max]")
        name = tokens[1]
        rest = tokens[2:]
        origin = ""
        if rest:
            if len(rest) != 4:
                raise core.CommandError("zone 的坐标要么全省略，要么给全 4 个：x_min y_min x_max y_max")
            try:
                x_min, y_min, x_max, y_max = (float(v) for v in rest)
            except ValueError as exc:
                raise core.CommandError("zone 坐标需要数值：%r" % (rest,)) from exc
            origin = "显式坐标"
        else:
            cur = self._require_current("zone")
            radius = self.points[cur]["radius"]
            if not core.is_finite(radius):
                raise core.CommandError(
                    "当前点位 %s 的 radius 无效，无法做半宽半高；请直接给 4 个坐标" % cur)
            x, y, _yaw, src = self._resolve_position()
            x_min, y_min, x_max, y_max = x - radius, y - radius, x + radius, y + radius
            origin = "以 %s (%s) 为中心 ± %.3f m" % (src, cur, radius)
        if not (x_min < x_max and y_min < y_max):
            raise core.CommandError("矩形要求 x_min < x_max 且 y_min < y_max")
        existing = next((z for z in self.zones if z["name"] == name), None)
        if existing is not None:
            same = all(abs(existing[k] - v) < 1e-9 for k, v in
                       (("x_min", x_min), ("x_max", x_max), ("y_min", y_min), ("y_max", y_max)))
            if same:
                print("[zone] %s 已存在且范围相同，跳过" % name)
                return
            raise core.CommandError(
                "禁陀螺区 %s 已存在但范围不同：已有 x[%.3f, %.3f] y[%.3f, %.3f]，"
                "当前 x[%.3f, %.3f] y[%.3f, %.3f]；先 `del %s` 再建"
                % (name, existing["x_min"], existing["x_max"], existing["y_min"], existing["y_max"],
                   x_min, x_max, y_min, y_max, name))
        self._snapshot()
        self.zones.append(core.zone_entry(name, x_min, y_min, x_max, y_max))
        print("[zone] %s x[%.3f, %.3f] y[%.3f, %.3f]（%s）"
              % (name, x_min, x_max, y_min, y_max, origin))

    def _cmd_del(self, tokens: List[str]) -> None:
        if len(tokens) != 2:
            raise core.CommandError("用法：del <name>")
        name = tokens[1]
        if name in self.points:
            self._snapshot()
            p = self.points[name]
            p["x"] = p["y"] = p["yaw_deg"] = float("nan")
            p["touched"] = True
            p["src"] = "del"
            print("[del] 点位 %s 已清空为 .nan（radius 保留）" % name)
            return
        if any(n["name"] == name for n in self.nodes):
            self._snapshot()
            self.nodes = [n for n in self.nodes if n["name"] != name]
            removed = [e for e in self.edges if name in e]
            self.edges = [e for e in self.edges if name not in e]
            print("[del] 节点 %s 已删，同时删掉 %d 条边" % (name, len(removed)))
            return
        if any(z["name"] == name for z in self.zones):
            self._snapshot()
            self.zones = [z for z in self.zones if z["name"] != name]
            print("[del] 禁陀螺区 %s 已删" % name)
            return
        raise core.CommandError("没有叫 %s 的点位/节点/区域" % name)

    def _cmd_undo(self, tokens: List[str]) -> None:
        if not self._undo:
            raise core.CommandError("没有可撤销的改动")
        self.points, self.nodes, self.edges, self.zones, self.current = self._undo.pop()
        print("[undo] 已回退到上一条改动之前")

    def _cmd_wait(self, tokens: List[str]) -> None:
        if len(tokens) != 2:
            raise core.CommandError("用法：wait <sec>")
        try:
            seconds = float(tokens[1])
        except ValueError as exc:
            raise core.CommandError("wait 需要秒数：%r" % tokens[1]) from exc
        if seconds < 0:
            raise core.CommandError("wait 秒数不能为负")
        time.sleep(seconds)
        print("[wait] %.3f s" % seconds)

    def _cmd_help(self, tokens: List[str]) -> None:
        print(HELP, end="")

    def _cmd_save(self, tokens: List[str]) -> None:
        updates = {
            name: {"x": self.points[name]["x"], "y": self.points[name]["y"],
                   "yaw_deg": self.points[name]["yaw_deg"], "radius": self.points[name]["radius"]}
            for name in self.point_order if self.points[name]["touched"]
        }
        if not updates:
            print("[save] 没有任何点位被改动（点位文件仍会按原样重写一遍 + 备份）")
        result = core.apply_point_updates(self.points_file, updates, dry_run=self.args.dry_run)
        if self.args.dry_run:
            print("[save] 点位文件 %s：dry-run，未写（未建 .bak）" % result["path"])
        else:
            print("[save] 点位文件 %s 已写（备份 %s）" % (result["path"], result["bak"]))
        if result["changed"]:
            print("[save] 共改写 %d 行：" % len(result["changed"]))
            for item in result["changed"]:
                print("  - %s" % item["old"].rstrip("\n"))
                print("  + %s" % item["new"].rstrip("\n"))
        else:
            print("[save] 点位行内容与磁盘一致（无字节变化）")

        graph = core.write_waypoint_graph(self.graph_file, self.nodes, self.edges, self.zones,
                                          dry_run=self.args.dry_run)
        suffix = ("dry-run，未写" if self.args.dry_run
                  else ("已写，备份 %s" % graph["bak"] if graph["bak"] else "已写（此前无文件，无 .bak）"))
        print("[save] 路径图 %s：%d 节点 / %d 边 / %d 区域，%s"
              % (graph["path"], len(self.nodes), len(self.edges), len(self.zones), suffix))
        if self.args.dry_run:
            print("[save] 路径图将写成：")
            print(graph["text"], end="")
        self._print_summary()
        self._print_map_check()

    # -- 主循环 --------------------------------------------------------------

    @staticmethod
    def _strip_comment(line: str) -> str:
        out = []
        for i, ch in enumerate(line):
            if ch == "#" and (i == 0 or line[i - 1].isspace()):
                break
            out.append(ch)
        return "".join(out).strip()

    def _dispatch(self, text: str):
        tokens = text.split()
        cmd = tokens[0]
        if cmd == "quit":
            print("[quit] 退出（未保存的改动丢弃）")
            return "quit"
        handlers = {
            "show": self._cmd_show,
            "next": self._cmd_next,
            "set": self._cmd_set,
            "yaw": self._cmd_yaw,
            "radius": self._cmd_radius,
            "node": self._cmd_node,
            "link": self._cmd_link,
            "zone": self._cmd_zone,
            "del": self._cmd_del,
            "undo": self._cmd_undo,
            "wait": self._cmd_wait,
            "save": self._cmd_save,
            "help": self._cmd_help,
        }
        handler = handlers.get(cmd)
        if handler is None:
            print("ERR: 未知命令 %r（输入 help 看命令表）" % cmd)
            self.errors += 1
            return None
        try:
            handler(tokens)
        except core.CommandError as exc:
            print("ERR: %s" % exc)
            self.errors += 1
        return None

    def _run_script(self) -> int:
        if self.args.script == "-":
            lines = sys.stdin.read().splitlines()
            where = "<stdin>"
        else:
            path = Path(self.args.script)
            if not path.is_file():
                raise core.CommandError("脚本文件不存在：%s" % path)
            lines = path.read_text(encoding="utf-8").splitlines()
            where = str(path)
        print("[脚本] %s（%d 行）" % (where, len(lines)))
        for number, raw in enumerate(lines, 1):
            text = self._strip_comment(raw)
            if not text:
                continue
            print("> %s" % text)
            if self._dispatch(text) == "quit":
                print("[脚本] 第 %d 行 quit，提前结束" % number)
                break
        print("[脚本] 结束：错误 %d 条" % self.errors)
        return 1 if self.errors else 0

    def _run_interactive(self) -> int:
        is_tty = sys.stdin.isatty()
        print("[交互] 输入 help 看命令表；quit 退出（退出不自动保存）")
        while True:
            if is_tty:
                print("mark> ", end="", flush=True)
            line = sys.stdin.readline()
            if not line:
                if is_tty:
                    print()
                break
            text = self._strip_comment(line)
            if not text:
                continue
            if self._dispatch(text) == "quit":
                break
        return 1 if self.errors else 0

    def run(self) -> int:
        self._setup_sources()
        try:
            self._cmd_show([])
            if not self.args.script:
                return self._run_interactive()
            return self._run_script()
        finally:
            self._shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mark_points.py",
        description="现场打点：把 5 个任务点与路径图写回 yaml（只改点位那几行，其余字节不动）",
        epilog=HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--points-file", default=str(core.DEFAULT_POINTS_FILE),
                        help="task_table.yaml 路径，默认 %(default)s")
    parser.add_argument("--graph-file", default=str(core.DEFAULT_GRAPH_FILE),
                        help="waypoint_graph.yaml 路径，默认 %(default)s")
    parser.add_argument("--source", choices=[core.SRC_POSE, core.SRC_CLICK, "either"],
                        default="either", help="位姿来源，默认 %(default)s")
    parser.add_argument("--script", metavar="FILE",
                        help="从文件（`-` = stdin）逐行执行命令后退出，用于现场脚本化/离线冒烟")
    parser.add_argument("--dry-run", action="store_true", help="只打印将要写入的内容，绝不落盘")
    parser.add_argument("--no-ros", action="store_true",
                        help="测试模式：完全不 import/初始化 ROS，位姿来自 --pose-stdin")
    parser.add_argument("--pose-stdin", metavar="FILE",
                        help="--no-ros 的位姿来源（`-` = 真实 stdin），每行 `x y [yaw_deg]`")
    parser.add_argument("--odom-frame", default="odom", help="世界系 frame 名，默认 %(default)s")
    parser.add_argument("--robot-frame", default="vehicle", help="车体 frame 名，默认 %(default)s")
    parser.add_argument("--clicked-topic", default="/clicked_point",
                        help="RViz「Publish Point」发布的话题，默认 %(default)s")
    parser.add_argument("--tf-timeout", type=float, default=3.0, help="等 tf 的超时（s），默认 %(default)s")
    parser.add_argument("--min-leave-dist", type=float, default=6.0,
                        help="supply<->supply_pullout 的最小距离，低于它打印警告，默认 %(default)s")
    parser.add_argument("--rule-leave-dist", type=float, default=5.0,
                        help="规则要求的最小「离开」距离（m），只用于警告文案，默认 %(default)s")
    parser.add_argument("--nav-map", default=str(core.DEFAULT_NAV_MAP),
                        help="save 后自检用的导航栅格（tools/nav_map.py 格式），默认 %(default)s")
    parser.add_argument("--path-margin", type=float, default=core.DEFAULT_PATH_MARGIN,
                        help="点位安全边距阈值（m），低于它显著警告，默认 %(default)s（与 global_planning.yaml 一致）")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        session = MarkSession(args)
        return session.run()
    except core.CommandError as exc:
        print("ERR: %s" % exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n[中断] 退出（未保存的改动丢弃）")
        return 130


if __name__ == "__main__":
    sys.exit(main())
