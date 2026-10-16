#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""现场打点/标定工具的公共底层实现（tools/mark_points.py、tools/calib_align.py 共用）。

只放两类东西：
1. **不依赖 ROS 的纯逻辑**：task_table.yaml `points:` 块的行级改写、waypoint_graph.yaml 读写、
   2D 刚体对齐（Umeyama/SVD，无缩放）、脚本化位姿源；
2. **需要 rclpy 时才 import 的薄封装**：tf 位姿查询、/clicked_point 点击监听。

坐标约定：世界系 = `odom`，长度 m，朝向统一用 **deg**（yaml 字段名 `yaw_deg`）。

`points:` 块改写的关键约束（见各函数 docstring）：
- 只重写被点名的那些**单行 flow-map** 行；
- 行首（缩进 / 键名 / 对齐空格）与行尾（空格 + `# 注释` + 换行）逐字保留；
- 其余所有行、所有块**字节不动**；写之前先存 `.bak`。
"""

from __future__ import annotations

import json
import math
import re
import shutil
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent
DEFAULT_POINTS_FILE = REPO_ROOT / "src/sentry_decision_sys/config/task_table.yaml"
DEFAULT_GRAPH_FILE = REPO_ROOT / "src/sentry_planner/config/waypoint_graph.yaml"
DEFAULT_ALIGNMENT_FILE = REPO_ROOT / "src/sentry_planner/config/map_alignment.yaml"
DEFAULT_NAV_MAP = REPO_ROOT / "src/sentry_planner/map/701_model/nav_map.bin"
DEFAULT_PATH_MARGIN = 0.35  # m：与 global_planning.yaml 的 path_margin 默认值一致

# 位姿来源标签
SRC_POSE = "pose"
SRC_CLICK = "click"
SRC_FILE = "file"


class CommandError(RuntimeError):
    """输入/命令错误：打印给操作员，不终止会话。"""


# ---------------------------------------------------------------------------
# 数值格式化
# ---------------------------------------------------------------------------


def is_nan(value: Any) -> bool:
    if value is None:
        return True
    try:
        return math.isnan(float(value))
    except (TypeError, ValueError):
        return True


def is_finite(value: Any) -> bool:
    if value is None:
        return False
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def format_number(value: Any) -> str:
    """点位行里的数值：NaN → `.nan`；否则最多 3 位小数、至少保留 1 位小数。"""
    if is_nan(value):
        return ".nan"
    text = "%.3f" % float(value)
    if "." in text:
        text = text.rstrip("0")
        if text.endswith("."):
            text += "0"
    return text


def format_m(value: Any, nd: int = 3) -> str:
    return "nan" if not is_finite(value) else ("%.*f" % (nd, float(value)))


def wrap_deg(value: float) -> float:
    """把角度折到 (-180, 180]。"""
    return (float(value) + 180.0) % 360.0 - 180.0


def wrap_deg_diff(a: float, b: float) -> float:
    return wrap_deg(float(a) - float(b))


# ---------------------------------------------------------------------------
# task_table.yaml 的 points: 块
# ---------------------------------------------------------------------------

_POINT_LINE_RE = re.compile(
    r"^([ \t]*)([A-Za-z_][A-Za-z0-9_]*)([ \t]*:[ \t]*)(\{)([^}\n]*)(\})"
)
_TOP_LEVEL_KEY_RE = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)[ \t]*:")
_POINT_KEYS = ("x", "y", "yaw_deg", "radius")


@dataclass
class PointLine:
    """task_table.yaml `points:` 块里的一行单行 flow-map 点位。"""

    name: str
    index: int
    prefix: str  # 行首 → '{' 之前（含），原样保留
    inner: str
    suffix: str  # '}' 之后（含行尾空格/注释/换行），原样保留
    values: Dict[str, Any]

    def render(self, values: Dict[str, Any]) -> str:
        inner = "x: %s, y: %s, yaw_deg: %s, radius: %s" % (
            format_number(values.get("x")),
            format_number(values.get("y")),
            format_number(values.get("yaw_deg")),
            format_number(values.get("radius")),
        )
        return self.prefix + "{" + inner + "}" + self.suffix


def points_block_bounds(lines: Sequence[str], where: str = "<text>") -> Tuple[int, int]:
    """返回顶层 `points:` 块的 [首行, 结束行) 行号（结束行 = 下一个顶层键）。"""
    start = None
    for i, line in enumerate(lines):
        m = _TOP_LEVEL_KEY_RE.match(line)
        if m is not None and m.group(1) == "points":
            start = i
            break
    if start is None:
        raise CommandError("%s 里找不到顶层 `points:` 块" % where)
    end = len(lines)
    for j in range(start + 1, len(lines)):
        stripped = lines[j].strip()
        if stripped and not stripped.startswith("#") and not lines[j][:1].isspace():
            end = j
            break
    return start, end


def parse_point_lines(text: str, where: str = "<text>") -> Tuple[List[str], List[PointLine]]:
    lines = text.splitlines(keepends=True)
    start, end = points_block_bounds(lines, where)
    out: List[PointLine] = []
    for i in range(start + 1, end):
        raw = lines[i]
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        m = _POINT_LINE_RE.match(raw)
        if m is None:
            raise CommandError(
                "%s:%d `points:` 块里有非单行 flow-map 的条目；本工具不做改写，"
                "以免破坏注释：%r" % (where, i + 1, raw)
            )
        try:
            values = yaml.safe_load("{" + m.group(5) + "}")
        except Exception as exc:  # noqa: BLE001 - 报给操作员即可
            raise CommandError("%s:%d 点位行解析失败：%s" % (where, i + 1, exc)) from exc
        if not isinstance(values, dict):
            raise CommandError("%s:%d 点位行不是 flow-map：%r" % (where, i + 1, raw))
        out.append(
            PointLine(
                name=m.group(2),
                index=i,
                prefix=raw[: m.start(4)],
                inner=m.group(5),
                suffix=raw[m.end(6):],
                values=values,
            )
        )
    if not out:
        raise CommandError("%s 的 `points:` 块里没有任何点位行" % where)
    return lines, out


def read_points(path) -> Tuple[str, List[str], List[PointLine]]:
    p = Path(path)
    if not p.is_file():
        raise CommandError("点位文件不存在：%s" % p)
    text = p.read_text(encoding="utf-8")
    lines, point_lines = parse_point_lines(text, str(p))
    return text, lines, point_lines


def apply_point_updates(path, updates: Dict[str, Dict[str, Any]], dry_run: bool = False) -> Dict[str, Any]:
    """只重写 `updates` 里点名的行，其余字节逐字不动（非 dry_run 时先存 `.bak`）。

    updates: {点位名: {'x':..,'y':..,'yaw_deg':..,'radius':..}}
    返回 {'path','bak','written','changed':[{'name','old','new'}]}
    """
    _text, lines, point_lines = read_points(path)
    by_name = {p.name: p for p in point_lines}
    unknown = sorted(n for n in updates if n not in by_name)
    if unknown:
        raise CommandError(
            "点位不在 %s 的 points 块里：%s（可用：%s）"
            % (path, ", ".join(unknown), ", ".join(by_name))
        )
    changed: List[Dict[str, str]] = []
    for pl in point_lines:  # 按文件顺序改写，输出稳定
        if pl.name not in updates:
            continue
        new_line = pl.render(updates[pl.name])
        if new_line != lines[pl.index]:
            changed.append({"name": pl.name, "old": lines[pl.index], "new": new_line})
            lines[pl.index] = new_line
    result: Dict[str, Any] = {"path": str(path), "bak": None, "written": False, "changed": changed}
    if dry_run:
        return result
    p = Path(path)
    bak = p.with_name(p.name + ".bak")
    shutil.copy2(p, bak)
    p.write_bytes("".join(lines).encode("utf-8"))
    result["bak"] = str(bak)
    result["written"] = True
    return result


def load_points_values(path) -> Dict[str, Dict[str, Any]]:
    """读出现有点位数值（缺字段补 NaN），用来初始化打点会话。"""
    _text, _lines, point_lines = read_points(path)
    out: Dict[str, Dict[str, Any]] = {}
    for pl in point_lines:
        row = {k: float("nan") for k in _POINT_KEYS}
        for k in _POINT_KEYS:
            if k in pl.values:
                try:
                    row[k] = float(pl.values[k])
                except (TypeError, ValueError):
                    row[k] = float("nan")
        out[pl.name] = row
    return out


# ---------------------------------------------------------------------------
# waypoint_graph.yaml
# ---------------------------------------------------------------------------

GRAPH_HEADER = (
    "# 现场打点生成的路径图：节点 + 边 + 禁小陀螺区(斜坡/起伏段)\n"
    "# 由 tools/mark_points.py 生成；坐标均为 odom 系，单位 m。\n"
    "# nodes 名唯一；edges 无向、引用 nodes 名；spin_forbidden_zones 为轴对齐矩形。\n"
    "# 打点建议：沿可跑路线每 2~3 m 一个节点，路口/坡道出入口各放一个；\n"
    "#   本文件被整体重写前的上一版（含仓库里模板的注释）保存在同名 .bak。\n"
)


def _fnum(value: Any) -> str:
    return "%.3f" % float(value) if is_finite(value) else "null"


def _jstr(value: Any) -> str:
    return json.dumps(str(value), ensure_ascii=False)


yaml_quote = _jstr  # 供其它工具渲染 yaml 里的字符串用


def node_entry(name: Any, x: Any, y: Any) -> Dict[str, Any]:
    return {"name": str(name), "x": float(x), "y": float(y)}


def zone_entry(name: Any, x_min: Any, y_min: Any, x_max: Any, y_max: Any) -> Dict[str, Any]:
    return {
        "name": str(name),
        "x_min": float(x_min),
        "x_max": float(x_max),
        "y_min": float(y_min),
        "y_max": float(y_max),
    }


def render_waypoint_graph(nodes: Sequence[Dict[str, Any]], edges: Sequence[Sequence[str]],
                          zones: Sequence[Dict[str, Any]]) -> str:
    out = [GRAPH_HEADER]
    if nodes:
        out.append("nodes:\n")
        for n in nodes:
            out.append("  - {name: %s, x: %s, y: %s}\n" % (_jstr(n["name"]), _fnum(n["x"]), _fnum(n["y"])))
    else:
        out.append("nodes: []\n")
    if edges:
        out.append("edges:\n")
        for edge in edges:
            out.append("  - [%s, %s]\n" % (_jstr(edge[0]), _jstr(edge[1])))
    else:
        out.append("edges: []\n")
    if zones:
        out.append("spin_forbidden_zones:\n")
        for z in zones:
            out.append(
                "  - {name: %s, x_min: %s, x_max: %s, y_min: %s, y_max: %s}\n"
                % (_jstr(z["name"]), _fnum(z["x_min"]), _fnum(z["x_max"]),
                   _fnum(z["y_min"]), _fnum(z["y_max"]))
            )
    else:
        out.append("spin_forbidden_zones: []\n")
    return "".join(out)


def write_waypoint_graph(path, nodes, edges, zones, dry_run: bool = False) -> Dict[str, Any]:
    """整体重写路径图；文件已存在时先存 `.bak`。"""
    text = render_waypoint_graph(nodes, edges, zones)
    p = Path(path)
    result: Dict[str, Any] = {"path": str(p), "text": text, "bak": None, "written": False}
    if dry_run:
        return result
    if p.exists():
        bak = p.with_name(p.name + ".bak")
        shutil.copy2(p, bak)
        result["bak"] = str(bak)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(text.encode("utf-8"))
    result["written"] = True
    return result


def load_waypoint_graph(path) -> Optional[Tuple[List[Dict[str, Any]], List[List[str]], List[Dict[str, Any]]]]:
    p = Path(path)
    if not p.is_file():
        return None
    try:
        data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    except Exception as exc:  # noqa: BLE001
        raise CommandError("路径图 %s 解析失败：%s" % (p, exc)) from exc
    if not isinstance(data, dict):
        raise CommandError("路径图 %s 顶层不是 mapping" % p)
    nodes: List[Dict[str, Any]] = []
    for i, raw in enumerate(data.get("nodes") or []):
        try:
            nodes.append(node_entry(raw["name"], raw["x"], raw["y"]))
        except (TypeError, KeyError, ValueError) as exc:
            raise CommandError("路径图 %s nodes[%d] 缺 name/x/y：%r" % (p, i, raw)) from exc
    edges: List[List[str]] = []
    for i, raw in enumerate(data.get("edges") or []):
        try:
            edges.append([str(raw[0]), str(raw[1])])
        except (TypeError, IndexError) as exc:
            raise CommandError("路径图 %s edges[%d] 应该是 [a, b]：%r" % (p, i, raw)) from exc
    zones: List[Dict[str, Any]] = []
    for i, raw in enumerate(data.get("spin_forbidden_zones") or []):
        try:
            zones.append(zone_entry(raw["name"], raw["x_min"], raw["y_min"], raw["x_max"], raw["y_max"]))
        except (TypeError, KeyError, ValueError) as exc:
            raise CommandError(
                "路径图 %s spin_forbidden_zones[%d] 缺 name/x_min/x_max/y_min/y_max：%r" % (p, i, raw)
            ) from exc
    return nodes, edges, zones


# ---------------------------------------------------------------------------
# /clicked_point 点击
# ---------------------------------------------------------------------------


class ClickedPointListener:
    """订阅 /clicked_point，把点变换到 target_frame，供命令消费一次。

    `handle()` 是纯逻辑（不 import ROS），可无 ROS 单测；`attach()` 才真正建订阅。
    `on_point(x, y)` 回调用于「点击即配对」的标定流程。
    """

    def __init__(self, target_frame: str = "odom", transform_fn: Optional[Callable[[str, Any], Tuple[float, float]]] = None,
                 on_point: Optional[Callable[[float, float], None]] = None):
        self.target_frame = target_frame
        self._transform_fn = transform_fn
        self._on_point = on_point
        self._lock = threading.Lock()
        self._pending: Optional[Tuple[float, float, str, float]] = None
        self._last_error: Optional[str] = None
        self._last_raw_frame: Optional[str] = None
        self._count = 0
        self._sub = None

    def attach(self, node, topic: str = "/clicked_point", qos_depth: int = 10):
        from geometry_msgs.msg import PointStamped

        self._sub = node.create_subscription(PointStamped, topic, self.handle, qos_depth)
        return self._sub

    def detach(self) -> None:
        """放开订阅引用（进程退出前调用，避免销毁期还有回调在跑）。"""
        self._sub = None
        self._on_point = None

    def handle(self, msg) -> Optional[Tuple[float, float]]:
        raw_frame = str(getattr(msg.header, "frame_id", "") or "")
        x = float(msg.point.x)
        y = float(msg.point.y)
        with self._lock:
            self._last_raw_frame = raw_frame
        if raw_frame and raw_frame != self.target_frame:
            if self._transform_fn is None:
                self._warn(
                    "收到 frame=%s 的点击，但没有可用的 tf；请把 RViz 的 Fixed Frame 设成 %s"
                    % (raw_frame, self.target_frame)
                )
                return None
            try:
                x, y = self._transform_fn(raw_frame, msg.header.stamp, x, y)
            except Exception as exc:  # noqa: BLE001
                self._warn(
                    "点击点 frame=%s 无法变换到 %s（请把 RViz 的 Fixed Frame 设成 %s）：%s"
                    % (raw_frame, self.target_frame, self.target_frame, exc)
                )
                return None
        with self._lock:
            self._pending = (float(x), float(y), raw_frame or self.target_frame, time.time())
            self._count += 1
            callback = self._on_point
        if callback is not None:
            callback(float(x), float(y))
        return (float(x), float(y))

    def take(self) -> Optional[Tuple[float, float, str, float]]:
        with self._lock:
            item, self._pending = self._pending, None
        return item

    def peek(self) -> Optional[Tuple[float, float, str, float]]:
        with self._lock:
            return self._pending

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "count": self._count,
                "pending": self._pending,
                "last_error": self._last_error,
                "last_raw_frame": self._last_raw_frame,
            }

    def _warn(self, text: str) -> None:
        with self._lock:
            self._last_error = text
        print("[警告] " + text, file=sys.stderr, flush=True)


def transform_point_2d(x: float, y: float, translation, rotation) -> Tuple[float, float]:
    """按 tf 的 (translation, rotation) 把 2D 点从 source frame 变到 target frame。"""
    yaw = math.radians(yaw_deg_from_quaternion(rotation))
    c, s = math.cos(yaw), math.sin(yaw)
    return (x * c - y * s + float(translation.x), x * s + y * c + float(translation.y))


def make_tf_xy_lookup(buffer, target_frame: str) -> Callable[[str, Any, float, float], Tuple[float, float]]:
    """构造 `transform_fn(frame, stamp, x, y) -> (x, y)`：用 tf 把 source frame 的点变到 target_frame。"""

    def _lookup(frame: str, stamp: Any, x: float, y: float) -> Tuple[float, float]:
        from rclpy.time import Time

        candidates = []
        if stamp is not None:
            try:
                candidates.append(Time.from_msg(stamp))
            except Exception:  # noqa: BLE001 - 有的消息 stamp 是 0，直接走 latest
                pass
        candidates.append(Time())
        last_exc = None
        for when in candidates:
            try:
                tf = buffer.lookup_transform(target_frame, frame, when)
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
                continue
            return transform_point_2d(x, y, tf.transform.translation, tf.transform.rotation)
        raise RuntimeError("lookup_transform(%s <- %s) 失败：%s" % (target_frame, frame, last_exc))

    return _lookup


# ---------------------------------------------------------------------------
# tf 位姿（odom -> vehicle）
# ---------------------------------------------------------------------------


def yaw_deg_from_quaternion(q) -> float:
    return math.degrees(
        math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    )


class TfPoseProvider:
    """tf parent -> child 的 2D 位姿（x, y, yaw_deg），带等待超时。"""

    def __init__(self, buffer, parent: str = "odom", child: str = "vehicle", timeout: float = 3.0):
        self.buffer = buffer
        self.parent = parent
        self.child = child
        self.timeout = float(timeout)

    def available(self) -> bool:
        try:
            from rclpy.time import Time

            return bool(self.buffer.can_transform(self.parent, self.child, Time()))
        except Exception:  # noqa: BLE001
            return False

    def get_pose(self, timeout: Optional[float] = None) -> Tuple[float, float, float]:
        from rclpy.time import Time

        deadline = time.time() + (self.timeout if timeout is None else float(timeout))
        last_exc: Optional[Exception] = None
        while True:
            try:
                tf = self.buffer.lookup_transform(self.parent, self.child, Time())
                t = tf.transform.translation
                return (float(t.x), float(t.y), yaw_deg_from_quaternion(tf.transform.rotation))
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
            if time.time() >= deadline:
                raise CommandError(
                    "tf %s -> %s 不可用（等了 %.1f s）：%s" % (self.parent, self.child, self.timeout, last_exc)
                )
            time.sleep(0.05)


# ---------------------------------------------------------------------------
# 脚本化位姿（--no-ros 测试模式）
# ---------------------------------------------------------------------------


class ScriptedPoseProvider:
    """从文件/标准输入按行取位姿，格式 `x y yaw_deg`（yaw 可省略 → NaN）。每取一次消费一行。"""

    def __init__(self, path, where: str = ""):
        self.source = str(path)
        self.where = where or self.source
        self._fh = None
        self._lines: Optional[List[str]] = None
        if self.source == "-":
            self._fh = sys.stdin
        else:
            p = Path(self.source)
            if not p.is_file():
                raise CommandError("位姿文件不存在：%s" % p)
            self._lines = [
                ln for ln in p.read_text(encoding="utf-8").splitlines()
                if ln.strip() and not ln.strip().startswith("#")
            ]
        self._index = 0

    def _next_line(self) -> Optional[str]:
        if self._lines is not None:
            if self._index >= len(self._lines):
                return None
            line = self._lines[self._index]
            self._index += 1
            return line
        while True:
            line = self._fh.readline()
            if not line:
                return None
            if line.strip() and not line.strip().startswith("#"):
                self._index += 1
                return line

    def remaining(self) -> Optional[int]:
        if self._lines is None:
            return None
        return max(0, len(self._lines) - self._index)

    def get_pose(self) -> Tuple[float, float, float]:
        line = self._next_line()
        if line is None:
            raise CommandError(
                "位姿已用完（%s 里第 %d 次请求）：请在脚本里补一行 `x y yaw_deg`"
                % (self.where, self._index + 1)
            )
        parts = line.replace(",", " ").split()
        if len(parts) < 2:
            raise CommandError("位姿行格式应为 `x y [yaw_deg]`，实际：%r" % line)
        try:
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) > 2 else float("nan")
        except ValueError as exc:
            raise CommandError("位姿行数值解析失败：%r（%s）" % (line, exc)) from exc
        return (x, y, yaw)


# ---------------------------------------------------------------------------
# 2D 刚体对齐（Umeyama/SVD，无缩放）
# ---------------------------------------------------------------------------


@dataclass
class RigidFit:
    """把 src(map) 刚体变换到 dst(odom) 的结果：p_odom = R * p_map + t。"""

    dx: float
    dy: float
    dyaw_deg: float
    r: np.ndarray
    t: np.ndarray
    residuals: np.ndarray
    rms: float
    n: int

    def transform(self, x: float, y: float) -> Tuple[float, float]:
        out = self.r @ np.array([float(x), float(y)]) + self.t
        return (float(out[0]), float(out[1]))

    def inverse_transform(self, x: float, y: float) -> Tuple[float, float]:
        out = self.r.T @ (np.array([float(x), float(y)]) - self.t)
        return (float(out[0]), float(out[1]))


def rotation_2d(deg: float) -> np.ndarray:
    rad = math.radians(float(deg))
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s], [s, c]], dtype=float)


def rigid_align_2d(src: Sequence[Sequence[float]], dst: Sequence[Sequence[float]]) -> RigidFit:
    """估计 p_odom = R(dyaw) * p_map + [dx, dy]（只含旋转+平移，无缩放）。

    src = map 系地标点，dst = 对应 odom 系地标点（同序、同数量）。
    """
    p = np.asarray(src, dtype=float).reshape(-1, 2)
    q = np.asarray(dst, dtype=float).reshape(-1, 2)
    if p.shape != q.shape:
        raise CommandError("map/odom 地标数量不一致：%d vs %d" % (p.shape[0], q.shape[0]))
    n = int(p.shape[0])
    if n < 2:
        raise CommandError("至少需要 2 组同名点才能估计旋转（当前 %d 组）" % n)
    pc = p.mean(axis=0)
    qc = q.mean(axis=0)
    if float(np.max(np.abs(p - pc))) < 1e-9:
        raise CommandError("map 侧地标点几乎重合，无法解算旋转")
    m = (p - pc).T @ (q - qc)
    u, singular, vt = np.linalg.svd(m)
    v = vt.T
    d = np.sign(np.linalg.det(v @ u.T))
    r = v @ np.diag([1.0, d]) @ u.T
    if np.linalg.det(r) < 0:  # 数值兜底：不允许镜像
        r = -r
    t = qc - r @ pc
    residuals = np.linalg.norm((r @ p.T).T + t - q, axis=1)
    rms = float(math.sqrt(float(np.mean(residuals ** 2)))) if n else float("nan")
    dyaw = math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0])))
    return RigidFit(dx=float(t[0]), dy=float(t[1]), dyaw_deg=dyaw, r=r, t=t,
                    residuals=residuals, rms=rms, n=n)


def refusal_reason(n: int, rms: float, min_n: int = 3, max_rms: float = 0.30) -> Optional[str]:
    """对齐结果的写入门槛：不合格时返回原因，合格返回 None。"""
    if n < min_n:
        return "地标组数 n=%d < %d：2D 刚体解算至少需要 %d 组同名点，拒绝写入" % (n, min_n, min_n)
    if not is_finite(rms):
        return "残差 RMS 不是有限值（%r），拒绝写入" % (rms,)
    if rms > max_rms:
        return "残差 RMS=%.3f m > %.2f m：地图/里程计对齐不可信，拒绝写入" % (rms, max_rms)
    return None
