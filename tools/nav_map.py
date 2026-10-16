#!/usr/bin/env python3
"""nav_map 栅格格式的**唯一**实现（切片 1 拥有，其他切片 import 复用）。

文件布局（`nav_map.bin`）::

    NAVMAP1\\n
    {"resolution":0.05,"origin":[x0,y0],"width":W,"height":H,"frame":"odom",
     "source":"...","arrays":[["cost","u1"],["dist","f4"],["slope","u1"]]}\\n
    <arrays 里的每个数组，按 json 的顺序，行主序(row 0 = 顶行)、小端、无填充>

- `cost`  uint8: 0 = 自由, 100 = 占据（膨胀前的原始占据栅格）
- `dist`  float32: 该格中心到最近占据格中心的精确欧氏距离（m）
- `slope` uint8: 1 = 坡道/台阶格（禁小陀螺区），否则 0

几何约定与 ROS map_server 一致（row 0 = 顶行 = 最大 y）::

    x = x0 + (col + 0.5) * r
    y = y0 + (height - 1 - row + 0.5) * r

配套的 `nav_map.pgm`(P5 255) + `nav_map.yaml` 给 RViz `Map` 显示用：
0 = 占据(黑), 254 = 自由, 205 = 未知；PGM 第 0 行也是顶行。

命令行用法（现场排查用，库本身不需要 CLI）::

    python3 tools/nav_map.py nav_map.bin            # 打印头部信息与统计
    python3 tools/nav_map.py nav_map.bin --query 3.0 4.0
    python3 tools/nav_map.py nav_map.bin --ascii-width 100
"""

import argparse
import json
import math
import os
import sys

import numpy as np

MAGIC = b"NAVMAP1\n"

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(TOOLS_DIR)
# tools/calib_align.py 现场产出的 map->odom 对齐文件（与 tools/marking_core.py 的常量同一路径）
DEFAULT_ALIGNMENT_FILE = os.path.join(
    REPO_ROOT, "src", "sentry_planner", "config", "map_alignment.yaml"
)

# json 的 arrays 字段里允许的 dtype 名
_DTYPES = {
    "u1": "<u1",
    "i1": "i1",
    "u2": "<u2",
    "i2": "<i2",
    "u4": "<u4",
    "i4": "<i4",
    "f4": "<f4",
    "f8": "<f8",
}
CANONICAL_ARRAYS = (("cost", "u1"), ("dist", "f4"), ("slope", "u1"))

# EDT 后端：装了 scipy 就用 ndimage.distance_transform_edt，否则用纯 numpy 的
# Felzenszwalb/Huttenlocher 精确可分离 EDT。
try:  # pragma: no cover - 取决于本机是否装了 scipy
    from scipy import ndimage as _ndimage

    SCIPY_AVAILABLE = True
except Exception:  # noqa: BLE001 - 任何 import 失败都退回纯 numpy
    _ndimage = None
    SCIPY_AVAILABLE = False


# --------------------------------------------------------------------------- #
# 栅格对象
# --------------------------------------------------------------------------- #
class NavMap:
    """导航栅格。行列约定见模块 docstring（row 0 = 顶行 = 最大 y）。"""

    __slots__ = (
        "cost",
        "dist",
        "slope",
        "unknown",
        "resolution",
        "origin",
        "frame",
        "source",
        "align",
        "arrays",
    )

    def __init__(
        self,
        cost,
        dist=None,
        slope=None,
        resolution=0.05,
        origin=(0.0, 0.0),
        frame="odom",
        source="",
        unknown=None,
        align=None,
        arrays=None,
    ):
        cost = np.asarray(cost)
        if cost.ndim != 2:
            raise ValueError(f"cost 必须是二维数组，实际 shape={cost.shape}")
        height, width = cost.shape
        if width <= 0 or height <= 0:
            raise ValueError(f"栅格尺寸必须为正，实际 {width}x{height}")

        self.cost = np.ascontiguousarray(cost, dtype=np.uint8)
        if dist is None:
            self.dist = np.zeros((height, width), dtype=np.float32)
        else:
            self.dist = _check_array(dist, (height, width), "dist").astype(np.float32, copy=True)
        if slope is None:
            self.slope = np.zeros((height, width), dtype=np.uint8)
        else:
            self.slope = _check_array(slope, (height, width), "slope").astype(np.uint8, copy=True)
        if unknown is not None:
            unknown = _check_array(unknown, (height, width), "unknown").astype(bool, copy=True)
        self.unknown = unknown

        self.resolution = float(resolution)
        if self.resolution <= 0.0:
            raise ValueError(f"resolution 必须为正，实际 {self.resolution}")
        origin = tuple(float(v) for v in np.asarray(origin).reshape(-1)[:2])
        if len(origin) != 2:
            raise ValueError(f"origin 必须是 [x0, y0]，实际 {origin}")
        self.origin = origin
        self.frame = str(frame)
        self.source = str(source)
        self.align = _normalize_align(align)
        self.arrays = tuple(tuple(a) for a in (arrays if arrays is not None else CANONICAL_ARRAYS))

    # -- 基本属性 ---------------------------------------------------------- #
    @property
    def width(self):
        return int(self.cost.shape[1])

    @property
    def height(self):
        return int(self.cost.shape[0])

    @property
    def shape(self):
        return (self.height, self.width)

    def __repr__(self):
        return (
            f"NavMap({self.width}x{self.height} r={self.resolution} origin={self.origin} "
            f"frame={self.frame!r} align={self.align} arrays={list(self.arrays)})"
        )

    # -- 几何查询（模块级同名函数是规范实现，这里只是转发） ------------------ #
    def world_to_cell(self, x, y):
        return world_to_cell(self, x, y)

    def cell_to_world(self, col, row):
        return cell_to_world(self, col, row)

    def in_bounds(self, col, row):
        return in_bounds(self, col, row)

    def is_free(self, x, y):
        return is_free(self, x, y)

    def clearance(self, x, y):
        return clearance(self, x, y)

    def line_clear(self, x0, y0, x1, y1, margin=0.0):
        return line_clear(self, x0, y0, x1, y1, margin)

    def render_ascii(self, width=100):
        return render_ascii(self, width)


def _check_array(arr, shape, name):
    arr = np.asarray(arr)
    if arr.shape != shape:
        raise ValueError(f"{name} 的 shape 应为 {shape}，实际 {arr.shape}")
    return arr


# --------------------------------------------------------------------------- #
# 几何：世界坐标 <-> 格子下标
# --------------------------------------------------------------------------- #
def world_to_cell(m, x, y):
    """世界坐标 -> (col, row)。超出栅格时返回越界下标（用 in_bounds 判断）。"""
    col = int(math.floor((float(x) - m.origin[0]) / m.resolution))
    row = m.height - 1 - int(math.floor((float(y) - m.origin[1]) / m.resolution))
    return col, row


def cell_to_world(m, col, row):
    """(col, row) -> 格中心的世界坐标 (x, y)。"""
    x = m.origin[0] + (int(col) + 0.5) * m.resolution
    y = m.origin[1] + (m.height - 1 - int(row) + 0.5) * m.resolution
    return x, y


def in_bounds(m, col, row):
    return 0 <= int(col) < m.width and 0 <= int(row) < m.height


def is_free(m, x, y):
    """(x, y) 所在格是否可通行（cost == 0）。越界视为不可通行（未知=阻挡）。"""
    col, row = world_to_cell(m, x, y)
    if not in_bounds(m, col, row):
        return False
    return bool(m.cost[row, col] == 0)


def clearance(m, x, y):
    """(x, y) 所在格到最近占据格的欧氏距离（m）。越界返回 0.0。"""
    col, row = world_to_cell(m, x, y)
    if not in_bounds(m, col, row):
        return 0.0
    return float(m.dist[row, col])


def _cells_along(m, x0, y0, x1, y1, step=None):
    """线段上的采样格 (rows, cols)，步长默认 r/2。"""
    r = m.resolution
    step = (r * 0.5) if step is None else float(step)
    dx = float(x1) - float(x0)
    dy = float(y1) - float(y0)
    length = math.hypot(dx, dy)
    n = max(1, int(math.ceil(length / step)))
    t = np.linspace(0.0, 1.0, n + 1)
    xs = float(x0) + t * dx
    ys = float(y0) + t * dy
    cols = np.floor((xs - m.origin[0]) / r).astype(np.int64)
    rows = (m.height - 1) - np.floor((ys - m.origin[1]) / r).astype(np.int64)
    return rows, cols


def line_clear(m, x0, y0, x1, y1, margin=0.0):
    """从 (x0,y0) 到 (x1,y1) 的直线是否可通行。

    以 r/2 步长采样，要求每个采样格：在界内、cost == 0、且 dist >= margin。
    注意 `margin` 是通道半宽（m）：0.0 只要求不撞格子，0.3 要求全程离墙 >= 0.3 m。
    """
    rows, cols = _cells_along(m, x0, y0, x1, y1)
    inside = (cols >= 0) & (cols < m.width) & (rows >= 0) & (rows < m.height)
    if not bool(inside.all()):
        return False
    if bool((m.cost[rows, cols] != 0).any()):
        return False
    margin = float(margin)
    if margin > 0.0 and bool((m.dist[rows, cols] < margin).any()):
        return False
    return True


# --------------------------------------------------------------------------- #
# 精确欧氏距离变换（EDT）
# --------------------------------------------------------------------------- #
def _edt_1d_sq(f):
    """Felzenszwalb & Huttenlocher 一维下包络，f 是平方距离（无源点为 inf）。"""
    n = int(f.shape[0])
    if n == 0:
        return np.zeros(0, dtype=np.float64)
    # 用有限大数代替 inf，保证抛物线求交不出 nan；真实平方距离 <= (n-1)^2 << BIG
    big = 1e12
    g = np.where(np.isfinite(f), f, big).astype(np.float64)
    v = np.zeros(n, dtype=np.int64)
    z = np.empty(n + 1, dtype=np.float64)
    k = 0
    v[0] = 0
    z[0] = -np.inf
    z[1] = np.inf
    for q in range(1, n):
        s = ((g[q] + q * q) - (g[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        while s <= z[k]:
            k -= 1
            s = ((g[q] + q * q) - (g[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = np.inf
    d = np.empty(n, dtype=np.float64)
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + g[v[k]]
    return d


def _edt_felzenszwalb(free, resolution):
    """纯 numpy 精确可分离 EDT：free=True 的格求到最近 occupied 格的距离（m）。"""
    big = 1e12
    f = np.where(free, np.inf, 0.0)
    # 先按行（axis=1）再按列（axis=0），两次一维 EDT 即可得到二维精确结果
    d = np.apply_along_axis(_edt_1d_sq, 1, f)
    d = np.apply_along_axis(_edt_1d_sq, 0, d)
    d = np.sqrt(d) * float(resolution)
    d[d >= big * 0.5] = np.inf
    return d


def edt_occupied(occupied, resolution, backend=None):
    """占据栅格 -> 每格到最近占据格中心的精确欧氏距离（m, float32）。

    `backend`: None/"auto" 自动（装了就 scipy），或 "scipy"/"felzenszwalb" 强制指定。
    栅格里一个占据格都没有时返回全 inf（未定义 -> 无穷）。
    """
    occ = np.asarray(occupied, dtype=bool)
    if occ.size == 0:
        raise ValueError("occupied 不能是空栅格")
    if backend in (None, "auto"):
        backend = "scipy" if SCIPY_AVAILABLE else "felzenszwalb"
    if not occ.any():
        return np.full(occ.shape, np.inf, dtype=np.float32)
    if backend == "scipy":
        if not SCIPY_AVAILABLE:
            raise RuntimeError("backend='scipy' 但本机没有 scipy.ndimage")
        d = _ndimage.distance_transform_edt(~occ, sampling=float(resolution))
    elif backend == "felzenszwalb":
        d = _edt_felzenszwalb(~occ, float(resolution))
    else:
        raise ValueError(f"未知 backend: {backend!r}")
    return np.asarray(d, dtype=np.float32)


# --------------------------------------------------------------------------- #
# map -> odom 的 2D 刚体对齐（tools/calib_align.py 现场产出 map_alignment.yaml）
# --------------------------------------------------------------------------- #
# 正向：p_odom = R(dyaw_deg) * p_map + [dx, dy]，角度逆时针为正。
# 官方 map.pcd 落在「官方录制时的 odin 起点系」（这里当 map 系），真车本次启动的 odom
# 原点未必与它重合，两者差的正是这个变换；导航侧只用 odom，所以建栅格前先把点云变过去。
def read_alignment(path):
    """读 map_alignment.yaml -> dict（file/dx/dy/dyaw_deg/rms_m/n）。字段缺失或非有限数直接报错。"""
    import yaml

    with open(os.fspath(path), "r", encoding="utf-8") as fh:
        meta = yaml.safe_load(fh) or {}
    if not isinstance(meta, dict):
        raise ValueError(f"{path}: 不是 map_alignment.yaml（顶层应为键值映射）")
    align = {"file": os.fspath(path)}
    for key in ("dx", "dy", "dyaw_deg"):
        if key not in meta:
            raise ValueError(f"{path}: 缺少 {key} 字段（需要 dx / dy / dyaw_deg）")
        value = float(meta[key])
        if not math.isfinite(value):
            raise ValueError(f"{path}: {key}={meta[key]!r} 不是有限数")
        align[key] = value
    align["rms_m"] = float(meta["rms_m"]) if meta.get("rms_m") is not None else None
    align["n"] = int(meta["n"]) if meta.get("n") is not None else None
    return align


def _normalize_align(align):
    """把 NavMap 构造/load 传进来的 align 规范化（None / dict / json 里的字段）。"""
    if align is None:
        return None
    if not isinstance(align, dict):
        raise ValueError(f"align 必须是 dict 或 None，实际 {type(align).__name__}")
    out = {
        "file": str(align.get("file", "")),
        "dx": float(align["dx"]),
        "dy": float(align["dy"]),
        "dyaw_deg": float(align["dyaw_deg"]),
    }
    if align.get("rms_m") is not None:
        out["rms_m"] = float(align["rms_m"])
    if align.get("n") is not None:
        out["n"] = int(align["n"])
    return out


def resolve_alignment(explicit=None, use_default=True, default_path=None):
    """决定用哪个对齐：显式指定 -> 必须存在；否则默认路径存在就用；都没有 -> None。

    `explicit` 给了但文件不存在、或默认文件存在却读不动，都会直接抛异常 ——
    宁愿现场报错，也不要静默按未对齐处理。
    """
    if explicit:
        path = os.fspath(explicit)
        if not os.path.exists(path):
            raise FileNotFoundError(f"找不到对齐文件: {path}")
        return read_alignment(path)
    if not use_default:
        return None
    path = os.fspath(default_path) if default_path else DEFAULT_ALIGNMENT_FILE
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return read_alignment(path)
    return None


def align_points(x, y, align):
    """批量把 map 系点变到 odom 系：p_odom = R(dyaw_deg)*p_map + [dx, dy]。align=None 原样返回。"""
    xs = np.asarray(x, dtype=np.float64)
    ys = np.asarray(y, dtype=np.float64)
    if align is None:
        return xs, ys
    rad = math.radians(float(align["dyaw_deg"]))
    c, s = math.cos(rad), math.sin(rad)
    return xs * c - ys * s + float(align["dx"]), xs * s + ys * c + float(align["dy"])


def align_point(x, y, align):
    """单点版本（标量进标量出），给现场手算/核对用。"""
    xs, ys = align_points(np.array([x], dtype=np.float64), np.array([y], dtype=np.float64), align)
    return float(xs[0]), float(ys[0])


def alignment_json(align):
    """json 头里的 align 字段（没用对齐时是 None）。"""
    if align is None:
        return None
    align = _normalize_align(align)
    return {
        "file": align["file"],
        "dx": align["dx"],
        "dy": align["dy"],
        "dyaw_deg": align["dyaw_deg"],
    }


def format_alignment(align):
    """人看的对齐摘要行内容（不含前缀标签）。"""
    if align is None:
        return "无（按 map.pcd 原始坐标系，frame=odom 仅当车起点与官方录制起点重合时成立）"
    align = _normalize_align(align)
    extra = ""
    if align.get("rms_m") is not None:
        extra += f", rms={align['rms_m']:.4f} m"
    if align.get("n") is not None:
        extra += f", n={align['n']}"
    return (
        f"dx={align['dx']:.4f} dy={align['dy']:.4f} dyaw={align['dyaw_deg']:.4f}deg "
        f"(map->odom, 来自 {align['file']}{extra})"
    )


# --------------------------------------------------------------------------- #
# 二进制格式：写 / 读
# --------------------------------------------------------------------------- #
def _atomic_write_bytes(path, blob):
    """先写同目录 .tmp 再 rename，避免读者看到半个文件。"""
    path = os.fspath(path)
    tmp = path + ".tmp"
    with open(tmp, "wb") as fh:
        fh.write(blob)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def _json_sidecar_path(path):
    base, ext = os.path.splitext(os.fspath(path))
    return base + ".json" if ext else os.fspath(path) + ".json"


def _arrays_bytes(m):
    out = bytearray()
    for name, dtype in m.arrays:
        arr = getattr(m, name, None)
        if arr is None:
            raise ValueError(f"arrays 里声明了 {name!r}，但对象里没有这个数组")
        npdt = _DTYPES.get(dtype)
        if npdt is None:
            raise ValueError(f"arrays 里的 dtype {dtype!r} 不支持")
        arr = np.ascontiguousarray(arr, dtype=npdt)
        out += arr.tobytes()
    return bytes(out)


def header_of(m):
    """返回写进 bin/json 的头部 dict（键顺序固定）。"""
    return {
        "resolution": round(float(m.resolution), 9),
        "origin": [float(m.origin[0]), float(m.origin[1])],
        "width": m.width,
        "height": m.height,
        "frame": m.frame,
        "source": m.source,
        "align": alignment_json(m.align),
        "arrays": [[name, dtype] for name, dtype in m.arrays],
    }


def write(
    path,
    cost=None,
    dist=None,
    slope=None,
    resolution=0.05,
    origin=(0.0, 0.0),
    frame="odom",
    source="",
    arrays=None,
    unknown=None,
    align=None,
    write_json=True,
):
    """写出 `nav_map.bin`（并默认写出同名的 `nav_map.json` 头副本）。

    `cost` 也可以直接传一个 NavMap（此时其余参数被忽略）。
    返回写进文件的头部 dict。

    注意：`unknown` 掩码不写进 .bin/.json —— 未知格在 `cost` 里已经是 100（未知=阻挡），
    掩码只在 `to_pgm()` 里用来画 205 那一档像素；`load()` 回来时 `unknown is None`。
    `align` 会以 {"file","dx","dy","dyaw_deg"} 记进 json 的 `align` 字段（没用对齐时是 null）。
    """
    if isinstance(cost, NavMap):
        m = cost
    else:
        if cost is None:
            raise ValueError("write() 需要 cost 数组或 NavMap 对象")
        m = NavMap(
            cost,
            dist=dist,
            slope=slope,
            resolution=resolution,
            origin=origin,
            frame=frame,
            source=source,
            unknown=unknown,
            align=align,
            arrays=arrays,
        )
    header = header_of(m)
    line = json.dumps(header, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    _atomic_write_bytes(path, MAGIC + line + b"\n" + _arrays_bytes(m))
    if write_json:
        _atomic_write_bytes(_json_sidecar_path(path), line + b"\n")
    return header


def write_map(m, path, write_json=True):
    """把已有 NavMap 写到 `path`。"""
    return write(path, m, write_json=write_json)


def load(path):
    """读 `nav_map.bin` -> NavMap。数组按 json 里的 `arrays` 顺序解析。"""
    with open(os.fspath(path), "rb") as fh:
        data = fh.read()
    if not data.startswith(MAGIC):
        raise ValueError(f"{path}: 开头不是 {MAGIC!r}，不是 nav_map.bin")
    end = data.find(b"\n", len(MAGIC))
    if end < 0:
        raise ValueError(f"{path}: 头部 json 行没有结束换行")
    try:
        header = json.loads(data[len(MAGIC) : end].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{path}: 头部 json 解析失败: {exc}") from exc

    width = int(header["width"])
    height = int(header["height"])
    n = width * height
    arrays = [(str(name), str(dtype)) for name, dtype in header["arrays"]]
    body = memoryview(data)[end + 1 :]

    out = {}
    offset = 0
    for name, dtype in arrays:
        npdt = _DTYPES.get(dtype)
        if npdt is None:
            raise ValueError(f"{path}: arrays 里的 dtype {dtype!r} 不支持")
        need = n * np.dtype(npdt).itemsize
        if offset + need > len(body):
            raise ValueError(
                f"{path}: {name!r} 数据不足（需要 {need} 字节，剩 {len(body) - offset} 字节）"
            )
        arr = np.array(np.frombuffer(body, dtype=npdt, count=n, offset=offset)).reshape(height, width)
        out[name] = arr
        offset += need
    if "cost" not in out:
        raise ValueError(f"{path}: arrays 里没有 cost")
    return NavMap(
        out["cost"],
        dist=out.get("dist"),
        slope=out.get("slope"),
        resolution=header["resolution"],
        origin=header["origin"],
        frame=header.get("frame", "odom"),
        source=header.get("source", ""),
        align=header.get("align"),
        arrays=arrays,
    )


# --------------------------------------------------------------------------- #
# PGM / map_server yaml
# --------------------------------------------------------------------------- #
def to_pgm(m, unknown=None):
    """NavMap -> P5 PGM 字节。0 = 占据(黑), 254 = 自由, 205 = 未知。"""
    img = np.where(m.cost != 0, np.uint8(0), np.uint8(254))
    unk = m.unknown if unknown is None else np.asarray(unknown, dtype=bool)
    if unk is not None:
        img[np.asarray(unk, dtype=bool)] = 205
    return b"P5\n%d %d\n255\n" % (m.width, m.height) + img.tobytes()


def _parse_pgm(data):
    """解析 P5 PGM 字节 -> (width, height, uint8 数组)。"""
    if isinstance(data, np.ndarray):
        arr = np.asarray(data)
        if arr.ndim != 2:
            raise ValueError("PGM 数组必须是二维")
        return arr.shape[1], arr.shape[0], arr.astype(np.uint8, copy=False)
    if not isinstance(data, (bytes, bytearray, memoryview)):
        with open(os.fspath(data), "rb") as fh:
            data = fh.read()
    data = bytes(data)

    fields = []
    i = 0
    while len(fields) < 4:
        # 跳过空白与 # 注释
        while i < len(data) and data[i : i + 1].isspace():
            i += 1
        if i < len(data) and data[i : i + 1] == b"#":
            while i < len(data) and data[i : i + 1] != b"\n":
                i += 1
            continue
        start = i
        while i < len(data) and not data[i : i + 1].isspace():
            i += 1
        if start == i:
            raise ValueError("PGM 头不完整")
        fields.append(data[start:i])
    magic, token_w, token_h, token_max = fields
    if magic != b"P5":
        raise ValueError(f"只支持 P5 PGM，实际 magic={magic!r}")
    width, height, maxval = int(token_w), int(token_h), int(token_max)
    if maxval != 255:
        raise ValueError(f"只支持 maxval=255，实际 {maxval}")
    i += 1  # 单个空白字符结束头
    need = width * height
    if len(data) - i < need:
        raise ValueError(f"PGM 像素数据不足：需要 {need} 字节，剩 {len(data) - i} 字节")
    img = np.frombuffer(data, dtype=np.uint8, count=need, offset=i).reshape(height, width)
    return width, height, np.array(img)


def from_pgm(
    data,
    resolution=0.05,
    origin=(0.0, 0.0),
    frame="odom",
    source="",
    occupied_thresh=0.65,
    free_thresh=0.196,
    negate=False,
):
    """PGM（路径/字节/数组）-> NavMap（只有 cost/unknown，dist、slope 为 0）。

    概率换算与 map_server 一致：p = (255 - v)/255（negate=1 时 p = v/255），
    p > occupied_thresh -> 占据(100)；p < free_thresh -> 自由(0)；其余 -> 未知。
    未知格在 cost 里记 100（未知=阻挡），同时记录在 `unknown` 掩码里。
    """
    width, height, img = _parse_pgm(data)
    p = img.astype(np.float64) / 255.0
    if not negate:
        p = 1.0 - p
    cost = np.full((height, width), 100, dtype=np.uint8)
    unknown = np.ones((height, width), dtype=bool)
    free = p < float(free_thresh)
    occ = p > float(occupied_thresh)
    cost[free] = 0
    unknown[free] = False
    unknown[occ] = False
    return NavMap(
        cost,
        resolution=resolution,
        origin=origin,
        frame=frame,
        source=source,
        unknown=unknown,
    )


def read_map_yaml(path):
    """读 map_server 风格 yaml -> dict（缺的键给默认值）。"""
    import yaml  # 只在真正用到时才依赖

    with open(os.fspath(path), "r", encoding="utf-8") as fh:
        meta = yaml.safe_load(fh) or {}
    if not isinstance(meta, dict):
        raise ValueError(f"{path}: 不是 map_server 风格的 yaml")
    origin = list(meta.get("origin", [0.0, 0.0, 0.0]))
    while len(origin) < 3:
        origin.append(0.0)
    return {
        "image": meta.get("image"),
        "resolution": float(meta.get("resolution", 0.05)),
        "origin": [float(v) for v in origin],
        "negate": int(meta.get("negate", 0)),
        "occupied_thresh": float(meta.get("occupied_thresh", 0.65)),
        "free_thresh": float(meta.get("free_thresh", 0.196)),
        "frame": str(meta.get("frame", "odom")),
        "mode": meta.get("mode"),
        "raw": meta,
    }


def write_pgm(m, pgm_path, yaml_path=None, unknown=None):
    """写 `nav_map.pgm` + `nav_map.yaml`（各自 tmp+rename），返回 yaml 路径。"""
    write_pgm_image = to_pgm(m, unknown=unknown)
    _atomic_write_bytes(pgm_path, write_pgm_image)
    if yaml_path is None:
        yaml_path = os.path.splitext(os.fspath(pgm_path))[0] + ".yaml"

    def fmt(v):
        return format(float(v), ".6g")

    text = (
        f"image: {os.path.basename(os.fspath(pgm_path))}\n"
        f"resolution: {fmt(m.resolution)}\n"
        f"origin: [{fmt(m.origin[0])}, {fmt(m.origin[1])}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    _atomic_write_bytes(yaml_path, text.encode("utf-8"))
    return yaml_path


def load_pgm(pgm_path, yaml_path=None):
    """读 `nav_map.pgm`（可选 yaml 提供 resolution/origin/thresh）-> NavMap。"""
    meta = {}
    if yaml_path is not None:
        meta = read_map_yaml(yaml_path)
        image = meta.get("image")
        if image and not os.path.isabs(image):
            # map_server 约定：image 相对 yaml 所在目录
            cand = os.path.join(os.path.dirname(os.path.abspath(os.fspath(yaml_path))), image)
            if os.path.exists(cand):
                pgm_path = cand
    out = from_pgm(
        pgm_path,
        resolution=meta.get("resolution", 0.05),
        origin=meta.get("origin", [0.0, 0.0, 0.0])[:2],
        frame=meta.get("frame", "odom"),
        source=os.fspath(pgm_path),
        occupied_thresh=meta.get("occupied_thresh", 0.65),
        free_thresh=meta.get("free_thresh", 0.196),
        negate=bool(meta.get("negate", 0)),
    )
    if yaml_path is not None:
        out.source = os.fspath(pgm_path)
    return out


# --------------------------------------------------------------------------- #
# 肉眼检查用
# --------------------------------------------------------------------------- #
def render_ascii(m, width=100, occupied_ch="#", free_ch="."):
    """把栅格粗粒度渲染成 ASCII（每个字符覆盖一小块，块里只要有一个占据就是 #）。"""
    cost = m.cost
    height, width_cells = cost.shape
    chars = max(1, min(int(width), width_cells))
    rows_out = max(1, int(round(height * chars / float(width_cells))))
    col_edges = np.linspace(0, width_cells, chars + 1).astype(np.int64)
    row_edges = np.linspace(0, height, rows_out + 1).astype(np.int64)
    lines = []
    for i in range(rows_out):
        r0, r1 = row_edges[i], max(row_edges[i + 1], row_edges[i] + 1)
        chars_row = []
        for j in range(chars):
            c0, c1 = col_edges[j], max(col_edges[j + 1], col_edges[j] + 1)
            block = cost[r0:r1, c0:c1]
            chars_row.append(occupied_ch if bool((block != 0).any()) else free_ch)
        lines.append("".join(chars_row))
    return "\n".join(lines)


def summary_lines(m, name="nav_map"):
    occupied = int(np.count_nonzero(m.cost))
    total = m.width * m.height
    free = total - occupied
    dist = m.dist[np.isfinite(m.dist)]
    lines = [
        f"{name}: {m.width} x {m.height} = {total} cells, r={m.resolution} m/cell"
        f", origin(左下角)=({m.origin[0]}, {m.origin[1]}), frame={m.frame!r}",
        f"  cost: occupied={occupied} ({occupied / total * 100:.1f}%)"
        f"  free={free} ({free / total * 100:.1f}%)",
        f"  slope cells: {int(np.count_nonzero(m.slope))}",
        f"  dist: max={float(dist.max()) if dist.size else float('nan'):.3f} m"
        f"  mean={float(dist.mean()) if dist.size else float('nan'):.3f} m"
        f"  inf={int(np.count_nonzero(~np.isfinite(m.dist)))}",
        f"  source: {m.source}",
        f"  align: {format_alignment(m.align)}",
    ]
    if m.unknown is not None:
        lines.append(f"  unknown cells: {int(np.count_nonzero(m.unknown))}")
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(description="nav_map 栅格文件查看/查询工具")
    parser.add_argument("path", help="nav_map.bin 路径")
    parser.add_argument("--query", nargs=2, type=float, metavar=("X", "Y"), help="查询该世界坐标")
    parser.add_argument("--ascii-width", type=int, default=0, help="打印 ASCII 渲染（列数，0=不打印）")
    parser.add_argument("--line", nargs=4, type=float, metavar=("X0", "Y0", "X1", "Y1"),
                        help="检查这两点之间是否可通行")
    parser.add_argument("--line-margin", type=float, default=0.0, help="--line 的通道半宽 (m)")
    parser.add_argument("--pgm", help="另外校验这个 PGM(+yaml) 的占据是否逐格一致")
    args = parser.parse_args(argv)

    m = load(args.path)
    for line in summary_lines(m, name=os.path.basename(args.path)):
        print(line)
    if args.query:
        x, y = args.query
        col, row = m.world_to_cell(x, y)
        print(
            f"query ({x}, {y}) -> cell(col={col}, row={row}) in_bounds={in_bounds(m, col, row)}"
            f" is_free={is_free(m, x, y)} clearance={clearance(m, x, y):.3f} m"
            f" slope={int(m.slope[row, col]) if in_bounds(m, col, row) else '-'}"
        )
    if args.line:
        x0, y0, x1, y1 = args.line
        ok = line_clear(m, x0, y0, x1, y1, args.line_margin)
        print(f"line_clear(({x0}, {y0}) -> ({x1}, {y1}), margin={args.line_margin}) = {ok}")
    if args.pgm:
        yaml_path = os.path.splitext(args.pgm)[0] + ".yaml"
        other = load_pgm(args.pgm, yaml_path if os.path.exists(yaml_path) else None)
        same = (other.cost.shape == m.cost.shape) and bool(np.array_equal(other.cost, m.cost))
        print(f"pgm 占据与 bin 逐格一致: {same}")
    if args.ascii_width > 0:
        print(render_ascii(m, args.ascii_width))
    return 0


if __name__ == "__main__":
    sys.exit(main())
