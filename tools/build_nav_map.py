#!/usr/bin/env python3
"""从现场点云 (`map.pcd`) 生成导航栅格 `nav_map.{bin,json,pgm,yaml}`。

流程（纯 numpy，无编译、无 pcl/ros 依赖；只需 python3+numpy）：

1. `tools/pcd_cloud_publisher.load_pcd()` 解析 PCD（复用已有实现，不重写）；
2. **map -> odom 对齐**：若 `src/sentry_planner/config/map_alignment.yaml` 存在（tools/calib_align.py
   现场产出），先按 `p_odom = R(dyaw_deg) * p_map + [dx, dy]` 把点云变到 odom 系；`--no-align`
   可关掉，`--align FILE` 可指定别的文件。对齐参数会记进 `nav_map.json` 的 `align` 字段；
3. 取点云 bbox 并外扩 `--pad`，原点向下对齐到 `--res` 的整数倍 -> 栅格原点(左下角)；
   每个格记录「点到过没有」（`seen`）与「z 带内点到过没有」；
4. `cost`：z 落在 [--obstacle-z-min, --obstacle-z-max] 的点打占据；**一个点都没有的格也算占据**
   （未知=阻挡，规划器不会跑到测绘范围外）；
5. `dist`：由原始占据算精确欧氏距离变换（见 nav_map.edt_occupied）；
6. `slope`：每格最大 z 网格（3x3 平滑）中心差分求坡度，超过 `--slope-deg` 即为坡道格；
   另外把格内 max-min z 跨度超过 `--step-height` 的格也算台阶格；
7. 四个文件各自 tmp+rename 原子写出（在 nav_map.py 里实现），随后回读校验，
   并打印摘要 + 粗粒度 ASCII 渲染，方便人眼核对场地结构。

用法::

    python3 tools/build_nav_map.py                       # 用默认 PCD/参数，写回 701_model 目录
    python3 tools/build_nav_map.py --no-align --dry-run  # 不套 map->odom 对齐，只算不写
    python3 tools/build_nav_map.py --align /tmp/map_alignment.yaml
"""

import argparse
import math
import os
import sys
import time

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(TOOLS_DIR)
DEFAULT_PCD = os.path.join(REPO_ROOT, "src", "sentry_planner", "map", "701_model", "map.pcd")
DEFAULT_OUTDIR = os.path.join(REPO_ROOT, "src", "sentry_planner", "map", "701_model")

if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)

import nav_map  # noqa: E402  (复用格式实现)

try:
    from pcd_cloud_publisher import load_pcd  # noqa: E402
except Exception as exc:  # noqa: BLE001
    print(f"无法 import tools/pcd_cloud_publisher.py: {exc}", file=sys.stderr)
    print("它需要 rclpy；先 source /opt/ros/kilted/setup.bash（或直接装 ROS 环境）再跑。", file=sys.stderr)
    raise


def _shift(a, di, dj, fill=0.0):
    """把数组平移 (di, dj)，越界用 fill 填充（不引入 scipy）。"""
    h, w = a.shape
    out = np.full_like(a, fill)
    r0, r1 = max(0, -di), h - max(0, di)
    c0, c1 = max(0, -dj), w - max(0, dj)
    out[r0 + di : r1 + di, c0 + dj : c1 + dj] = a[r0:r1, c0:c1]
    return out


def _box_smooth(values, valid, k):
    """对 values 做 (2k+1)^2 均值，只用 valid 的邻居（空处不留 0，避免假坡度）。"""
    if k <= 0:
        return np.where(valid, values, 0.0)
    vals = np.where(valid, values, 0.0).astype(np.float64)
    vmask = valid.astype(np.float64)
    total = np.zeros_like(vals)
    count = np.zeros_like(vals)
    for di in range(-k, k + 1):
        for dj in range(-k, k + 1):
            total += _shift(vals, di, dj, 0.0)
            count += _shift(vmask, di, dj, 0.0)
    return total / np.maximum(count, 1.0)


def _cell_max_min(values, flat, n_cells):
    """按格分组求 max/min（排序 + reduceat，比 np.maximum.at 快）。"""
    order = np.argsort(flat, kind="stable")
    f = flat[order]
    v = values[order]
    starts = np.flatnonzero(np.r_[True, f[1:] != f[:-1]])
    vmax = np.full(n_cells, -np.inf)
    vmin = np.full(n_cells, np.inf)
    vmax[f[starts]] = np.maximum.reduceat(v, starts)
    vmin[f[starts]] = np.minimum.reduceat(v, starts)
    return vmax, vmin


def build(args):
    t0 = time.time()
    align = None
    if not getattr(args, "no_align", False):
        try:
            align = nav_map.resolve_alignment(args.align or None, use_default=True)
        except (OSError, ValueError) as exc:
            raise SystemExit(f"对齐文件不可用: {exc}") from exc
    arr, _width, _height, _fields = load_pcd(args.pcd)
    names = arr.dtype.names or ()
    for axis in ("x", "y", "z"):
        if axis not in names:
            raise SystemExit(f"{args.pcd}: 点云缺少 {axis} 字段（实际字段 {names}）")

    x = np.asarray(arr["x"], dtype=np.float64)
    y = np.asarray(arr["y"], dtype=np.float64)
    z = np.asarray(arr["z"], dtype=np.float64)
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    n_total, n_bad = int(x.size), int(x.size - np.count_nonzero(finite))
    x, y, z = x[finite], y[finite], z[finite]
    if x.size == 0:
        raise SystemExit(f"{args.pcd}: 没有有效点")
    # map -> odom：先把点云变过去，后面 bbox / 栅格 / 斜坡全在 odom 系里算
    x, y = nav_map.align_points(x, y, align)

    res = float(args.res)
    pad = float(args.pad)
    # 原点向下对齐到 res 的整数倍，取整到 1e-6 避免 0.9000000000000001 这种脏值
    x0 = round(math.floor((x.min() - pad) / res) * res, 6)
    y0 = round(math.floor((y.min() - pad) / res) * res, 6)
    x1 = x.max() + pad
    y1 = y.max() + pad
    width = max(1, int(math.ceil((x1 - x0) / res)))
    height = max(1, int(math.ceil((y1 - y0) / res)))
    n_cells = width * height

    print(f"PCD        : {args.pcd}")
    print(f"  有效点 {x.size} / {n_total}（非有限 {n_bad}）")
    print(f"对齐: {nav_map.format_alignment(align)}")
    print(
        f"  点云范围 x[{x.min():.3f}, {x.max():.3f}] y[{y.min():.3f}, {y.max():.3f}] "
        f"z[{z.min():.3f}, {z.max():.3f}]，外扩 pad={pad} m"
    )

    # 世界坐标 -> 格下标（row 0 = 最大 y，与 nav_map 约定一致）
    cols = np.floor((x - x0) / res).astype(np.int64)
    rows = (height - 1) - np.floor((y - y0) / res).astype(np.int64)
    inb = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    if not bool(inb.all()):
        print(f"  注意: {int((~inb).sum())} 个点落在栅格外（已忽略）")
    flat = (rows * width + cols)[inb]
    zi = z[inb]

    # 「有没有点到过这个格」
    seen = np.bincount(flat, minlength=n_cells)

    band = (zi >= float(args.obstacle_z_min)) & (zi <= float(args.obstacle_z_max))
    occ_flat = np.zeros(n_cells, dtype=bool)
    occ_flat[flat[band]] = True
    occupied = occ_flat.reshape(height, width)
    unknown_blocked = (seen == 0).reshape(height, width)
    occupied = occupied | unknown_blocked

    cmax_flat, cmin_flat = _cell_max_min(zi[band], flat[band], n_cells)
    cmax = cmax_flat.reshape(height, width)
    cmin = cmin_flat.reshape(height, width)
    has_band = np.isfinite(cmax)
    span = np.where(has_band, cmax - np.where(has_band, cmin, 0.0), 0.0)

    smoothed = _box_smooth(cmax, has_band, int(args.smooth))
    g_row, g_col = np.gradient(smoothed, res, res)  # 行方向(向下=减小 y)、列方向
    slope_deg = np.degrees(np.arctan(np.hypot(g_row, g_col)))
    is_slope = ((slope_deg > float(args.slope_deg)) | (span > float(args.step_height))) & has_band

    cost = np.where(occupied, np.uint8(100), np.uint8(0))
    slope = is_slope.astype(np.uint8)
    dist = nav_map.edt_occupied(occupied, res, backend=args.edt_backend)

    src = os.path.relpath(os.path.abspath(args.pcd), REPO_ROOT)
    if src.startswith(".."):
        src = os.path.abspath(args.pcd)
    m = nav_map.NavMap(
        cost,
        dist=dist,
        slope=slope,
        resolution=res,
        origin=(x0, y0),
        frame=args.frame,
        source=src,
        align=align,
    )

    occupied_n = int(np.count_nonzero(m.cost))
    free_n = n_cells - occupied_n
    free_frac = free_n / n_cells
    print(f"z 带       : [{float(args.obstacle_z_min):.2f}, {float(args.obstacle_z_max):.2f}] m")
    print(
        f"分辨率     : {res} m/cell   栅格原点(左下角) = ({x0}, {y0})   坐标系 = {args.frame}"
    )
    print(f"栅格大小   : {width} x {height} = {n_cells} cells")
    print(
        f"占据/通行  : occupied={occupied_n} ({occupied_n / n_cells * 100:.1f}%)"
        f"  free={free_n} ({free_frac * 100:.1f}%)"
        f"  [其中无点的未知格 {int(unknown_blocked.sum())}]"
    )
    print(
        f"坡道/台阶格: {int(slope.sum())}  (坡度 > {float(args.slope_deg):g}° 或格内 z 跨度 > "
        f"{float(args.step_height):g} m)"
    )
    backend = args.edt_backend or ("scipy" if nav_map.SCIPY_AVAILABLE else "felzenszwalb")
    dist_finite = m.dist[np.isfinite(m.dist)]
    print(
        f"距离变换   : backend={backend}"
        f" (scipy {'可用' if nav_map.SCIPY_AVAILABLE else '不可用'})"
        f"  max={float(dist_finite.max()) if dist_finite.size else float('nan'):.3f} m"
        f"  平均={float(dist_finite.mean()) if dist_finite.size else float('nan'):.3f} m"
    )
    if not (0.1 < free_frac < 1.0):
        print(f"警告: 可通行占比 {free_frac:.3f} 不在 (0.1, 1.0) 内，请检查 --obstacle-z-min/max")

    if args.dry_run:
        print("--dry-run: 未写文件")
    else:
        os.makedirs(args.outdir, exist_ok=True)
        bin_path = os.path.join(args.outdir, "nav_map.bin")
        pgm_path = os.path.join(args.outdir, "nav_map.pgm")
        yaml_path = os.path.join(args.outdir, "nav_map.yaml")
        nav_map.write(bin_path, m)
        nav_map.write_pgm(m, pgm_path, yaml_path)
        print(f"输出       : {bin_path}")
        for p in (bin_path, os.path.join(args.outdir, "nav_map.json"), pgm_path, yaml_path):
            print(f"             {p}  ({os.path.getsize(p)} 字节)")

        back = nav_map.load(bin_path)
        same_bin = (
            np.array_equal(back.cost, m.cost)
            and np.array_equal(back.dist, m.dist)
            and np.array_equal(back.slope, m.slope)
            and back.resolution == m.resolution
            and back.origin == m.origin
            and back.frame == m.frame
        )
        back_pgm = nav_map.load_pgm(pgm_path, yaml_path)
        same_pgm = back_pgm.cost.shape == m.cost.shape and np.array_equal(
            back_pgm.cost != 0, m.cost != 0
        )
        same_geom = (
            back_pgm.resolution == m.resolution
            and tuple(back_pgm.origin) == tuple(m.origin)
            and back_pgm.width == m.width
            and back_pgm.height == m.height
        )
        print(f"回读校验   : bin 逐格一致={same_bin}  pgm 占据逐格一致={same_pgm}  几何一致={same_geom}")
        if not (same_bin and same_pgm and same_geom):
            raise SystemExit("回读校验失败")

    chars = max(1, min(int(args.ascii_width), width))
    rows_out = max(1, int(round(height * chars / float(width))))
    print(
        f"ASCII 渲染 : 每个字符约 {int(math.ceil(width / chars))} x {int(math.ceil(height / rows_out))} cells"
        f"  '#'=占据/未知  '.'=可通行   (顶行 = 最大 y = {y1:.2f})"
    )
    print(nav_map.render_ascii(m, chars))
    print(f"耗时       : {time.time() - t0:.2f} s")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="从 701 场地点云生成导航栅格 nav_map.{bin,json,pgm,yaml}",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--pcd", default=DEFAULT_PCD, help="输入点云（PCD）")
    parser.add_argument("--res", type=float, default=0.05, help="栅格分辨率 m/cell")
    parser.add_argument("--obstacle-z-min", type=float, default=0.30, help="占据判定 z 下限 (m)")
    parser.add_argument("--obstacle-z-max", type=float, default=3.00, help="占据判定 z 上限 (m)")
    parser.add_argument("--slope-deg", type=float, default=8.0, help="坡度阈值 (deg)")
    parser.add_argument("--outdir", default=DEFAULT_OUTDIR, help="输出目录")
    parser.add_argument("--ascii-width", type=int, default=100, help="ASCII 渲染宽度（字符数）")
    parser.add_argument("--pad", type=float, default=0.5, help="点云 bbox 外扩量 (m)")
    parser.add_argument("--step-height", type=float, default=0.15, help="格内 z 跨度台阶阈值 (m)")
    parser.add_argument("--smooth", type=int, default=1, help="最大 z 网格的盒式平滑半径（格）")
    parser.add_argument("--frame", default="odom", help="栅格坐标系名（写进 json）")
    parser.add_argument(
        "--align",
        default="",
        help="map_alignment.yaml（tools/calib_align.py 产出）：先把点云按 "
        "p_odom = R(dyaw)*p_map + [dx,dy] 变到 odom 系再建栅格",
    )
    parser.add_argument(
        "--no-align",
        action="store_true",
        help=f"忽略对齐（默认：{os.path.relpath(nav_map.DEFAULT_ALIGNMENT_FILE, REPO_ROOT)} 存在就自动使用）",
    )
    parser.add_argument(
        "--edt-backend",
        choices=("auto", "scipy", "felzenszwalb"),
        default="auto",
        help="距离变换实现",
    )
    parser.add_argument("--dry-run", action="store_true", help="只计算+打印，不写文件")
    args = parser.parse_args(argv)
    if args.edt_backend == "auto":
        args.edt_backend = None
    if args.align and args.no_align:
        parser.error("--align 与 --no-align 不能同时给")
    if args.align and not os.path.exists(args.align):
        parser.error(f"找不到对齐文件: {args.align}")
    if not os.path.exists(args.pcd):
        parser.error(f"找不到点云文件: {args.pcd}")
    return build(args)


if __name__ == "__main__":
    sys.exit(main())
