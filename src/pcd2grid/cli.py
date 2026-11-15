"""Command line interface for pcd2grid."""
import argparse
import json
import sys

import numpy as np

from . import __version__
from .core import GridParams, convert
from .io import read_points
from .output import write_compare, write_map
from .processing import read_trajectory


def _floats(text, n, what):
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != n:
        raise argparse.ArgumentTypeError(f"{what} needs {n} comma-separated values")
    return [None if p in ("", "none", "None") else float(p) for p in parts]


def _crop(text):
    return tuple(_floats(text, 6, "--crop (xmin,xmax,ymin,ymax,zmin,zmax)"))


def _pair(text):
    return tuple(_floats(text, 2, "value"))


def _matrix(text):
    """Load a 4x4 matrix from a .txt/.npy file or from 16 comma-separated numbers."""
    if text.endswith(".npy"):
        return np.load(text)
    if "," in text:
        return np.array([float(v) for v in text.split(",")]).reshape(4, 4)
    return np.loadtxt(text).reshape(4, 4)


def build_parser():
    ap = argparse.ArgumentParser(
        prog="pcd2grid",
        description="Convert a 3D point cloud map into a 2D occupancy grid (pgm + yaml) for Nav2.")
    ap.add_argument("input", help="point cloud file (.pcd, .ply, .las, .laz, .npy)")
    ap.add_argument("out_prefix", help="output prefix; writes <prefix>.pgm and <prefix>.yaml")
    ap.add_argument("--res", type=float, default=0.05, help="cell size in metres (default 0.05)")
    ap.add_argument("--obs-min", type=float, default=0.1, help="minimum obstacle height above ground, m")
    ap.add_argument("--obs-max", type=float, default=0.7, help="maximum obstacle height above ground, m")
    ap.add_argument("--ground-tol", type=float, default=0.05, help="max distance from ground to count as ground, m")
    ap.add_argument("--min-points", type=int, default=2, help="obstacle points needed to occupy a cell")
    ap.add_argument("--ground-mode", choices=["plane", "grid", "heightmap", "none", "auto"], default="plane",
                    help="plane: 全局平面；grid: 分块平面；heightmap: 连续高度图；none: 绝对 z；"
                         "auto: 点云长边超过 --auto-ground-size 时用 grid，否则用 plane")
    ap.add_argument("--auto-ground-size", type=float, default=100.0,
                    help="--ground-mode auto 切换到分块的长边阈值，m（默认 100）")
    ap.add_argument("--ground-cell", type=float, default=0.5, help="地面高度图格宽，m")
    ap.add_argument("--ground-smooth", type=float, default=0.5, help="地面高度图高斯平滑尺度，m")
    ap.add_argument("--min-obstacle-area", type=float, default=0.0, help="删除小于此面积的孤立障碍，m²，默认关闭")
    ap.add_argument("--min-free-area", type=float, default=0.0, help="删除小于此面积的孤立空闲块，m²")
    ap.add_argument("--start", type=_pair, help="输出地图坐标系中的连通性检查起点 X,Y")
    ap.add_argument("--robot-radius", type=float, default=0.0, help="连通性检查的圆形机器人半径，m；不修改地图")
    ap.add_argument("--trajectory", help="与点云同坐标系的 TUM 轨迹，保守补全空闲区域")
    ap.add_argument("--ray-range", type=float, default=10.0, help="轨迹到端点最大三维距离，m")
    ap.add_argument("--max-rays", type=int, default=200_000, help="最大近似射线数")
    ap.add_argument("--report", help="保存参数、警告和统计的 JSON 文件")
    ap.add_argument("--tile", type=float, default=5.0, help="tile size for --ground-mode grid, m")
    ap.add_argument("--ground-z", type=float, help="approximate ground height to start the fit from "
                    "(default: lowest strong peak of the height histogram)")
    ap.add_argument("--level", action="store_true",
                    help="detect the floor plane and rotate the cloud so it is horizontal; the rotation "
                         "is saved to <prefix>_level.txt")
    ap.add_argument("--max-cells", type=int, default=100_000_000, help="refuse grids with more cells than this")
    ap.add_argument("--up-axis", choices=["x", "y", "z", "-x", "-y", "-z"], default="z",
                    help="which input axis points up (default z)")
    ap.add_argument("--transform", type=_matrix, metavar="FILE|16_NUMBERS",
                    help="4x4 transform applied after --up-axis (txt/npy file or 16 comma-separated values)")
    ap.add_argument("--crop", type=_crop, metavar="XMIN,XMAX,YMIN,YMAX,ZMIN,ZMAX",
                    help="crop box; leave a value empty or 'none' to keep it open")
    ap.add_argument("--z-slab", type=_pair, metavar="ZMIN,ZMAX",
                    help="keep only points in this absolute height range (e.g. one floor)")
    ap.add_argument("--voxel", type=float, default=0.0, help="voxel downsampling size in metres (0 = off)")
    ap.add_argument("--inflate", type=float, default=0.0, help="obstacle inflation radius in metres")
    ap.add_argument("--origin", type=_pair, metavar="X,Y", help="force the lower-left corner of the map")
    ap.add_argument("--origin-yaw", type=float, default=0.0, help="yaw written to the yaml origin, rad")
    ap.add_argument("--mode", choices=["trinary", "scale", "raw"], default="trinary",
                    help="map_server mode written to the yaml (default trinary)")
    ap.add_argument("--no-preview", action="store_true", help="do not write <prefix>_preview.png")
    ap.add_argument("--compare", action="store_true",
                    help="also write <prefix>_compare.png: point cloud top view, point classes and grid side by side")
    ap.add_argument("--npy", action="store_true", help="also write <prefix>.npy (occupancy 0..100, -1 unknown)")
    ap.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        xyz = read_points(args.input)
        params = GridParams(
            res=args.res, obs_min=args.obs_min, obs_max=args.obs_max, ground_tol=args.ground_tol,
            min_points=args.min_points, ground_mode=args.ground_mode, tile=args.tile,
            auto_ground_size=args.auto_ground_size,
            up_axis=args.up_axis, transform=args.transform, crop=args.crop, voxel=args.voxel,
            z_slab=args.z_slab, inflate=args.inflate, origin=args.origin, ground_z=args.ground_z,
            max_cells=args.max_cells, level=args.level, layers=args.compare,
            ground_cell=args.ground_cell, ground_smooth=args.ground_smooth,
            min_obstacle_area=args.min_obstacle_area, min_free_area=args.min_free_area,
            start=args.start, robot_radius=args.robot_radius,
            trajectory=read_trajectory(args.trajectory) if args.trajectory else None,
            ray_range=args.ray_range, max_rays=args.max_rays)
        print(f"read {len(xyz)} points from {args.input}")
        result = convert(xyz, params)
        files = write_map(result, args.out_prefix, mode=args.mode, origin_yaw=args.origin_yaw,
                          preview=not args.no_preview, save_npy=args.npy)
        if args.compare:
            files.append(write_compare(result, args.out_prefix + "_compare.png", obs_max=args.obs_max))
        if result.level_rotation is not None:
            # 4x4 transform from the input frame (after --up-axis/--transform) to the map frame
            t = np.eye(4)
            t[:3, :3] = result.level_rotation
            np.savetxt(args.out_prefix + "_level.txt", t, fmt="%.9f")
            files.append(args.out_prefix + "_level.txt")
        if args.report:
            config = {key: value.tolist() if isinstance(value, np.ndarray) else value
                      for key, value in vars(args).items()}
            with open(args.report, "w", encoding="utf-8") as stream:
                json.dump({"version": __version__, "parameters": config, "stats": result.stats,
                           "warnings": result.warnings, "info": result.info},
                          stream, ensure_ascii=False, indent=2, allow_nan=False)
            files.append(args.report)
    except (ValueError, ImportError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    s = result.stats
    print(result.info)
    for w in result.warnings:
        print(f"warning: {w}", file=sys.stderr)
    print(f"grid {s['width']} x {s['height']} cells ({s['width'] * result.res:.1f} m x "
          f"{s['height'] * result.res:.1f} m), occupied {s['occupied']:.1%}, free {s['free']:.1%}")
    if "connectivity" in s:
        print("连通性: " + json.dumps(s["connectivity"], ensure_ascii=False, allow_nan=False))
    if "rays" in s:
        print(f"有效近似射线: {s['rays']}")
    print(f"lower-left origin ({result.origin[0]:.2f}, {result.origin[1]:.2f}); wrote " + ", ".join(files))
    return 0


if __name__ == "__main__":
    sys.exit(main())
