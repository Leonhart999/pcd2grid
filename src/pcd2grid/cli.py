"""Command line interface for pcd2grid."""
import argparse
import sys

import numpy as np

from . import __version__
from .core import GridParams, convert
from .io import read_points
from .output import write_compare, write_map


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
    ap.add_argument("--ground-mode", choices=["plane", "grid", "none"], default="plane",
                    help="plane: one global plane; grid: per-tile planes; none: absolute z")
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
            up_axis=args.up_axis, transform=args.transform, crop=args.crop, voxel=args.voxel,
            z_slab=args.z_slab, inflate=args.inflate, origin=args.origin, ground_z=args.ground_z,
            max_cells=args.max_cells, level=args.level, layers=args.compare)
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
    except (ValueError, ImportError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    s = result.stats
    print(result.info)
    for w in result.warnings:
        print(f"warning: {w}", file=sys.stderr)
    print(f"grid {s['width']} x {s['height']} cells ({s['width'] * result.res:.1f} m x "
          f"{s['height'] * result.res:.1f} m), occupied {s['occupied']:.1%}, free {s['free']:.1%}")
    print(f"lower-left origin ({result.origin[0]:.2f}, {result.origin[1]:.2f}); wrote " + ", ".join(files))
    return 0


if __name__ == "__main__":
    sys.exit(main())
