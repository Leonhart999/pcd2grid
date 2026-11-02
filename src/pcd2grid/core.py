"""Core conversion: point cloud (N, 3) -> 2D occupancy grid.

Height-band projection:
  1. Optionally rotate/translate the cloud so that +z is up.
  2. Estimate the ground (single plane, local per-tile planes, or none).
  3. Cells with enough points whose height above ground lies in [obs_min, obs_max]
     are occupied.
  4. Cells that contain ground points and are not occupied are free.
  5. Everything else is unknown.
"""
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

OCC, FREE, UNKNOWN = 0, 254, 205  # pgm gray values: black = occupied, white = free, gray = unknown


@dataclass
class GridParams:
    res: float = 0.05
    obs_min: float = 0.1
    obs_max: float = 0.7
    ground_tol: float = 0.05
    min_points: int = 2
    ground_mode: str = "plane"          # plane | grid | none
    tile: float = 5.0                   # tile size in metres for ground_mode=grid
    up_axis: str = "z"                  # x, y, z, -x, -y, -z
    transform: np.ndarray = None        # optional 4x4 matrix applied before projection
    crop: tuple = None                  # (xmin, xmax, ymin, ymax, zmin, zmax); None entries are open
    voxel: float = 0.0                  # voxel downsampling size in metres, 0 = off
    z_slab: tuple = None                # (zmin, zmax) keep only this absolute height range
    inflate: float = 0.0                # obstacle inflation radius in metres
    origin: tuple = None                # (x, y) override for the lower-left corner
    closing_iters: int = 2


_UP_ROT = {
    "z": np.eye(3),
    "-z": np.diag([1.0, -1.0, -1.0]),
    "y": np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]]),
    "-y": np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]]),
    "x": np.array([[0, 0, -1.0], [0, 1.0, 0], [1.0, 0, 0]]),
    "-x": np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]]),
}


def orient(xyz, up_axis="z", transform=None):
    """Apply the up-axis rotation, then an optional 4x4 transform."""
    if up_axis not in _UP_ROT:
        raise ValueError(f"invalid up axis '{up_axis}'")
    xyz = xyz @ _UP_ROT[up_axis].T
    if transform is not None:
        t = np.asarray(transform, dtype=np.float64)
        if t.shape != (4, 4):
            raise ValueError("transform must be a 4x4 matrix")
        xyz = xyz @ t[:3, :3].T + t[:3, 3]
    return xyz


def voxel_downsample(xyz, size):
    """Keep one point per voxel (the first one found)."""
    if size <= 0:
        return xyz
    keys = np.floor(xyz / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[np.sort(idx)]


def apply_crop(xyz, crop):
    if crop is None:
        return xyz
    m = np.ones(len(xyz), dtype=bool)
    for axis in range(3):
        lo, hi = crop[2 * axis], crop[2 * axis + 1]
        if lo is not None:
            m &= xyz[:, axis] >= lo
        if hi is not None:
            m &= xyz[:, axis] <= hi
    return xyz[m]


def estimate_ground(z, bin_size=0.02):
    """Height of the most populated z histogram bin."""
    lo, hi = np.percentile(z, 1), np.percentile(z, 99)
    if hi - lo < bin_size:
        return float(np.median(z))
    hist, edges = np.histogram(z, bins=np.arange(lo, hi + bin_size, bin_size))
    k = int(np.argmax(hist))
    return 0.5 * (edges[k] + edges[k + 1])


def fit_ground_plane(xyz, z0, tols=(0.2, 0.08, 0.05)):
    """Iteratively fit z = a*x + b*y + c, tightening the inlier band each round."""
    a = b = 0.0
    c = z0
    for tol in tols:
        resid = xyz[:, 2] - (a * xyz[:, 0] + b * xyz[:, 1] + c)
        m = np.abs(resid) < tol
        if m.sum() < 3:
            break
        A = np.c_[xyz[m, 0], xyz[m, 1], np.ones(m.sum())]
        a, b, c = np.linalg.lstsq(A, xyz[m, 2], rcond=None)[0]
    return a, b, c


def height_above_ground(xyz, mode, tile):
    """Return per-point height above the estimated ground and a short description."""
    if mode == "none":
        return xyz[:, 2].copy(), "no ground estimation (absolute z)"
    g0 = estimate_ground(xyz[:, 2])
    a, b, c = fit_ground_plane(xyz, g0)
    h = xyz[:, 2] - (a * xyz[:, 0] + b * xyz[:, 1] + c)
    if mode == "plane":
        tilt = np.degrees(np.arctan(np.hypot(a, b)))
        return h, f"ground plane tilt {tilt:.2f} deg, height {c:.3f} m at the origin"
    if mode != "grid":
        raise ValueError(f"invalid ground mode '{mode}'")
    # Per-tile plane fit; tiles with too few ground points fall back to the global plane
    tx = np.floor((xyz[:, 0] - xyz[:, 0].min()) / tile).astype(np.int64)
    ty = np.floor((xyz[:, 1] - xyz[:, 1].min()) / tile).astype(np.int64)
    key = tx * (ty.max() + 1) + ty
    out = h.copy()
    n_local = 0
    for k in np.unique(key):
        sel = key == k
        pts = xyz[sel]
        lo = np.percentile(pts[:, 2], 5)
        near = pts[np.abs(pts[:, 2] - (lo + 0.1)) < 0.25]
        if len(near) < 30:
            continue
        zl = estimate_ground(near[:, 2])
        la, lb, lc = fit_ground_plane(pts, zl)
        out[sel] = pts[:, 2] - (la * pts[:, 0] + lb * pts[:, 1] + lc)
        n_local += 1
    return out, f"local ground planes fitted in {n_local} tiles of {tile:g} m"


@dataclass
class GridResult:
    grid: np.ndarray                # (H, W) uint8, row 0 = lowest y
    res: float
    origin: tuple                   # (x, y) of the lower-left corner
    info: str = ""
    stats: dict = field(default_factory=dict)

    @property
    def image(self):
        """Image orientation: first row is the largest y."""
        return np.flipud(self.grid)


def convert(xyz, p=None):
    """Convert an (N, 3) point cloud to a GridResult."""
    p = p or GridParams()
    xyz = orient(np.asarray(xyz, dtype=np.float64), p.up_axis, p.transform)
    xyz = apply_crop(xyz, p.crop)
    if p.z_slab is not None:
        xyz = xyz[(xyz[:, 2] >= p.z_slab[0]) & (xyz[:, 2] <= p.z_slab[1])]
    xyz = voxel_downsample(xyz, p.voxel)
    if len(xyz) < 10:
        raise ValueError("too few points left after filtering")

    h, ground_info = height_above_ground(xyz, p.ground_mode, p.tile)

    if p.origin is not None:
        xmin, ymin = p.origin
        xmax, ymax = xyz[:, :2].max(axis=0) + 0.5
    else:
        xmin, ymin = xyz[:, :2].min(axis=0) - 0.5
        xmax, ymax = xyz[:, :2].max(axis=0) + 0.5
    w = int(np.ceil((xmax - xmin) / p.res))
    hgt = int(np.ceil((ymax - ymin) / p.res))
    ix = np.floor((xyz[:, 0] - xmin) / p.res).astype(np.int64)
    iy = np.floor((xyz[:, 1] - ymin) / p.res).astype(np.int64)
    inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < hgt)
    ix, iy, h = ix[inside], iy[inside], h[inside]

    obs = (h >= p.obs_min) & (h <= p.obs_max)
    gnd = np.abs(h) <= p.ground_tol
    obs_count = np.zeros((hgt, w), dtype=np.int32)
    np.add.at(obs_count, (iy[obs], ix[obs]), 1)
    gnd_count = np.zeros((hgt, w), dtype=np.int32)
    np.add.at(gnd_count, (iy[gnd], ix[gnd]), 1)

    occupied = obs_count >= p.min_points
    free = (gnd_count > 0) & ~occupied
    if p.closing_iters > 0:
        free = ndimage.binary_closing(free, structure=np.ones((3, 3)), iterations=p.closing_iters) & ~occupied
    if p.inflate > 0:
        r = int(np.ceil(p.inflate / p.res))
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        disk = (xx * xx + yy * yy) <= r * r
        occupied = ndimage.binary_dilation(occupied, structure=disk)
        free &= ~occupied

    grid = np.full((hgt, w), UNKNOWN, dtype=np.uint8)
    grid[free] = FREE
    grid[occupied] = OCC
    total = w * hgt
    stats = {"width": w, "height": hgt, "points": int(len(xyz)),
             "occupied": float(occupied.sum() / total), "free": float(free.sum() / total)}
    return GridResult(grid=grid, res=p.res, origin=(float(xmin), float(ymin)), info=ground_info, stats=stats)
