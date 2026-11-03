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
    ground_z: float = None              # seed height for the ground fit; None = automatic
    max_cells: int = 100_000_000        # refuse grids larger than this
    level: bool = False                 # rotate the cloud so the detected floor is horizontal
    layers: bool = False                # keep per-cell input views for a comparison image


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


def lowest_per_cell(xyz, cell):
    """Lowest point of every `cell` x `cell` column. Floor points dominate this set even when
    walls or the ceiling have far more points, because those are never the lowest in a column."""
    k = np.floor(xyz[:, :2] / cell).astype(np.int64)
    key = (k[:, 0] - k[:, 0].min()) * (k[:, 1].max() - k[:, 1].min() + 1) + (k[:, 1] - k[:, 1].min())
    order = np.lexsort((xyz[:, 2], key))
    ks = key[order]
    first = np.r_[True, ks[1:] != ks[:-1]]
    return xyz[order][first]


def estimate_level_rotation(xyz, cell=0.5, tol=0.1, iters=2000, max_tilt=45.0, seed=0):
    """Find the floor plane by RANSAC over per-cell lowest points and return the rotation that
    makes its normal point along +z, together with the floor tilt in degrees."""
    q = lowest_per_cell(xyz, cell)
    if len(q) < 3:
        return np.eye(3), 0.0
    rng = np.random.default_rng(seed)
    best_inliers = None
    cos_max = np.cos(np.radians(max_tilt))
    for _ in range(iters):
        s = q[rng.choice(len(q), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9 or abs(n[2]) / norm < cos_max:
            continue
        inliers = np.abs((q - s[0]) @ (n / norm)) < tol
        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best_inliers = inliers
    if best_inliers is None or best_inliers.sum() < 3:
        return np.eye(3), 0.0
    # Refine with a total least-squares plane through the inliers
    pts = q[best_inliers]
    n = np.linalg.svd(pts - pts.mean(axis=0), full_matrices=False)[2][-1]
    if n[2] < 0:
        n = -n
    tilt = float(np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0))))
    # Rodrigues rotation taking n onto +z
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    s = np.linalg.norm(v)
    if s < 1e-12:
        return np.eye(3), tilt
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rot = np.eye(3) + vx + vx @ vx * ((1 - n @ z) / (s * s))
    return rot, tilt


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


def estimate_ground(z, bin_size=0.02, min_frac=0.2, window=0.1):
    """Height of the lowest strong peak of the z histogram.

    The most populated height is not always the floor: in a room scanned from a tripod the
    ceiling usually has more points. So smooth the histogram over `window` metres, keep the
    local maxima that reach `min_frac` of the highest one, and take the lowest of them.
    """
    lo, hi = np.percentile(z, 1), np.percentile(z, 99)
    if hi - lo < bin_size:
        return float(np.median(z))
    hist, edges = np.histogram(z, bins=np.arange(lo, hi + bin_size, bin_size))
    half = max(1, int(round(window / bin_size / 2)))
    smooth = np.convolve(hist, np.ones(2 * half + 1), mode="same")
    local_max = smooth == ndimage.maximum_filter1d(smooth, size=2 * half + 1, mode="constant")
    peaks = np.flatnonzero(local_max & (smooth >= min_frac * smooth.max()))
    first = peaks[0]
    # Refine to the most populated raw bin around the smoothed peak
    s, e = max(0, first - half), min(len(hist), first + half + 1)
    k = s + int(np.argmax(hist[s:e]))
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


def ground_warnings(h, a, b):
    """Plausibility checks on a fitted ground plane; returns a list of warning strings."""
    warnings = []
    tilt = np.degrees(np.arctan(np.hypot(a, b)))
    if tilt > 15.0:
        warnings.append(f"fitted ground is tilted {tilt:.1f} deg; try --level, or check --up-axis")
    below = float(np.mean(h < -0.3))
    if below > 0.2:
        warnings.append(f"{below:.0%} of the points lie more than 0.3 m below the fitted ground; "
                        "it may be a ceiling or table, try --ground-z")
    return warnings


def height_above_ground(xyz, mode, tile, ground_z=None):
    """Return per-point height above the estimated ground, a description and warnings."""
    if mode == "none":
        return xyz[:, 2].copy(), "no ground estimation (absolute z)", []
    g0 = estimate_ground(xyz[:, 2]) if ground_z is None else ground_z
    a, b, c = fit_ground_plane(xyz, g0)
    h = xyz[:, 2] - (a * xyz[:, 0] + b * xyz[:, 1] + c)
    warnings = ground_warnings(h, a, b)
    if mode == "plane":
        tilt = np.degrees(np.arctan(np.hypot(a, b)))
        return h, f"ground plane tilt {tilt:.2f} deg, height {c:.3f} m at the origin", warnings
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
    return out, f"local ground planes fitted in {n_local} tiles of {tile:g} m", warnings


@dataclass
class GridResult:
    grid: np.ndarray                # (H, W) uint8, row 0 = lowest y
    res: float
    origin: tuple                   # (x, y) of the lower-left corner
    info: str = ""
    stats: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)
    level_rotation: np.ndarray = None   # 3x3 rotation applied by --level, None if not used
    layers: dict = None                 # per-cell input views, only when GridParams.layers is set

    @property
    def image(self):
        """Image orientation: first row is the largest y."""
        return np.flipud(self.grid)


def convert(xyz, p=None):
    """Convert an (N, 3) point cloud to a GridResult."""
    p = p or GridParams()
    xyz = orient(np.asarray(xyz, dtype=np.float64), p.up_axis, p.transform)
    level_rot, level_info, pre_warnings = None, "", []
    if p.ground_mode != "none" and len(xyz) >= 3:
        rot, tilt = estimate_level_rotation(xyz)
        if p.level:
            # Rotation about the origin, so the map origin stays where the SLAM frame origin was
            level_rot = rot
            xyz = xyz @ level_rot.T
            level_info = f"levelled: floor was tilted {tilt:.2f} deg; "
        elif tilt > 5.0:
            pre_warnings.append(f"the floor looks tilted by {tilt:.1f} deg; the height band will cut through "
                                "it. Use --level to rotate the cloud first")
    xyz = apply_crop(xyz, p.crop)
    if p.z_slab is not None:
        xyz = xyz[(xyz[:, 2] >= p.z_slab[0]) & (xyz[:, 2] <= p.z_slab[1])]
    xyz = voxel_downsample(xyz, p.voxel)
    if len(xyz) < 10:
        raise ValueError("too few points left after filtering")

    if p.origin is not None:
        xmin, ymin = p.origin
        xmax, ymax = xyz[:, :2].max(axis=0) + 0.5
    else:
        xmin, ymin = xyz[:, :2].min(axis=0) - 0.5
        xmax, ymax = xyz[:, :2].max(axis=0) + 0.5
    w = int(np.ceil((xmax - xmin) / p.res))
    hgt = int(np.ceil((ymax - ymin) / p.res))
    # Check the size before allocating: georeferenced clouds (LAS) can span kilometres
    if w * hgt > p.max_cells:
        raise ValueError(f"grid would be {w} x {hgt} cells ({(xmax - xmin):.0f} m x {(ymax - ymin):.0f} m) "
                         f"at {p.res} m/cell, above the limit of {p.max_cells}; "
                         "use a larger --res, --crop the area, or raise --max-cells")

    h, ground_info, warnings = height_above_ground(xyz, p.ground_mode, p.tile, p.ground_z)
    warnings = pre_warnings + warnings
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

    layers = None
    if p.layers:
        # Per-cell views of the input for visual checking (see output.write_compare)
        top = np.full((hgt, w), -np.inf)
        np.maximum.at(top, (iy, ix), h)
        above_count = np.zeros((hgt, w), dtype=np.int32)
        np.add.at(above_count, (iy[h > p.obs_max], ix[h > p.obs_max]), 1)
        below_count = np.zeros((hgt, w), dtype=np.int32)
        np.add.at(below_count, (iy[h < -p.ground_tol], ix[h < -p.ground_tol]), 1)
        # Class per cell, highest priority last: 1 below ground, 2 above band, 3 ground, 4 obstacle band
        classes = np.zeros((hgt, w), dtype=np.uint8)
        classes[below_count > 0] = 1
        classes[above_count > 0] = 2
        classes[gnd_count > 0] = 3
        classes[obs_count > 0] = 4
        layers = {"top_height": np.where(np.isfinite(top), top, np.nan), "classes": classes}

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
    if p.ground_mode != "none" and not occupied.any():
        warnings.append("no occupied cells; the ground estimate or the --obs-min/--obs-max band may be wrong")
    return GridResult(grid=grid, res=p.res, origin=(float(xmin), float(ymin)), info=level_info + ground_info,
                      stats=stats, warnings=warnings, level_rotation=level_rot, layers=layers)
