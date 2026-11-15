'''Occupancy grid from per-frame scans and per-frame poses (evidence per cell, not a union of points).

Input is one point cloud per frame in the sensor frame (a directory of .pcd/.ply/.npy/.las/.laz files)
and a TUM trajectory with the pose of the sensor frame in a level world frame (z up, one floor).

A cell is occupied only if obstacle-height points were seen there in enough frames, from a long enough
stretch of the path, and few beams of other frames passed through it. Height is measured from the floor
under the sensor of each frame, and points far from the sensor are not used.
'''
import argparse
import json
import os
import re
import sys
import time
from types import SimpleNamespace

import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation

from . import __version__
from .io import _READERS, read_points
from .output import write_map

OCC, FREE, UNKNOWN = 0, 254, 205
MAX_HITS = 65535   # the per-cell counters are uint16

PARAMS = [
    ('res', 0.05, 'cell size, m'),
    ('map_range', 20.0, 'points farther than this from the sensor are not used, m (angular error grows with range)'),
    ('obs_min', 0.10, 'minimum obstacle height above the local floor, m'),
    ('obs_max', 0.70, 'maximum obstacle height above the local floor, m'),
    ('ground_tol', 0.05, '|height| below this counts as floor, m'),
    ('scan_stride', 2, 'use every n-th frame'),
    ('ray_deg', 0.5, 'azimuth bin of the free-space rays, degrees'),
    ('min_hits', 4, 'number of frames that must see an obstacle point in a cell'),
    ('min_span', 1.0, 'path length between the first and the last frame that hit a cell, m'),
    ('occ_ratio', 0.10, 'minimum hits / (hits + beams that passed through); low, so that glass that is seen through stays'),
    ('robot_radius', 0.3, 'cells this close to the trajectory are free, m; 0 disables'),
    ('closing', 2, 'iterations of the 3x3 closing of the free space'),
    ('sensor_height', 0.0, 'sensor height above the floor, m; 0 = estimate from the scans'),
    ('max_cells', 40_000_000, 'refuse grids with more cells than this (before cropping)'),
]
DEFAULTS = {name: value for name, value, _ in PARAMS}


def check_params(p):
    '''Validated copy of a parameter dict (unknown keys are an error).'''
    unknown = sorted(set(p) - set(DEFAULTS))
    if unknown:
        raise ValueError('unknown parameters: ' + ', '.join(unknown))
    p = dict(p)
    for key in DEFAULTS:
        if not np.isfinite(p[key]):
            raise ValueError(key + ' must be finite')
    for key in ('scan_stride', 'min_hits', 'closing', 'max_cells'):
        p[key] = int(p[key])
    if p['res'] <= 0 or p['map_range'] <= 0:
        raise ValueError('res and map_range must be positive')
    if not 0 < p['ray_deg'] <= 90:
        raise ValueError('ray_deg must be in (0, 90]')
    if p['obs_min'] >= p['obs_max']:
        raise ValueError('obs_min must be below obs_max')
    if min(p['ground_tol'], p['robot_radius'], p['min_span'], p['sensor_height'], p['closing']) < 0:
        raise ValueError('ground_tol, robot_radius, min_span, sensor_height and closing must not be negative')
    if p['scan_stride'] < 1 or p['min_hits'] < 1 or p['max_cells'] < 1:
        raise ValueError('scan_stride, min_hits and max_cells must be at least 1')
    if not 0 <= p['occ_ratio'] <= 1:
        raise ValueError('occ_ratio must be in [0, 1]')
    return p


class ScanSet:
    '''Per-frame point clouds in the sensor frame, read one frame at a time.'''

    def __init__(self, count, loader, stamps=None):
        self.count = int(count)
        self.stamps = None if stamps is None else np.asarray(stamps, dtype=np.float64)
        if self.stamps is not None and len(self.stamps) != self.count:
            raise ValueError('one stamp per frame is needed')
        self._loader = loader

    def __len__(self):
        return self.count

    def points(self, k):
        '''Finite XYZ of frame k as an (N, 3) float64 array.'''
        xyz = np.asarray(self._loader(k), dtype=np.float64)
        if xyz.ndim != 2 or xyz.shape[1] < 3:
            raise ValueError('frame %d is not an (N, 3+) array' % k)
        xyz = xyz[:, :3]
        return xyz[np.isfinite(xyz).all(axis=1)]

    def subset(self, keep):
        keep = np.asarray(keep, dtype=np.int64)
        stamps = None if self.stamps is None else self.stamps[keep]
        return ScanSet(len(keep), lambda k: self._loader(int(keep[k])), stamps)

    def transformed(self, matrix):
        '''Frames mapped through a 4x4 transform (point cloud frame -> frame of the poses).'''
        m = np.asarray(matrix, dtype=np.float64)
        return ScanSet(self.count, lambda k: self.points(k) @ m[:3, :3].T + m[:3, 3], self.stamps)

    @classmethod
    def from_arrays(cls, arrays, stamps=None):
        arrays = list(arrays)
        return cls(len(arrays), lambda k: arrays[k], stamps)


def _natural_key(name):
    return [int(s) if s.isdigit() else s for s in re.split(r'(\d+)', name)]


_PLAIN_STAMP = re.compile(r'^\d+(\.\d+)?$')
_INDEXED_STAMP = re.compile(r'^\d+_(\d{10,})$')


def _stamps_from_names(stems):
    '''Frame timestamps in seconds from file stems; raises ValueError unless every stem carries one.

    A stem is a plain number (seconds) or <index>_<integer> as in 000123_1793427856000031948, where the
    integer is a Unix time in s, ms, us or ns (told apart by its number of digits).'''
    if all(_PLAIN_STAMP.match(x) for x in stems):
        return np.array([float(x) for x in stems])
    found = [_INDEXED_STAMP.match(x) for x in stems]
    if not all(found):
        raise ValueError('file names carry no timestamps')
    out = []
    for m in found:
        digits = m.group(1)
        n = len(digits)
        out.append(int(digits) * (1.0 if n <= 11 else 1e-3 if n <= 14 else 1e-6 if n <= 17 else 1e-9))
    return np.array(out)


def scans_from_directory(directory):
    '''Frames from the point cloud files of a directory.

    If every file stem carries a timestamp (a number in seconds, or <index>_<unix time> such as 000123_1793427856000031948 with the time in s, ms, us or ns) the frames are sorted by it;
    otherwise the frames are sorted by file name (natural order) and have no stamps.'''
    exts = sorted(_READERS)
    found = []
    for name in os.listdir(directory):
        low = name.lower()
        for ext in exts:
            if low.endswith(ext):
                found.append((name, name[:-len(ext)]))
                break
    if not found:
        raise ValueError('no point cloud files (' + ', '.join(exts) + ') in ' + str(directory))
    try:
        stamps = _stamps_from_names([stem for _, stem in found])
        if not np.isfinite(stamps).all():
            stamps = None
    except ValueError:
        stamps = None
    if stamps is not None:
        order = np.argsort(stamps, kind='stable')
        stamps = stamps[order]
    else:
        order = sorted(range(len(found)), key=lambda i: _natural_key(found[i][0]))
    paths = [os.path.join(directory, found[i][0]) for i in order]
    return ScanSet(len(paths), lambda k: read_points(paths[k]), stamps)


def read_tum(path):
    '''TUM trajectory (t x y z qx qy qz qw) -> sorted stamps, rotation matrices, positions.'''
    data = np.loadtxt(path, ndmin=2, comments='#')
    if data.size == 0 or data.shape[1] < 8:
        raise ValueError(str(path) + ' is not a TUM trajectory (8 columns: t x y z qx qy qz qw)')
    data = data[:, :8]
    if not np.isfinite(data).all():
        raise ValueError('trajectory contains non-finite values')
    if (np.linalg.norm(data[:, 4:8], axis=1) < 1e-6).any():
        raise ValueError('trajectory contains a zero quaternion')
    data = data[np.argsort(data[:, 0], kind='stable')]
    return data[:, 0], Rotation.from_quat(data[:, 4:8]).as_matrix(), data[:, 1:4]


def _nearest(stamps, t, max_dt):
    j = np.searchsorted(t, stamps)
    lo, hi = np.clip(j - 1, 0, len(t) - 1), np.clip(j, 0, len(t) - 1)
    pick = np.where(np.abs(t[hi] - stamps) < np.abs(t[lo] - stamps), hi, lo)
    return pick, np.abs(t[pick] - stamps) <= max_dt


def match_poses(scans, stamps, mode='auto', max_dt=0.05):
    '''Pair frames with trajectory poses; returns (frame indices, pose indices).

    stamp: the nearest pose in time within max_dt; frames without one are dropped.
    order: the k-th frame gets the k-th pose; the counts must be equal.
    auto: stamp if the file names are timestamps that fit the trajectory, otherwise order.'''
    if mode not in ('auto', 'stamp', 'order'):
        raise ValueError('match must be auto, stamp or order')
    if mode != 'order' and scans.stamps is not None:
        pick, ok = _nearest(scans.stamps, stamps, max_dt)
        if ok.sum() >= 2:
            return np.flatnonzero(ok), pick[ok]
        if mode == 'stamp':
            raise ValueError('fewer than two frames have a pose within max_dt of their stamp')
    elif mode == 'stamp':
        raise ValueError('the file names are not timestamps; use --match order')
    if len(scans) != len(stamps):
        raise ValueError('%d frames but %d poses; the file names do not match the trajectory stamps, '
                         'and matching by order needs equal counts' % (len(scans), len(stamps)))
    return np.arange(len(scans)), np.arange(len(scans))


def estimate_sensor_height(scans, rot, samples=200):
    '''Height of the sensor above the floor: lowest strong peak of the heights of nearby points.'''
    z = []
    for k in np.linspace(0, len(scans) - 1, min(samples, len(scans))).astype(int):
        d = scans.points(int(k))[::4] @ rot[k].T
        z.append(d[np.hypot(d[:, 0], d[:, 1]) < 4.0, 2])
    hist, edges = np.histogram(np.concatenate(z), bins=np.arange(-3.0, 0.0, 0.02))
    if hist.max() == 0:
        raise ValueError('no points 0 to 3 m below the sensor within 4 m; pass --sensor-height')
    k = int(np.flatnonzero(hist >= 0.5 * hist.max())[0])
    k += int(np.argmax(hist[k:k + 5]))
    return float(-0.5 * (edges[k] + edges[k + 1]))


def accumulate(scans, rot, pos, p, log=print):
    '''Per-cell evidence: frames with an obstacle hit, beams passing through, floor hits, path span of the hits.'''
    res, rng = p['res'], p['map_range']
    hs = p['sensor_height'] or estimate_sensor_height(scans, rot)
    lo = pos[:, :2].min(axis=0) - rng - 0.5
    hi = pos[:, :2].max(axis=0) + rng + 0.5
    w, h = (int(v) for v in np.ceil((hi - lo) / res))
    if w * h > p['max_cells']:
        raise ValueError('grid of %d x %d cells is too large; use a coarser res or a smaller map_range' % (w, h))
    xmin, ymin = lo
    hit = np.zeros(w * h, np.uint16)
    miss = np.zeros(w * h, np.uint16)
    gnd = np.zeros(w * h, np.uint16)
    s_lo = np.full(w * h, np.inf, np.float32)
    s_hi = np.full(w * h, -np.inf, np.float32)
    path = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1))]
    nbin = int(round(360 / p['ray_deg']))
    steps = np.arange(0.0, rng, res)
    ang = (np.arange(nbin) + 0.5) * (2 * np.pi / nbin) - np.pi
    ray_dx, ray_dy = np.outer(np.cos(ang), steps), np.outer(np.sin(ang), steps)

    def cells(x, y):
        ix = np.floor((x - xmin) / res).astype(np.int64)
        iy = np.floor((y - ymin) / res).astype(np.int64)
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        return np.unique(iy[ok] * w + ix[ok])

    t0 = time.time()
    frames = range(0, len(scans), p['scan_stride'])
    for done, k in enumerate(frames):
        b = scans.points(k)
        b = b[np.einsum('ij,ij->i', b, b) < rng * rng]
        if not len(b):
            continue
        d = b @ rot[k].T                       # offset from the sensor in world axes
        height = d[:, 2] + hs                  # above the floor under this frame's sensor
        x, y = d[:, 0] + pos[k, 0], d[:, 1] + pos[k, 1]
        band = (height >= p['obs_min']) & (height <= p['obs_max'])
        floor = np.abs(height) <= p['ground_tol']
        c = cells(x[band], y[band])
        hit[c] += 1
        s_lo[c] = np.minimum(s_lo[c], path[k])
        s_hi[c] = np.maximum(s_hi[c], path[k])
        gnd[cells(x[floor], y[floor])] += 1
        # Free-space rays: per azimuth bin, up to one cell before the nearest obstacle-height return.
        rho = np.hypot(d[band, 0], d[band, 1])
        ab = ((np.arctan2(d[band, 1], d[band, 0]) + np.pi) / (2 * np.pi) * nbin).astype(int) % nbin
        near = np.full(nbin, np.inf)
        np.minimum.at(near, ab, rho)
        seen = np.isfinite(near)
        through = steps[None, :] < (near[seen, None] - res)
        cm = cells((pos[k, 0] + ray_dx[seen])[through], (pos[k, 1] + ray_dy[seen])[through])
        miss[cm[~np.isin(cm, c, assume_unique=True)]] += 1
        if done % 500 == 0:
            log('  grid evidence: frame %d/%d (%.0f s)' % (done, len(frames), time.time() - t0))
    shape = (h, w)
    span = np.where(hit > 0, s_hi - s_lo, 0).astype(np.float32)
    return dict(hit=hit.reshape(shape), miss=miss.reshape(shape), gnd=gnd.reshape(shape),
                span=span.reshape(shape), origin=np.array([xmin, ymin]), sensor_height=hs)


def driven_mask(shape, origin, pos, p):
    '''Cells within robot_radius of the trajectory.'''
    driven = np.zeros(shape, bool)
    if p['robot_radius'] <= 0:
        return driven
    ix = np.floor((pos[:, 0] - origin[0]) / p['res']).astype(int)
    iy = np.floor((pos[:, 1] - origin[1]) / p['res']).astype(int)
    driven[iy, ix] = True
    r = int(round(p['robot_radius'] / p['res']))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return ndimage.binary_dilation(driven, structure=xx * xx + yy * yy <= r * r)


def classify(ev, pos, p):
    '''Occupied / free / unknown grid (uint8, row 0 = lowest y) from the evidence.'''
    hit, miss = ev['hit'].astype(np.float32), ev['miss'].astype(np.float32)
    driven = driven_mask(hit.shape, ev['origin'], pos, p)
    occupied = ((hit >= p['min_hits']) & (ev['span'] >= p['min_span'])
                & (hit >= p['occ_ratio'] * (hit + miss)) & ~driven)
    free = ((ev['gnd'] > 0) | (ev['miss'] >= 2) | driven) & ~occupied
    if p['closing']:
        free = ndimage.binary_closing(free, structure=np.ones((3, 3)), iterations=p['closing']) & ~occupied
    grid = np.full(hit.shape, UNKNOWN, np.uint8)
    grid[free] = FREE
    grid[occupied] = OCC
    return grid


def crop_to_content(grid, origin, res, margin=0.5):
    ys, xs = np.where(grid != UNKNOWN)
    m = int(round(margin / res))
    y0, y1 = max(ys.min() - m, 0), min(ys.max() + m + 1, grid.shape[0])
    x0, x1 = max(xs.min() - m, 0), min(xs.max() + m + 1, grid.shape[1])
    return grid[y0:y1, x0:x1], origin + np.array([x0, y0]) * res


def read_matrix(source):
    '''4x4 rigid transform from an array, a txt/npy file or 16 comma-separated numbers.'''
    try:
        if isinstance(source, np.ndarray):
            m = source
        elif ',' in str(source):
            m = np.array([float(v) for v in str(source).split(',')])
        elif str(source).lower().endswith('.npy'):
            m = np.load(source, allow_pickle=False)
        else:
            m = np.loadtxt(source)
        m = np.asarray(m, dtype=np.float64).reshape(4, 4)
    except (ValueError, OSError) as e:
        raise ValueError('cannot read a 4x4 matrix from %s: %s' % (source, e)) from e
    rot = m[:3, :3]
    if not (np.isfinite(m).all() and np.allclose(m[3], [0, 0, 0, 1])
            and np.allclose(rot.T @ rot, np.eye(3), atol=1e-3) and np.linalg.det(rot) > 0):
        raise ValueError('the matrix must be a rigid transform (rotation + translation, last row 0 0 0 1)')
    return m


def _matrix_arg(text):
    try:
        return read_matrix(text)
    except ValueError as e:
        raise argparse.ArgumentTypeError(str(e)) from e


def build_grid(scans, trajectory, out_prefix, params=None, match='auto', max_dt=0.05, mode='trinary',
               sensor_to_body=None,
               preview=True, save_npy=False, save_evidence=False, log=print):
    '''Grid from per-frame scans (a directory or a ScanSet) and a TUM trajectory.

    Writes <out_prefix>.pgm and .yaml (plus preview, npy and evidence on request); returns the statistics.'''
    p = check_params(dict(DEFAULTS, **(params or {})))
    if isinstance(scans, (str, os.PathLike)):
        scans = scans_from_directory(scans)
    if sensor_to_body is not None:
        scans = scans.transformed(read_matrix(sensor_to_body))
    stamps, rot, pos = read_tum(trajectory)
    frame, pose = match_poses(scans, stamps, match, max_dt)
    if len(frame) < len(scans):
        log('  %d of %d frames have no pose and are skipped' % (len(scans) - len(frame), len(scans)))
    scans, rot, pos = scans.subset(frame), rot[pose], pos[pose]
    used = len(range(0, len(scans), p['scan_stride']))
    if used > MAX_HITS:
        raise ValueError('%d frames is too many for the 16-bit counters; raise scan_stride' % used)
    os.makedirs(os.path.dirname(os.path.abspath(out_prefix)), exist_ok=True)
    source = 'given' if p['sensor_height'] else 'estimated'
    ev = accumulate(scans, rot, pos, p, log)
    if not (ev['hit'].any() or ev['gnd'].any()):
        raise ValueError('no points within the range and height limits; check the poses and the sensor height')
    if save_evidence:
        np.savez(out_prefix + '_evidence.npz', **ev)
    grid, origin = crop_to_content(classify(ev, pos, p), ev['origin'], p['res'])
    result = SimpleNamespace(image=np.ascontiguousarray(np.flipud(grid)), grid=grid, res=p['res'], origin=origin)
    files = write_map(result, out_prefix, mode=mode, preview=preview, save_npy=save_npy)
    p['sensor_height'] = float(ev['sensor_height'])
    return dict(params=p, trajectory=os.path.basename(str(trajectory)), frames_matched=len(scans), frames_used=used,
                sensor_height_source=source, width=int(grid.shape[1]), height=int(grid.shape[0]),
                origin=[round(float(origin[0]), 3), round(float(origin[1]), 3), 0.0],
                occupied=float((grid == OCC).mean()), free=float((grid == FREE).mean()), files=files)


def build_parser():
    ap = argparse.ArgumentParser(
        prog='pcd2grid-scans',
        description='Occupancy grid (pgm + yaml) for Nav2 from one point cloud per frame and a TUM trajectory.')
    ap.add_argument('scans', help='directory with one point cloud per frame in the sensor frame '
                                  '(.pcd .ply .npy .las .laz); numeric file names are frame timestamps')
    ap.add_argument('trajectory', help='TUM trajectory (t x y z qx qy qz qw): pose of the sensor frame '
                                       'in a level world frame, z up')
    ap.add_argument('out_prefix', help='output prefix; writes <prefix>.pgm and <prefix>.yaml')
    ap.add_argument('--match', choices=('auto', 'stamp', 'order'), default='auto',
                    help='how frames get poses: by timestamp, by order, or by timestamp when the file names '
                         'are timestamps that fit the trajectory (default auto)')
    ap.add_argument('--max-dt', type=float, default=0.05,
                    help='largest time difference between a frame and its pose, s (default 0.05)')
    ap.add_argument('--sensor-to-body', type=_matrix_arg, metavar='FILE|16_NUMBERS',
                    help='4x4 transform (txt/npy file or 16 comma-separated values) from the frame of the point '
                         'clouds into the frame the trajectory poses belong to, e.g. lidar -> IMU/base')
    for name, value, text in PARAMS:
        ap.add_argument('--' + name.replace('_', '-'), type=type(value), default=value,
                        help='%s (default %s)' % (text, value))
    ap.add_argument('--mode', choices=['trinary', 'scale', 'raw'], default='trinary',
                    help='map_server mode written to the yaml (default trinary)')
    ap.add_argument('--no-preview', action='store_true', help='do not write <prefix>_preview.png')
    ap.add_argument('--npy', action='store_true', help='also write <prefix>.npy (occupancy 0..100, -1 unknown)')
    ap.add_argument('--save-evidence', action='store_true', help='also write <prefix>_evidence.npz')
    ap.add_argument('--report', help='JSON file for the parameters and statistics')
    ap.add_argument('--version', action='version', version='%(prog)s ' + __version__)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    params = {name: getattr(args, name) for name in DEFAULTS}
    try:
        stats = build_grid(args.scans, args.trajectory, args.out_prefix, params, match=args.match,
                           max_dt=args.max_dt, mode=args.mode, sensor_to_body=args.sensor_to_body,
                           preview=not args.no_preview,
                           save_npy=args.npy, save_evidence=args.save_evidence)
        if args.report:
            with open(args.report, 'w', encoding='utf-8') as stream:
                json.dump({'version': __version__, 'stats': stats}, stream, indent=2, allow_nan=False)
    except (ValueError, ImportError, OSError) as e:
        print('error: %s' % e, file=sys.stderr)
        return 1
    print('frames: %d matched, %d used; sensor height %.2f m (%s)' % (
        stats['frames_matched'], stats['frames_used'], stats['params']['sensor_height'],
        stats['sensor_height_source']))
    print('grid %d x %d cells (%.1f m x %.1f m), occupied %.1f%%, free %.1f%%' % (
        stats['width'], stats['height'], stats['width'] * args.res, stats['height'] * args.res,
        100 * stats['occupied'], 100 * stats['free']))
    print('lower-left origin (%.2f, %.2f); wrote %s' % (stats['origin'][0], stats['origin'][1],
                                                      ', '.join(stats['files'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
