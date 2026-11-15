'''Tests for pcd2grid-scans: evidence grid from per-frame scans and poses.'''
import os
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image
from scipy.spatial.transform import Rotation

from pcd2grid import scans as sc

OCC, FREE = sc.OCC, sc.FREE
FAST = dict(map_range=9.0, ray_deg=1.0, scan_stride=1)
FAST_ARGS = ['--map-range', '9', '--ray-deg', '1', '--scan-stride', '1']
BOX = (1.0, 1.5, -1.0, -0.5)   # x0, x1, y0, y1 of a box that is 0.5 m high
SENSOR_HEIGHT = 0.5


def quiet(message):
    pass


def _world():
    '''A 10 x 10 m room with 2 m walls, a floor and a box; no occlusion.'''
    g = np.arange(-5.0, 5.0001, 0.05)
    z = np.arange(0.0, 2.0, 0.05)
    gg, zz = (a.ravel() for a in np.meshgrid(g, z))
    one = np.ones_like(gg)
    walls = np.concatenate([np.column_stack([s * 5.0 * one, gg, zz]) for s in (1, -1)]
                           + [np.column_stack([gg, s * 5.0 * one, zz]) for s in (1, -1)])
    f = np.arange(-5.0, 5.0001, 0.1)
    fx, fy = np.meshgrid(f, f)
    floor = np.column_stack([fx.ravel(), fy.ravel(), np.zeros(fx.size)])
    bx, by, bz = np.meshgrid(np.arange(BOX[0], BOX[1] + 1e-9, 0.05), np.arange(BOX[2], BOX[3] + 1e-9, 0.05),
                             np.arange(0.0, 0.5 + 1e-9, 0.05))
    box = np.column_stack([bx.ravel(), by.ravel(), bz.ravel()])
    return np.concatenate([walls, floor, box])


def _person(x=-3.0, y=-3.0):
    '''A column of points that is only there for two frames.'''
    gx, gy, gz = np.meshgrid(np.arange(-0.1, 0.1001, 0.05) + x, np.arange(-0.1, 0.1001, 0.05) + y,
                             np.arange(0.0, 0.6, 0.05))
    return np.column_stack([gx.ravel(), gy.ravel(), gz.ravel()])


def _poses(n=72, radius=2.5):
    a = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    pos = np.column_stack([radius * np.cos(a), radius * np.sin(a), np.full(n, SENSOR_HEIGHT)])
    rot = Rotation.from_rotvec(np.column_stack([np.zeros(n), np.zeros(n), a + np.pi / 2])).as_matrix()
    return 1.7e9 + 0.1 * np.arange(n), rot, pos


def _frames(rot, pos, transient=None):
    '''The room as seen from every pose, in the sensor frame, with a little noise.'''
    world = _world()
    rng = np.random.default_rng(0)
    out = []
    for k in range(len(pos)):
        pts = world if transient is None or k not in (10, 11) else np.concatenate([world, transient])
        body = (pts - pos[k]) @ rot[k]
        out.append(body + rng.normal(0.0, 0.005, body.shape))
    return out


def _write_tum(path, stamps, rot, pos):
    np.savetxt(path, np.column_stack([stamps, pos, Rotation.from_matrix(rot).as_quat()]), fmt='%.9f')


def _write_dir(directory, stamps, frames, ext):
    os.makedirs(directory)
    for t, pts in zip(stamps, frames):
        path = os.path.join(directory, '%.6f%s' % (t, ext))
        if ext == '.npy':
            np.save(path, pts)
            continue
        header = '\n'.join(['# .PCD v0.7', 'VERSION 0.7', 'FIELDS x y z', 'SIZE 4 4 4', 'TYPE F F F',
                            'COUNT 1 1 1', 'WIDTH %d' % len(pts), 'HEIGHT 1', 'VIEWPOINT 0 0 0 1 0 0 0',
                            'POINTS %d' % len(pts), 'DATA ascii']) + '\n'
        with open(path, 'w', newline='\n') as f:
            f.write(header)
            np.savetxt(f, pts, fmt='%.5f')


def _read_grid(path):
    '''Grid with row 0 = lowest y, like the arrays inside the module.'''
    return np.flipud(np.array(Image.open(path)))


def _patch(grid, stats, x, y, r=0.1):
    res = stats['params']['res']
    ox, oy = stats['origin'][:2]
    x0, x1 = int((x - r - ox) / res), int((x + r - ox) / res) + 1
    y0, y1 = int((y - r - oy) / res), int((y + r - oy) / res) + 1
    return grid[max(y0, 0):max(y1, 0), max(x0, 0):max(x1, 0)]


@pytest.fixture(scope='module')
def room(tmp_path_factory):
    root = tmp_path_factory.mktemp('room')
    stamps, rot, pos = _poses()
    frames = _frames(rot, pos, _person())
    tum = str(root / 'traj.tum')
    _write_tum(tum, stamps, rot, pos)
    prefix = str(root / 'grid')
    stats = sc.build_grid(sc.ScanSet.from_arrays(frames, stamps), tum, prefix, FAST, log=quiet)
    return SimpleNamespace(stamps=stamps, frames=frames, tum=tum, stats=stats, prefix=prefix,
                           grid=_read_grid(prefix + '.pgm'))


def test_walls_and_box_are_occupied(room):
    for x, y in [(5.0, 0.0), (-5.0, 0.0), (0.0, 5.0), (0.0, -5.0), (1.25, -0.75)]:
        assert (_patch(room.grid, room.stats, x, y) == OCC).any(), (x, y)


def test_open_floor_and_driven_cells_are_free(room):
    for x, y in [(0.0, 0.0), (-2.0, 2.0), (2.5, 0.0)]:
        patch = _patch(room.grid, room.stats, x, y)
        assert not (patch == OCC).any() and (patch == FREE).mean() > 0.6, (x, y)


def test_object_seen_in_two_frames_is_not_occupied(room):
    assert not (_patch(room.grid, room.stats, -3.0, -3.0, r=0.3) == OCC).any()


def test_grid_is_cropped_to_the_room(room):
    size = room.stats['width'] * 0.05, room.stats['height'] * 0.05
    assert all(10.0 <= s <= 11.5 for s in size), size


def test_sensor_height_is_estimated(room):
    assert room.stats['sensor_height_source'] == 'estimated'
    assert abs(room.stats['params']['sensor_height'] - SENSOR_HEIGHT) < 0.03


def test_given_sensor_height_is_used_and_poses_are_matched_by_stamp(room, tmp_path):
    sub = sc.ScanSet.from_arrays(room.frames[::4], room.stamps[::4])   # 18 frames, the trajectory has 72 poses
    stats = sc.build_grid(sub, room.tum, str(tmp_path / 'g'), dict(FAST, sensor_height=0.5), log=quiet)
    assert stats['sensor_height_source'] == 'given'
    assert stats['params']['sensor_height'] == 0.5
    assert stats['frames_used'] == 18
    assert (_patch(_read_grid(str(tmp_path / 'g.pgm')), stats, 5.0, 0.0) == OCC).any()


def test_cli_reads_a_directory_and_matches_the_in_memory_grid(room, tmp_path):
    _write_dir(str(tmp_path / 'scans'), room.stamps, room.frames, '.npy')
    prefix = str(tmp_path / 'out' / 'grid')
    assert sc.main([str(tmp_path / 'scans'), room.tum, prefix] + FAST_ARGS) == 0
    for suffix in ('.pgm', '.yaml', '_preview.png'):
        assert os.path.exists(prefix + suffix), suffix
    assert np.array_equal(_read_grid(prefix + '.pgm'), room.grid)
    with open(prefix + '.yaml') as f:
        text = f.read()
    assert 'mode: trinary' in text and 'free_thresh: 0.15' in text


def test_ascii_pcd_directory(room, tmp_path):
    _write_dir(str(tmp_path / 'scans'), room.stamps[::6], room.frames[::6], '.pcd')
    stats = sc.build_grid(str(tmp_path / 'scans'), room.tum, str(tmp_path / 'g'), FAST, log=quiet)
    assert stats['frames_used'] == 12
    grid = _read_grid(str(tmp_path / 'g.pgm'))
    for x, y in [(5.0, 0.0), (-5.0, 0.0), (1.25, -0.75)]:
        assert (_patch(grid, stats, x, y) == OCC).any(), (x, y)


def test_cli_reports_errors(tmp_path, capsys):
    (tmp_path / 'empty').mkdir()
    (tmp_path / 't.tum').write_text('0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n')
    assert sc.main([str(tmp_path / 'empty'), str(tmp_path / 't.tum'), str(tmp_path / 'g')]) == 1
    assert 'no point cloud files' in capsys.readouterr().err


def _dummy(n, stamps=None):
    return sc.ScanSet.from_arrays([np.zeros((1, 3))] * n, stamps)


def test_match_by_stamp_drops_frames_without_a_pose():
    scans = _dummy(5, [0.0, 1.0, 2.0, 3.02, 10.0])
    frame, pose = sc.match_poses(scans, np.array([0.0, 1.0, 2.0, 3.0]), 'auto', 0.05)
    assert list(frame) == [0, 1, 2, 3]
    assert list(pose) == [0, 1, 2, 3]


def test_auto_falls_back_to_order_when_the_names_are_indices():
    scans = _dummy(4, [0.0, 1.0, 2.0, 3.0])   # 000001.pcd style names do not fit the trajectory stamps
    frame, pose = sc.match_poses(scans, 1.7e9 + np.arange(4.0))
    assert list(frame) == [0, 1, 2, 3] and list(pose) == [0, 1, 2, 3]


def test_match_order_needs_equal_counts():
    with pytest.raises(ValueError, match='frames but'):
        sc.match_poses(_dummy(3), np.arange(4.0), 'order')


def test_match_stamp_needs_timestamps_that_fit():
    with pytest.raises(ValueError, match='timestamps'):
        sc.match_poses(_dummy(3), np.arange(3.0), 'stamp')
    with pytest.raises(ValueError, match='within'):
        sc.match_poses(_dummy(3, [100.0, 101.0, 102.0]), np.arange(3.0), 'stamp')


def test_directory_names_sort_naturally_without_stamps(tmp_path):
    for name in ('scan_10.npy', 'scan_2.npy', 'scan_1.npy'):
        np.save(str(tmp_path / name), np.full((1, 3), float(name.split('_')[1].split('.')[0])))
    scans = sc.scans_from_directory(str(tmp_path))
    assert scans.stamps is None
    assert [float(scans.points(k)[0, 0]) for k in range(3)] == [1.0, 2.0, 10.0]


def test_read_tum_sorts_and_converts(tmp_path):
    path = tmp_path / 't.tum'
    path.write_text('# comment\n2.0 1 2 3 0 0 0 1\n1.0 0 0 0 0 0 0.7071068 0.7071068\n')
    t, rot, pos = sc.read_tum(str(path))
    assert list(t) == [1.0, 2.0]
    assert np.allclose(rot[0], Rotation.from_euler('z', 90, degrees=True).as_matrix(), atol=1e-6)
    assert list(pos[1]) == [1.0, 2.0, 3.0]


def test_read_tum_rejects_bad_files(tmp_path):
    short = tmp_path / 'short.tum'
    short.write_text('0 0 0 0\n1 1 1 1\n')
    with pytest.raises(ValueError, match='TUM'):
        sc.read_tum(str(short))
    zero = tmp_path / 'zero.tum'
    zero.write_text('0 0 0 0 0 0 0 0\n1 1 0 0 0 0 0 1\n')
    with pytest.raises(ValueError, match='quaternion'):
        sc.read_tum(str(zero))


@pytest.mark.parametrize('bad', [dict(res=0), dict(obs_min=0.8), dict(scan_stride=0), dict(ray_deg=0),
                                 dict(occ_ratio=2), dict(unknown_key=1)])
def test_bad_params_are_rejected(bad):
    with pytest.raises(ValueError):
        sc.check_params(dict(sc.DEFAULTS, **bad))
