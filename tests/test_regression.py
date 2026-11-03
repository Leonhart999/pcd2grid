"""Extra tests: LZF/binary_compressed PCD, up-axis rotation, grid ground mode, map orientation."""
import numpy as np
import pytest

from pcd2grid.core import FREE, OCC, GridParams, convert, orient
from pcd2grid.io import read_pcd


def lzf_literal_compress(data):
    """Encode data as LZF literal runs only (valid LZF, no back references)."""
    out = bytearray()
    for i in range(0, len(data), 32):
        chunk = data[i:i + 32]
        out.append(len(chunk) - 1)
        out += chunk
    return bytes(out)


def test_binary_compressed_pcd(tmp_path):
    xyz = np.random.default_rng(3).uniform(-5, 5, (200, 3)).astype(np.float32)
    # binary_compressed stores columns one after another
    raw = xyz[:, 0].tobytes() + xyz[:, 1].tobytes() + xyz[:, 2].tobytes()
    comp = lzf_literal_compress(raw)
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              "WIDTH 200\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 200\nDATA binary_compressed\n")
    p = tmp_path / "c.pcd"
    with open(p, "wb") as f:
        f.write(header.encode())
        f.write(len(comp).to_bytes(4, "little"))
        f.write(len(raw).to_bytes(4, "little"))
        f.write(comp)
    assert np.allclose(read_pcd(str(p)), xyz, atol=1e-6)


def test_up_axis_rotations_map_axis_to_z():
    pts = np.array([[0.0, 0.0, 1.0]])
    # a point at +1 on the declared up axis must end up at z = +1
    for axis, vec in {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1],
                      "-x": [-1, 0, 0], "-y": [0, -1, 0], "-z": [0, 0, -1]}.items():
        out = orient(np.array([vec], dtype=float), axis)
        assert np.isclose(out[0, 2], 1.0), axis


def tripod_room(seed=5):
    """Room scanned from 1.3 m height: the ceiling has three times as many points as the floor."""
    rng = np.random.default_rng(seed)
    fx, fy = rng.uniform(0, 8, 20000), rng.uniform(0, 6, 20000)
    cx, cy = rng.uniform(0, 8, 60000), rng.uniform(0, 6, 60000)
    t = rng.uniform(0, 8, 6000)
    walls = np.c_[t, np.zeros_like(t), rng.uniform(-1.3, 1.6, 6000)]
    return np.vstack([np.c_[fx, fy, np.full_like(fx, -1.3)], np.c_[cx, cy, np.full_like(cx, 1.6)], walls])


def test_ground_is_floor_not_denser_ceiling():
    r = convert(tripod_room(), GridParams(res=0.1))
    ground = float(r.info.split("height ")[1].split(" m")[0])
    assert abs(ground + 1.3) < 0.02
    ix = int((4.0 - r.origin[0]) / r.res)
    assert r.grid[int((3.0 - r.origin[1]) / r.res), ix] == FREE
    assert r.grid[int((0.0 - r.origin[1]) / r.res), ix] == OCC
    assert not r.warnings


def test_level_fixes_tilted_cloud():
    xyz = tripod_room()
    a = np.radians(25.0)
    rot = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    tilted = xyz @ rot.T
    plain = convert(tilted, GridParams(res=0.1))
    assert any("--level" in w for w in plain.warnings)
    levelled = convert(tilted, GridParams(res=0.1, level=True))
    assert "tilted 25.0" in levelled.info
    assert np.allclose(levelled.level_rotation @ rot, np.eye(3), atol=0.01)
    ground_tilt = float(levelled.info.split("ground plane tilt ")[1].split(" deg")[0])
    assert ground_tilt < 0.5


def test_huge_extent_is_refused_before_allocating():
    xyz = np.array([[0.0, 0.0, 0.0]] * 20 + [[5000.0, 5000.0, 0.0]] * 20)
    with pytest.raises(ValueError, match="above the limit"):
        convert(xyz, GridParams(res=0.05))


def test_grid_mode_handles_two_slopes():
    rng = np.random.default_rng(4)
    x = rng.uniform(0, 20, 80000)
    y = rng.uniform(0, 10, 80000)
    z = np.where(x < 10, 0.0, 0.05 * (x - 10))   # flat half, then a 3 degree ramp
    ground = np.c_[x, y, z]
    wx = rng.uniform(0, 20, 6000)
    wall = np.c_[wx, np.zeros_like(wx), rng.uniform(0.2, 1.5, 6000) + np.where(wx < 10, 0, 0.05 * (wx - 10))]
    r = convert(np.vstack([ground, wall]), GridParams(res=0.1, ground_mode="grid", tile=5.0))
    ix = int((15.0 - r.origin[0]) / r.res)
    iy = int((5.0 - r.origin[1]) / r.res)
    assert r.grid[iy, ix] == FREE
    assert (r.grid == OCC).sum() > 0
