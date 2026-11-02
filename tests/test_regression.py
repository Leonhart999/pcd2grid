"""Extra tests: LZF/binary_compressed PCD, up-axis rotation, grid ground mode, map orientation."""
import numpy as np

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
