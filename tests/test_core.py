"""Synthetic-cloud tests for pcd2grid: tilted ground, walls, ceilings, orientation, I/O."""
import numpy as np
import pytest
from PIL import Image

from pcd2grid.cli import main
from pcd2grid.core import FREE, OCC, UNKNOWN, GridParams, convert, orient
from pcd2grid.io import read_pcd, read_ply, read_points
from pcd2grid.output import write_map


def make_room(tilt_deg=0.0, size=10.0, seed=0):
    """A size x size room: tilted ground, four walls 2 m high, and a ceiling at 2.5 m."""
    rng = np.random.default_rng(seed)
    slope = np.tan(np.radians(tilt_deg))
    gx = rng.uniform(0, size, 60000)
    gy = rng.uniform(0, size, 60000)
    ground = np.c_[gx, gy, slope * gx]
    pts = [ground]
    t = rng.uniform(0, size, 8000)
    z = rng.uniform(0.0, 2.0, 8000)
    for wall in (np.c_[t, np.zeros_like(t), z], np.c_[t, np.full_like(t, size), z],
                 np.c_[np.zeros_like(t), t, z], np.c_[np.full_like(t, size), t, z]):
        wall[:, 2] += slope * wall[:, 0]      # walls stand on the tilted ground
        pts.append(wall)
    cx = rng.uniform(0, size, 20000)
    cy = rng.uniform(0, size, 20000)
    ceiling = np.c_[cx, cy, 2.5 + slope * cx]
    pts.append(ceiling)
    return np.vstack(pts)


def cell(result, x, y):
    ix = int((x - result.origin[0]) / result.res)
    iy = int((y - result.origin[1]) / result.res)
    return result.grid[iy, ix]


def test_flat_room_walls_free_interior_ceiling_ignored():
    r = convert(make_room(), GridParams(res=0.1))
    assert cell(r, 5.0, 5.0) == FREE            # ceiling above does not block the floor
    assert cell(r, 5.0, 0.0) == OCC             # wall
    assert cell(r, 10.0, 5.0) == OCC
    assert cell(r, -0.4, -0.4) in (UNKNOWN, FREE, OCC)


def test_tilted_ground_is_not_obstacle():
    # Rotate the flat room about the y axis by 1 degree so the far side is ~17 cm higher.
    xyz = make_room()
    a = np.radians(1.0)
    rot = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    xyz = xyz @ rot.T
    r = convert(xyz, GridParams(res=0.1))
    assert cell(r, 3.0, 5.0) == FREE
    assert cell(r, 9.0, 5.0) == FREE


def test_image_orientation_row0_is_max_y():
    r = convert(make_room(), GridParams(res=0.1))
    img = r.image
    # Wall along y = 10 must appear in the first image rows, wall along y = 0 in the last rows
    assert (img[:8] == OCC).sum() > (img[8:16] == OCC).sum()
    assert (img[-8:] == OCC).sum() > 0


def test_up_axis_y_matches_z():
    xyz = make_room()
    # Re-express the same room in a y-up frame: (x, y, z) -> (x, z, -y)
    y_up = np.c_[xyz[:, 0], xyz[:, 2], -xyz[:, 1]]
    assert np.allclose(orient(y_up, "y"), xyz)
    base = convert(xyz, GridParams(res=0.1))
    other = convert(y_up, GridParams(res=0.1, up_axis="y"))
    assert base.grid.shape == other.grid.shape
    assert (base.grid == other.grid).all()


def test_z_slab_and_ground_none():
    xyz = make_room()
    r = convert(xyz, GridParams(res=0.1, ground_mode="none", obs_min=0.2, obs_max=1.5, ground_tol=0.05))
    assert cell(r, 5.0, 0.0) == OCC
    r2 = convert(xyz, GridParams(res=0.1, z_slab=(-0.1, 2.0)))
    assert r2.stats["points"] < len(xyz)


def test_inflate_grows_occupied():
    xyz = make_room()
    a = convert(xyz, GridParams(res=0.1))
    b = convert(xyz, GridParams(res=0.1, inflate=0.3))
    assert b.stats["occupied"] > a.stats["occupied"]


def test_origin_override_and_voxel():
    xyz = make_room()
    r = convert(xyz, GridParams(res=0.1, origin=(-1.0, -1.0), voxel=0.05))
    assert r.origin == (-1.0, -1.0)


def test_too_few_points():
    with pytest.raises(ValueError):
        convert(np.zeros((3, 3)), GridParams())


def write_pcd(path, xyz, mode):
    n = len(xyz)
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
              f"COUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA {mode}\n")
    data = np.c_[xyz, np.ones(n)].astype(np.float32)
    with open(path, "wb") as f:
        f.write(header.encode())
        if mode == "binary":
            f.write(data.tobytes())
        else:
            np.savetxt(f, data, fmt="%.6f")


@pytest.mark.parametrize("mode", ["binary", "ascii"])
def test_pcd_roundtrip(tmp_path, mode):
    xyz = np.random.default_rng(1).uniform(-5, 5, (100, 3)).astype(np.float32)
    p = tmp_path / "a.pcd"
    write_pcd(p, xyz, mode)
    got = read_pcd(str(p))
    assert np.allclose(got, xyz, atol=1e-4)


def test_ply_and_npy(tmp_path):
    xyz = np.random.default_rng(2).uniform(-5, 5, (50, 3))
    p = tmp_path / "a.ply"
    with open(p, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\nelement vertex 50\n"
                 "property double x\nproperty double y\nproperty double z\nend_header\n").encode())
        f.write(xyz.astype("<f8").tobytes())
    assert np.allclose(read_ply(str(p)), xyz)
    np.save(tmp_path / "a.npy", xyz)
    assert np.allclose(read_points(str(tmp_path / "a.npy")), xyz)


@pytest.mark.parametrize("mode,ext", [("trinary", ".pgm"), ("scale", ".png"), ("raw", ".pgm")])
def test_map_modes_write_expected_image(tmp_path, mode, ext):
    r = convert(make_room(), GridParams(res=0.1))
    out = tmp_path / "m"
    files = write_map(r, str(out), mode=mode, preview=False)
    assert files[0].endswith("m" + ext)
    yaml_text = (tmp_path / "m.yaml").read_text()
    assert f"mode: {mode}" in yaml_text and f"image: m{ext}" in yaml_text
    assert "free_thresh: 0.15" in yaml_text
    img = np.array(Image.open(files[0]))
    if mode == "scale":
        # unknown cells must be fully transparent so map_server reads them as -1
        assert img.shape[2] == 4 and (img[..., 3] == 0).any()
    elif mode == "raw":
        assert set(np.unique(img)) <= {0, 100, 255}
    else:
        assert set(np.unique(img)) <= {OCC, FREE, UNKNOWN}


def test_cli_and_yaml_output(tmp_path):
    src = tmp_path / "room.npy"
    np.save(src, make_room())
    out = tmp_path / "out" / "room"
    assert main([str(src), str(out), "--res", "0.1", "--npy"]) == 0
    assert (tmp_path / "out" / "room.pgm").exists()
    yaml_text = (tmp_path / "out" / "room.yaml").read_text()
    assert "resolution: 0.1" in yaml_text and "mode: trinary" in yaml_text
    assert np.array(Image.open(str(out) + ".pgm")).dtype == np.uint8
    assert main([str(tmp_path / "missing.pcd"), str(out)]) == 1
