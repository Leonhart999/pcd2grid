"""连续地面、后处理、连通性和保守射线的回归测试。"""
import json

import numpy as np
import pytest

from pcd2grid.cli import main
from pcd2grid.core import FREE, OCC, UNKNOWN, GridParams, convert, height_above_ground
from pcd2grid.processing import connectivity, ground_heightmap, raycast_free, read_trajectory, remove_small


def test_heightmap_curved_floor_and_obstacles():
    x, y = np.meshgrid(np.arange(0, 12, 0.1), np.arange(0, 6, 0.1))
    z = 0.012 * x ** 2
    floor = np.c_[x.ravel(), y.ravel(), z.ravel()]
    wall = floor[(floor[:, 0] > 5) & (floor[:, 0] < 5.3)].copy()
    wall[:, 2] += 0.4
    heights, coverage = ground_heightmap(np.vstack([floor, wall]), cell=0.5, smooth=0.25)
    assert np.percentile(np.abs(heights[:len(floor)]), 95) < 0.05
    assert np.median(heights[len(floor):]) > 0.3
    assert coverage > 0.9


def test_heightmap_sparse_holes_and_allocation_limit():
    xyz = np.array([[0, 0, 0], [0, 2, 0], [2, 0, 0], [2, 2, 0]], dtype=float)
    heights, _ = ground_heightmap(xyz)
    assert np.allclose(heights, 0)
    with pytest.raises(ValueError, match="max-cells"):
        ground_heightmap(xyz, cell=0.01, max_cells=10)


def test_component_filter_four_connected():
    mask = np.zeros((5, 5), bool)
    mask[1, 1] = True
    mask[2:4, 2:4] = True
    filtered = remove_small(mask, 2)
    assert filtered.sum() == 4
    assert not filtered[1, 1]


def test_connectivity_start_and_clearance():
    grid = np.full((9, 9), FREE, dtype=np.uint8)
    grid[:, 4] = OCC
    report = connectivity(grid, 0.1, (0, 0), (0.15, 0.15))
    assert report["components"] == 2
    assert report["reachable_cells"] == 36
    assert report["disconnected_cells"] == 36
    assert connectivity(grid, 0.1, (0, 0), (-1, 0))["start_valid"] is False
    grid[4, 4] = FREE
    assert connectivity(grid, 0.1, (0, 0))["components"] == 1
    assert connectivity(grid, 0.1, (0, 0), robot_radius=0.11)["components"] == 2


def test_raycast_stops_at_wall_and_preserves_it():
    occupied = np.zeros((3, 12), bool)
    occupied[:, 5] = True
    free, rays = raycast_free(occupied, np.array([[10.5, 1.5, 0]]),
                             np.array([[1.5, 1.5, 0]]), (0, 0), 1)
    assert rays == 1
    assert free[1, 1:5].all()
    assert not free[:, 5:].any()
    assert occupied[:, 5].all()


def test_raycast_diagonal_corner_is_blocked():
    occupied = np.zeros((5, 5), bool)
    occupied[0, 1] = True
    free, _ = raycast_free(occupied, np.array([[4.5, 4.5, 0]]),
                           np.array([[0.5, 0.5, 0]]), (0, 0), 1)
    assert free[0, 0]
    assert not free[1, 1]


def test_raycast_range_and_budget():
    occupied = np.zeros((3, 12), bool)
    points = np.array([[8.5, 1.5, 0], [9.5, 1.5, 0]])
    trajectory = np.array([[1.5, 1.5, 0]])
    free, rays = raycast_free(occupied, points, trajectory, (0, 0), 1, max_range=2)
    assert rays == 0 and not free.any()
    _, rays = raycast_free(occupied, points, trajectory, (0, 0), 1, max_rays=1)
    assert rays == 1


def test_trajectory_parse(tmp_path):
    path = tmp_path / "estimated.tum"
    path.write_text("0 1 2 3 0 0 0 1\n1 2 3 4 0 0 0 1\n")
    assert np.array_equal(read_trajectory(path), [[1, 2, 3], [2, 3, 4]])
    path.write_text("0 1 2\n")
    with pytest.raises(ValueError):
        read_trajectory(path)


@pytest.mark.parametrize("kwargs", [{"res": 0}, {"tile": -1}, {"ground_cell": np.nan},
                                    {"max_rays": 0}, {"robot_radius": -1}, {"obs_max": np.inf}])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        convert(np.zeros((20, 3)), GridParams(**kwargs))


def test_grid_sorted_fit_matches_legacy():
    from pcd2grid.core import estimate_ground, fit_ground_plane
    rng = np.random.default_rng(42)
    xyz = rng.uniform([0, 0, 0], [20, 20, 1], (12000, 3))
    xyz[:9000, 2] = 0.02 * xyz[:9000, 0]
    a, b, c = fit_ground_plane(xyz, estimate_ground(xyz[:, 2]))
    expected = xyz[:, 2] - (a * xyz[:, 0] + b * xyz[:, 1] + c)
    tx = np.floor((xyz[:, 0] - xyz[:, 0].min()) / 5).astype(np.int64)
    ty = np.floor((xyz[:, 1] - xyz[:, 1].min()) / 5).astype(np.int64)
    key = tx * (ty.max() + 1) + ty
    for k in np.unique(key):
        sel = key == k
        pts = xyz[sel]
        lo = np.percentile(pts[:, 2], 5)
        near = pts[np.abs(pts[:, 2] - (lo + 0.1)) < 0.25]
        if len(near) >= 30:
            la, lb, lc = fit_ground_plane(pts, estimate_ground(near[:, 2]))
            expected[sel] = pts[:, 2] - (la * pts[:, 0] + lb * pts[:, 1] + lc)
    actual, _, _ = height_above_ground(xyz, "grid", 5)
    assert np.array_equal(actual, expected)


def test_cli_new_options_report(tmp_path):
    x, y = np.meshgrid(np.arange(0, 3, 0.1), np.arange(0, 3, 0.1))
    xyz = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    src = tmp_path / "cloud.npy"
    np.save(src, xyz)
    tum = tmp_path / "poses.tum"
    tum.write_text("0 1 1 0.3 0 0 0 1\n")
    report = tmp_path / "report.json"
    assert main([str(src), str(tmp_path / "map"), "--ground-mode", "heightmap",
                 "--trajectory", str(tum), "--start", "1,1", "--report", str(report),
                 "--min-obstacle-area", "0.01", "--min-free-area", "0.01"]) == 0
    data = json.loads(report.read_text(encoding="utf-8"))
    assert data["stats"]["rays"] > 0
    assert data["stats"]["connectivity"]["start_valid"]


def test_removed_obstacle_becomes_unknown_not_free():
    x, y = np.meshgrid(np.arange(0.05, 3, 0.1), np.arange(0.05, 3, 0.1))
    floor = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    cloud = np.vstack([floor, [[1.55, 1.55, 0.4]] * 3])
    result = convert(cloud, GridParams(res=0.1, ground_mode="none", min_obstacle_area=0.02))
    ix, iy = np.floor((np.array([1.55, 1.55]) - result.origin) / result.res).astype(int)
    assert result.grid[iy, ix] == UNKNOWN
