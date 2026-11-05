"""地面高度图、连通性检查及保守轨迹射线补全。"""
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


def ground_heightmap(xyz, cell=0.5, smooth=1.0, max_cells=100_000_000):
    """最低点栅格经过中值去噪、最近邻补洞和平滑，再双线性采样。

    这是单层地面的下包络估计，不适用于上下重叠楼层；没有地面观测的
    大面积桌面仍可能被误认，应裁切楼层并检查分类图。
    """
    origin = xyz[:, :2].min(axis=0)
    ij = np.floor((xyz[:, :2] - origin) / cell).astype(np.int64)
    w, h = ij.max(axis=0) + 1
    if int(w) * int(h) > max_cells:
        raise ValueError("地面高度图超过 --max-cells；请增大 --ground-cell 或裁切范围")
    floor = np.full((h, w), np.inf)
    np.minimum.at(floor, (ij[:, 1], ij[:, 0]), xyz[:, 2])
    valid = np.isfinite(floor)
    nearest = ndimage.distance_transform_edt(~valid, return_distances=False, return_indices=True)
    floor = floor[tuple(nearest)]
    # 最低点通常不在格心；用局部坡度补偿到格心，避免斜坡系统性偏低。
    key = ij[:, 1] * w + ij[:, 0]
    order = np.lexsort((xyz[:, 2], key))
    first = order[np.r_[True, key[order][1:] != key[order][:-1]]]
    slope_y = np.gradient(floor, cell, axis=0) if h > 1 else np.zeros_like(floor)
    slope_x = np.gradient(floor, cell, axis=1) if w > 1 else np.zeros_like(floor)
    px, py = ij[first, 0], ij[first, 1]
    centers = origin + (ij[first] + 0.5) * cell
    delta = centers - xyz[first, :2]
    floor[py, px] += slope_x[py, px] * delta[:, 0] + slope_y[py, px] * delta[:, 1]
    floor = floor[tuple(nearest)]
    floor = ndimage.median_filter(floor, size=3, mode="nearest")
    if smooth > 0:
        floor = ndimage.gaussian_filter(floor, sigma=smooth / cell, mode="nearest")
    coords = ((xyz[:, :2] - origin) / cell - 0.5).T[::-1]
    sampled = ndimage.map_coordinates(floor, coords, order=1, mode="nearest")
    return xyz[:, 2] - sampled, float(valid.mean())


def remove_small(mask, min_cells):
    """四连通分量面积过滤；不把删除的障碍直接改成自由空间。"""
    if min_cells <= 1:
        return mask
    labels, _ = ndimage.label(mask)
    sizes = np.bincount(labels.ravel())
    keep = sizes >= min_cells
    keep[0] = False
    return keep[labels]


def connectivity(grid, res, origin, start=None, robot_radius=0.0):
    """未知和障碍均阻塞；不修改地图，返回从指定起点的四连通统计。"""
    free = grid == 254
    if robot_radius > 0:
        distance = ndimage.distance_transform_edt(np.pad(free, 1), sampling=res)[1:-1, 1:-1]
        free &= distance > robot_radius
    labels, n = ndimage.label(free)
    sizes = np.bincount(labels.ravel(), minlength=n + 1)
    result = {"components": int(n), "traversable_cells": int(free.sum()),
              "largest_component_cells": int(sizes[1:].max()) if n else 0,
              "robot_radius": float(robot_radius)}
    if start is not None:
        ix, iy = np.floor((np.asarray(start) - origin) / res).astype(int)
        label = int(labels[iy, ix]) if 0 <= iy < grid.shape[0] and 0 <= ix < grid.shape[1] else 0
        reachable = int(sizes[label]) if label else 0
        result.update(start_valid=bool(label), reachable_cells=reachable,
                      disconnected_cells=int(free.sum()) - reachable,
                      reachable_fraction=reachable / int(free.sum()) if free.any() else None)
    return result


def read_trajectory(path):
    """读取 TUM 的 timestamp x y z qx qy qz qw，只使用位置。"""
    data = np.loadtxt(path, ndmin=2)
    if data.shape[1] != 8 or not len(data) or not np.isfinite(data).all():
        raise ValueError("轨迹必须是非空、有限值的八列 TUM 文件")
    if np.any(np.diff(data[:, 0]) < 0):
        raise ValueError("TUM 时间戳必须单调不减")
    return data[:, 1:4]


def _line(x0, y0, x1, y1):
    """整数 Bresenham，包含首尾；逐格检查当前障碍。"""
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * err
        if twice >= dy:
            err += dy
            x0 += sx
        if twice <= dx:
            err += dx
            y0 += sy


def raycast_free(occupied, endpoints, trajectory, origin, res, max_range=10.0, max_rays=200_000):
    """聚合点云缺乏逐帧对应：最近轨迹位置仅为近似，遇障碍立即终止。

    不清除任何已有障碍。每个端点栅格只选一个代表点，并限制射线数量。
    """
    free = np.zeros_like(occupied)
    if not len(endpoints) or not len(trajectory):
        return free, 0
    cells = np.floor((endpoints[:, :2] - origin) / res).astype(np.int64)
    _, first = np.unique(cells, axis=0, return_index=True)
    if len(first) > max_rays:
        first = first[np.linspace(0, len(first) - 1, max_rays, dtype=int)]
    endpoints, cells = endpoints[first], cells[first]
    distances, poses = cKDTree(trajectory).query(endpoints)
    starts = np.floor((trajectory[poses, :2] - origin) / res).astype(np.int64)
    height, width = occupied.shape
    rays = 0
    for (x0, y0), (x1, y1), distance in zip(starts, cells, distances):
        if distance > max_range or not (0 <= x0 < width and 0 <= y0 < height):
            continue
        if occupied[y0, x0]:
            continue
        rays += 1
        previous = None
        for x, y in _line(int(x0), int(y0), int(x1), int(y1)):
            if not (0 <= x < width and 0 <= y < height) or occupied[y, x]:
                break
            # 对角步不能穿过两个障碍格之间的角缝。
            if previous is not None:
                px, py = previous
                if x != px and y != py and (occupied[py, x] or occupied[y, px]):
                    break
            free[y, x] = True
            previous = (x, y)
    return free, rays
