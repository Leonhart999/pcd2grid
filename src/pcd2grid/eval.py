"""ROS 栅格地图评估；坐标单位为米，角度单位为弧度。

共同已知指标在预测格心上统计；全真值指标在真值格心上反向采样预测。
两者均为最近格采样，不做插值或多格投票。全真值范围排除真值未知格，
但预测未知和预测地图外均算漏检。概率栅格以 occupancy_threshold 二分类。
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
import yaml


@dataclass
class MapGrid:
    """values[y, x] 为 -1（未知）或 0..100；第零行位于地图下方。"""

    values: np.ndarray
    resolution: float
    origin: tuple = (0.0, 0.0, 0.0)

    def __post_init__(self):
        self.values = np.asarray(self.values)
        if self.values.ndim != 2 or not self.values.size:
            raise ValueError("地图必须是非空二维数组")
        if not np.isfinite(self.values).all() or not np.all(
            (self.values == -1) | ((self.values >= 0) & (self.values <= 100))
        ):
            raise ValueError("地图值必须为 -1 或 0..100")
        self.resolution = _positive(self.resolution, "resolution")
        self.origin = tuple(float(v) for v in self.origin)
        if len(self.origin) != 3 or not np.isfinite(self.origin).all():
            raise ValueError("origin 必须包含三个有限数值")


def _positive(value, name):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} 必须为有限正值")
    return value


def validate_transform(matrix=None):
    """仅接受绕 z 轴旋转和平移；z 平移允许，但不得与 xy 混合。"""
    matrix = np.eye(4) if matrix is None else np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("transform 必须是有限数值组成的 4x4 矩阵")
    rotation = matrix[:3, :3]
    if not (np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
            and np.isclose(np.linalg.det(rotation), 1, atol=1e-8, rtol=0)
            and np.allclose(rotation[2], [0, 0, 1], atol=1e-8, rtol=0)
            and np.allclose(rotation[:2, 2], 0, atol=1e-8, rtol=0)):
        raise ValueError("transform 必须为平面刚体变换，禁止缩放、反射及 z/xy 混合")
    return matrix.copy()


def load_map(path):
    """按 Nav2 map_server 语义读取 ROS YAML 和图像。

    scale 中间概率在阈值区间线性映射到 0..100 并取最近整数。
    raw 像素 0..100 是概率，101..255 是未知，忽略 negate。
    trinary 的有透明通道图像将 alpha 纳入颜色均值；scale 非全不透明为未知。
    """
    path = Path(path)
    with path.open(encoding="utf-8") as stream:
        meta = yaml.safe_load(stream)
    if not isinstance(meta, dict):
        raise ValueError("地图 YAML 必须是映射")
    mode = meta.get("mode", "trinary")
    if mode not in ("trinary", "scale", "raw"):
        raise ValueError("地图 mode 必须为 trinary、scale 或 raw")
    resolution = _positive(meta["resolution"], "resolution")
    origin = meta["origin"]
    free = float(meta.get("free_thresh", 0.196))
    occupied = float(meta.get("occupied_thresh", 0.65))
    if not (np.isfinite([free, occupied]).all() and 0 <= free < occupied <= 1):
        raise ValueError("阈值必须满足 0 <= free_thresh < occupied_thresh <= 1")
    negate = meta.get("negate", 0)
    if negate not in (0, 1):
        raise ValueError("negate 必须为 0 或 1")
    with Image.open(path.parent / meta["image"]) as image:
        has_alpha = "A" in image.getbands() or "transparency" in image.info
        rgba = np.asarray(image.convert("RGBA"))
    gray = rgba[..., :3].mean(axis=2)
    if mode == "trinary" and has_alpha:
        gray = (rgba[..., :3].astype(float).sum(axis=2) + rgba[..., 3]) / 4
    values = np.full(gray.shape, -1, dtype=np.int16)
    if mode == "raw":
        raw = np.floor(gray + 0.5)
        valid = raw <= 100
        values[valid] = raw[valid].astype(np.int16)
    else:
        probability = gray / 255 if negate else (255 - gray) / 255
        values[probability > occupied] = 100
        values[probability < free] = 0
        if mode == "scale":
            middle = (probability >= free) & (probability <= occupied)
            values[middle] = np.rint(100 * (probability[middle] - free)
                                     / (occupied - free)).astype(np.int16)
            values[rgba[..., 3] != 255] = -1
    return MapGrid(np.flipud(values).copy(), resolution, origin)


def _world(grid, x, y):
    x = (x + 0.5) * grid.resolution
    y = (y + 0.5) * grid.resolution
    ox, oy, yaw = grid.origin
    c, s = np.cos(yaw), np.sin(yaw)
    return ox + c * x - s * y, oy + s * x + c * y


def _sample(grid, x, y, values=None):
    ox, oy, yaw = grid.origin
    dx, dy = np.asarray(x) - ox, np.asarray(y) - oy
    c, s = np.cos(yaw), np.sin(yaw)
    # 先判断浮点索引范围，再转整数，避免极大地图外坐标溢出。
    ix = np.floor((c * dx + s * dy) / grid.resolution)
    iy = np.floor((-s * dx + c * dy) / grid.resolution)
    inside = ((ix >= 0) & (iy >= 0) & (ix < grid.values.shape[1])
              & (iy < grid.values.shape[0]))
    values = grid.values if values is None else values
    result = np.full(np.shape(ix), -1, dtype=np.result_type(values.dtype, np.int16))
    result[inside] = values[iy[inside].astype(np.intp), ix[inside].astype(np.intp)]
    return result


def _pairs(source, target, transform, chunk_size):
    """只创建 chunk_size 大小的坐标数组，不创建整幅地图的坐标网格。"""
    width = source.values.shape[1]
    for start in range(0, source.values.size, chunk_size):
        index = np.arange(start, min(start + chunk_size, source.values.size))
        x, y = _world(source, index % width, index // width)
        tx = transform[0, 0] * x + transform[0, 1] * y + transform[0, 3]
        ty = transform[1, 0] * x + transform[1, 1] * y + transform[1, 3]
        yield source.values.flat[start:start + len(index)], _sample(target, tx, ty)


def _ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def _metrics(counts):
    tp, predicted, actual, free_hits, free_total, known, total = map(int, counts)
    union = predicted + actual - tp
    return {
        "occupied_precision": _ratio(tp, predicted),
        "occupied_recall": _ratio(tp, actual),
        "occupied_iou": _ratio(tp, union),
        "free_coverage": _ratio(free_hits, free_total),
        "known_coverage": _ratio(known, total),
        "counts": {"occupied_true_positive": tp, "predicted_occupied": predicted,
                   "ground_truth_occupied": actual, "occupied_union": union,
                   "free_true_positive": free_hits, "ground_truth_free": free_total,
                   "predicted_known": known, "evaluated_cells": total},
    }


def _accumulate(counts, prediction, truth, common_only, threshold):
    valid = truth >= 0
    if common_only:
        valid &= prediction >= 0
    p, t = prediction[valid], truth[valid]
    po, to = p >= threshold, t >= threshold
    pf, tf = (p >= 0) & (p < threshold), t < threshold
    counts += [np.count_nonzero(po & to), np.count_nonzero(po), np.count_nonzero(to),
               np.count_nonzero(pf & tf), np.count_nonzero(tf),
               np.count_nonzero(p >= 0), len(t)]


def _components(grid, radius, threshold):
    free = (grid.values >= 0) & (grid.values < threshold)
    if radius:
        # 补一圈障碍，把地图边界外纳入真实米制欧氏距离变换。
        distance = distance_transform_edt(np.pad(free, 1), sampling=grid.resolution)[1:-1, 1:-1]
        free &= distance > radius
    return label(free, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]]))[0]


def _connected(grid, components, start, end):
    # 标签 0 是障碍，标签值保持原整数精度，避免大量连通域溢出。
    ids = _sample(grid, np.array([start[0], end[0]]),
                  np.array([start[1], end[1]]), values=components)
    return bool(ids[0] > 0 and ids[0] == ids[1])


def evaluate_maps(prediction, ground_truth, transform=None, *, chunk_size=262144,
                  doors=None, robot_radius=0.0, occupancy_threshold=50.0):
    """transform 从预测世界系映射至真值/BIM 世界系；doors 端点处于后者。

    precision=TP/预测占据，recall=TP/真值占据，IoU=TP/占据并集，
    free_coverage=正确自由/真值自由；无有效分母返回 None。
    radius 使用障碍格心的米制欧氏距离，距离 <= 半径即不可通行；
    地图外第一圈格心同样视为障碍。这是离散栅格圆盘模型而非连续像素方块模型。
    """
    transform = validate_transform(transform)
    if isinstance(chunk_size, bool) or not isinstance(chunk_size, (int, np.integer)) or chunk_size <= 0:
        raise ValueError("chunk_size 必须为正整数")
    robot_radius = float(robot_radius)
    if not np.isfinite(robot_radius) or robot_radius < 0:
        raise ValueError("robot_radius 必须为有限非负值")
    threshold = _positive(occupancy_threshold, "occupancy_threshold")
    if threshold > 100:
        raise ValueError("occupancy_threshold 不得超过 100")
    common, full = np.zeros(7, dtype=np.int64), np.zeros(7, dtype=np.int64)
    for p, t in _pairs(prediction, ground_truth, transform, chunk_size):
        _accumulate(common, p, t, True, threshold)
    inverse = np.linalg.inv(transform)
    for t, p in _pairs(ground_truth, prediction, inverse, chunk_size):
        _accumulate(full, p, t, False, threshold)
    result = {
        "common_known": _metrics(common), "full_ground_truth": _metrics(full),
        "sampling": {"common_known": "prediction_cell_centers",
                     "full_ground_truth": "ground_truth_cell_centers",
                     "unknown_prediction": "miss", "occupancy_threshold": threshold,
                     "robot_radius_m": robot_radius,
                     "radius_model": "euclidean_blocked_cell_centers_including_outside_padding"},
    }
    if doors is not None:
        if not isinstance(doors, list):
            raise ValueError("doors 必须是包含 start/end 的 JSON 列表")
        pred_labels = _components(prediction, robot_radius, threshold)
        truth_labels = _components(ground_truth, robot_radius, threshold)
        items = []
        for index, door in enumerate(doors):
            if not isinstance(door, dict):
                raise ValueError("每个门必须包含 start/end")
            endpoints = []
            for key in ("start", "end"):
                point = np.asarray(door.get(key), dtype=float)
                if point.shape not in ((2,), (3,)) or not np.isfinite(point).all():
                    raise ValueError("门端点必须为有限的二维或三维 BIM 世界坐标")
                endpoints.append(point[:2])
            start, end = endpoints
            truth_connected = _connected(ground_truth, truth_labels, start, end)
            pred_points = [inverse[:2, :2] @ point + inverse[:2, 3] for point in endpoints]
            pred_connected = _connected(prediction, pred_labels, *pred_points)
            items.append({"index": index, "prediction_connected": pred_connected,
                          "ground_truth_connected": truth_connected,
                          "correct": pred_connected == truth_connected})
        correct = sum(item["correct"] for item in items)
        result["doors"] = {"items": items, "correct": correct, "total": len(items),
                           "accuracy": _ratio(correct, len(items))}
    return result


def _matrix(text):
    try:
        if "," in text:
            matrix = np.array([float(v) for v in text.split(",")]).reshape(4, 4)
        elif text.lower().endswith(".npy"):
            matrix = np.load(text, allow_pickle=False)
        else:
            matrix = np.loadtxt(text)
        return validate_transform(matrix)
    except (ValueError, OSError) as exc:
        raise argparse.ArgumentTypeError(f"无法读取平面 transform：{exc}") from exc


def build_parser():
    parser = argparse.ArgumentParser(prog="pcd2grid-eval", description="评估 ROS 预测地图与真值地图")
    parser.add_argument("prediction", help="预测地图 YAML")
    parser.add_argument("ground_truth", help="真值地图 YAML")
    parser.add_argument("--transform", type=_matrix, help="预测至 BIM/真值的 4x4 txt/npy 或 16 个逗号分隔数值")
    parser.add_argument("--doors", help="门端点 JSON 列表，每项含 start/end")
    parser.add_argument("--robot-radius", type=float, default=0.0, help="机器人半径（米），默认 0")
    parser.add_argument("--occupancy-threshold", type=float, default=50.0, help="已知概率的占据判定阈值（0..100，默认 50）")
    parser.add_argument("--chunk-size", type=int, default=262144, help="坐标采样分块格数")
    parser.add_argument("--output", help="输出 JSON 文件，默认 stdout")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        doors = None
        if args.doors:
            with open(args.doors, encoding="utf-8") as stream:
                doors = json.load(stream)
        result = evaluate_maps(load_map(args.prediction), load_map(args.ground_truth),
                               args.transform, doors=doors, robot_radius=args.robot_radius,
                               occupancy_threshold=args.occupancy_threshold,
                               chunk_size=args.chunk_size)
        text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        if args.output:
            Path(args.output).write_text(text, encoding="utf-8")
        else:
            sys.stdout.write(text)
    except (ValueError, OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        parser.error(f"评估失败：{exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
