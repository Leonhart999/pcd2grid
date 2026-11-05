"""评估模块的合成地图、采样几何、ROS 模式和门连通测试。"""
import json

import numpy as np
from PIL import Image
import pytest
import yaml

from pcd2grid.eval import MapGrid, evaluate_maps, load_map, main, validate_transform


def write_fixture(tmp_path, pixels, mode="trinary", **overrides):
    image_path = tmp_path / "map.png"
    Image.fromarray(np.asarray(pixels, dtype=np.uint8)).save(image_path)
    meta = dict(image=image_path.name, mode=mode, resolution=1.0,
                origin=[0, 0, 0], negate=0, free_thresh=0.15, occupied_thresh=0.65)
    meta.update(overrides)
    path = tmp_path / "map.yaml"
    path.write_text(yaml.safe_dump(meta), encoding="utf-8")
    return path


def transform(yaw=0, x=0, y=0):
    c, s = np.cos(yaw), np.sin(yaw)
    matrix = np.eye(4)
    matrix[:2, :2] = [[c, -s], [s, c]]
    matrix[:2, 3] = [x, y]
    return matrix


def test_identical_and_counts():
    grid = MapGrid([[100, 0, -1], [0, 100, 0]], 0.1)
    report = evaluate_maps(grid, grid, chunk_size=1)
    for scope in ("common_known", "full_ground_truth"):
        metrics = report[scope]
        assert metrics["occupied_precision"] == 1
        assert metrics["occupied_recall"] == 1
        assert metrics["occupied_iou"] == 1
        assert metrics["free_coverage"] == 1
        assert metrics["counts"]["evaluated_cells"] == 5
        assert metrics["counts"]["ground_truth_occupied"] == 2
    assert report == evaluate_maps(grid, grid, chunk_size=100)


@pytest.mark.parametrize("matrix,origin", [
    (transform(x=5, y=-3), (5, -3, 0)),
    (transform(np.pi / 2, 2, 3), (2, 3, np.pi / 2)),
    (transform(-0.73, 5, -1), (5, -1, -0.73)),
])
def test_alignment_translation_rotation(matrix, origin):
    values = [[100, 0, -1], [0, 100, 0]]
    prediction = MapGrid(values, 0.3)
    truth = MapGrid(values, 0.3, origin)
    metrics = evaluate_maps(prediction, truth, matrix, chunk_size=2)
    assert metrics["common_known"]["occupied_iou"] == 1
    assert metrics["full_ground_truth"]["free_coverage"] == 1


def test_origin_yaw_without_transform():
    prediction = MapGrid([[100, 0]], 1, (0, 0, np.pi / 2))
    truth = MapGrid([[100], [0]], 1, (-1, 0, 0))
    metrics = evaluate_maps(prediction, truth)
    assert metrics["full_ground_truth"]["occupied_iou"] == 1
    assert metrics["full_ground_truth"]["free_coverage"] == 1


def test_unknown_and_outside_prediction_are_misses():
    prediction = MapGrid([[100, -1, -1]], 1)
    truth = MapGrid([[100, 100, 0, 0]], 1)
    metrics = evaluate_maps(prediction, truth)
    assert metrics["common_known"]["occupied_recall"] == 1
    full = metrics["full_ground_truth"]
    assert full["occupied_precision"] == 1
    assert full["occupied_recall"] == 0.5
    assert full["occupied_iou"] == 0.5
    assert full["free_coverage"] == 0
    assert full["known_coverage"] == 0.25


def test_false_positive_denominators():
    prediction = MapGrid([[100, 100, 0, 0]], 1)
    truth = MapGrid([[100, 0, 100, 0]], 1)
    metrics = evaluate_maps(prediction, truth)["full_ground_truth"]
    assert metrics["occupied_precision"] == 0.5
    assert metrics["occupied_recall"] == 0.5
    assert metrics["occupied_iou"] == pytest.approx(1 / 3)
    assert metrics["free_coverage"] == 0.5


def test_full_truth_samples_truth_resolution():
    prediction = MapGrid([[100]], 2)
    truth = MapGrid([[100, 100], [100, 100]], 1)
    report = evaluate_maps(prediction, truth)
    assert report["common_known"]["counts"]["evaluated_cells"] == 1
    assert report["full_ground_truth"]["counts"]["evaluated_cells"] == 4
    assert report["full_ground_truth"]["occupied_iou"] == 1


@pytest.mark.parametrize("values", [[[-1]], [[0]], [[100]]])
def test_empty_denominators_and_strict_json(values):
    grid = MapGrid(values, 1)
    report = evaluate_maps(grid, grid, doors=[])
    metrics = report["full_ground_truth"]
    if values == [[-1]]:
        assert all(metrics[name] is None for name in (
            "occupied_precision", "occupied_recall", "occupied_iou", "free_coverage", "known_coverage"))
    elif values == [[0]]:
        assert metrics["occupied_precision"] is None
        assert metrics["occupied_iou"] is None
    else:
        assert metrics["free_coverage"] is None
    assert report["doors"]["accuracy"] is None
    json.dumps(report, allow_nan=False)


def test_trinary_flip_thresholds_negate(tmp_path):
    path = write_fixture(tmp_path, [[0, 205, 255], [255, 0, 205]])
    assert load_map(path).values.tolist() == [[0, 100, -1], [100, -1, 0]]
    path = write_fixture(tmp_path, [[0, 255]], negate=1)
    assert load_map(path).values.tolist() == [[0, 100]]


def test_trinary_rgb_mean_and_alpha_averaged(tmp_path):
    path = write_fixture(tmp_path, [[[0, 0, 0, 0], [255, 255, 255, 0], [255, 0, 0, 255]]])
    assert load_map(path).values.tolist() == [[100, -1, -1]]


def test_scale_probabilities_and_alpha(tmp_path):
    path = write_fixture(tmp_path, [[[0, 0, 0, 255], [255, 255, 255, 255],
                                     [153, 153, 153, 255], [0, 0, 0, 0],
                                     [255, 255, 255, 254]]], mode="scale", free_thresh=0.2)
    grid = load_map(path)
    assert grid.values[0, :2].tolist() == [100, 0]
    assert grid.values[0, 2] == 44
    assert grid.values[0, 3:].tolist() == [-1, -1]


def test_raw_values_unknown_and_negate(tmp_path):
    path = write_fixture(tmp_path, [[0, 20, 50, 100, 101, 255]], mode="raw")
    assert load_map(path).values.tolist() == [[0, 20, 50, 100, -1, -1]]
    path = write_fixture(tmp_path, [[0, 100]], mode="raw", negate=1)
    assert load_map(path).values.tolist() == [[0, 100]]


def test_door_connectivity_unknown_wall_and_transform():
    truth = MapGrid([[0, 0, 0], [0, 0, 0], [0, 0, 0]], 1, (10, 2, 0))
    prediction = MapGrid([[0, -1, 0], [0, -1, 0], [0, -1, 0]], 1)
    doors = [{"start": [10.5, 3.5], "end": [12.5, 3.5]},
             {"start": [10.5, 2.5, 5], "end": [10.5, 4.5, 5]},
             {"start": [0, 0], "end": [0, 1]}]
    report = evaluate_maps(prediction, truth, transform(x=10, y=2), doors=doors)["doors"]
    assert report["accuracy"] == pytest.approx(2 / 3)
    assert report["items"][0]["prediction_connected"] is False
    assert report["items"][0]["ground_truth_connected"] is True
    assert report["items"][1]["correct"] is True
    assert report["items"][2]["ground_truth_connected"] is False


def test_door_four_not_eight_connectivity():
    grid = MapGrid([[0, 100], [100, 0]], 1)
    report = evaluate_maps(grid, grid, doors=[{"start": [0.5, 0.5], "end": [1.5, 1.5]}])
    assert report["doors"]["items"][0]["prediction_connected"] is False


def test_robot_radius_meters_unknown_and_outside_blocked():
    grid = MapGrid(np.zeros((7, 7)), 0.1)
    doors = [{"start": [0.05, 0.35], "end": [0.65, 0.35]},
             {"start": [0.25, 0.35], "end": [0.45, 0.35]}]
    base = evaluate_maps(grid, grid, doors=doors)["doors"]["items"]
    inflated = evaluate_maps(grid, grid, doors=doors, robot_radius=0.11)["doors"]["items"]
    assert base[0]["prediction_connected"] is True
    assert inflated[0]["prediction_connected"] is False
    assert inflated[1]["prediction_connected"] is True
    grid.values[3, 3] = -1
    report = evaluate_maps(grid, grid, doors=doors, robot_radius=0.11)
    assert report["doors"]["items"][1]["prediction_connected"] is False


def test_euclidean_not_square_dilation():
    grid = MapGrid(np.zeros((9, 9)), 0.1)
    grid.values[4, 4] = 100
    doors = [{"start": [0.55, 0.55], "end": [0.65, 0.55]}]
    report = evaluate_maps(grid, grid, doors=doors, robot_radius=0.13)
    assert report["doors"]["items"][0]["prediction_connected"] is True


@pytest.mark.parametrize("matrix", [np.eye(3), np.full((4, 4), np.nan),
    np.diag([2, 2, 1, 1]), np.diag([-1, 1, 1, 1]),
    [[1, 0, 0.1, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]],
    [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0.1, 1]],
])
def test_invalid_transform(matrix):
    with pytest.raises(ValueError, match="transform"):
        validate_transform(matrix)


@pytest.mark.parametrize("kwargs", [{"robot_radius": -1}, {"robot_radius": np.nan},
    {"robot_radius": np.inf}, {"chunk_size": 0}, {"chunk_size": 1.5},
    {"chunk_size": True}, {"occupancy_threshold": 0}, {"occupancy_threshold": 101},
    {"occupancy_threshold": np.nan}])
def test_invalid_eval_parameters(kwargs):
    grid = MapGrid([[0]], 1)
    with pytest.raises(ValueError):
        evaluate_maps(grid, grid, **kwargs)


@pytest.mark.parametrize("metadata", [{"resolution": 0}, {"resolution": np.inf},
    {"origin": [0, 0, np.nan]}, {"origin": [0, 0]},
    {"free_thresh": 0.8}, {"occupied_thresh": np.nan},
    {"negate": 2}, {"mode": "other"}])
def test_invalid_map_metadata(tmp_path, metadata):
    path = write_fixture(tmp_path, [[0]], **metadata)
    with pytest.raises(ValueError):
        load_map(path)


@pytest.mark.parametrize("doors", [{}, [None], [{}], [{"start": [np.inf, 0], "end": [0, 0]}]])
def test_invalid_doors(doors):
    grid = MapGrid([[0]], 1)
    with pytest.raises(ValueError):
        evaluate_maps(grid, grid, doors=doors)


def test_cli_stdout_output_and_transform_files(tmp_path, capsys):
    path = write_fixture(tmp_path, [[0, 255]])
    assert main([str(path), str(path), "--chunk-size", "1"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["full_ground_truth"]["occupied_iou"] == 1
    output = tmp_path / "result.json"
    matrix_path = tmp_path / "transform.npy"
    np.save(matrix_path, np.eye(4))
    doors_path = tmp_path / "doors.json"
    doors_path.write_text("[]", encoding="utf-8")
    assert main([str(path), str(path), "--output", str(output), "--transform", str(matrix_path),
                 "--doors", str(doors_path)]) == 0
    assert json.loads(output.read_text(encoding="utf-8"))["doors"]["accuracy"] is None
    assert capsys.readouterr().out == ""
    text_matrix = tmp_path / "transform.txt"
    np.savetxt(text_matrix, np.eye(4))
    assert main([str(path), str(path), "--transform", str(text_matrix)]) == 0
    assert main([str(path), str(path), "--transform", ",".join(map(str, np.eye(4).ravel()))]) == 0


def test_map_yaml_origin_yaw_and_resolution(tmp_path):
    path = write_fixture(tmp_path, [[0, 255]], resolution=0.05, origin=[3.0, -2.0, 0.7])
    grid = load_map(path)
    assert grid.resolution == 0.05
    assert grid.origin == (3.0, -2.0, 0.7)


def test_many_door_components_keep_integer_precision():
    values = np.full((365, 365), 100)
    values[::2, ::2] = 0
    grid = MapGrid(values, 1)
    point = [364.5, 364.5]
    report = evaluate_maps(grid, grid, doors=[{"start": point, "end": point}])
    assert report["doors"]["items"][0]["prediction_connected"] is True


def test_cli_rejects_nonfinite_radius(tmp_path, capsys):
    path = write_fixture(tmp_path, [[255]])
    with pytest.raises(SystemExit) as error:
        main([str(path), str(path), "--robot-radius", "nan"])
    assert error.value.code == 2
    assert "robot_radius" in capsys.readouterr().err
