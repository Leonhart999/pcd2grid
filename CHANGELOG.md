# Changelog

All notable changes to pcd2grid are documented here.

## [Unreleased]

### Added

- `--ground-mode auto`: uses the tiled `grid` ground model when the long side of the point cloud's XY bounding box (after crop and `--level`) exceeds `--auto-ground-size` (default 100 m), and the single global `plane` otherwise. The decision is written to the info line and the `--report` JSON. The default mode stays `plane`.
- Optional continuous `heightmap` ground model with per-cell minima, slope-to-center compensation, nearest-neighbor filling, median filtering and Gaussian smoothing.
- Opt-in four-connected obstacle/free component area filters; deleted obstacles remain unknown.
- `--start` / `--robot-radius` connectivity statistics and reproducible `--report` JSON output.
- Experimental `--trajectory` TUM-based conservative free-space raycasting with range/ray budgets and obstacle/corner blocking. Fused clouds do not provide actual scan-pose correspondence; existing occupied cells are never cleared.
- `pcd2grid-eval` with ROS YAML/image loading, planar rigid alignment, full-ground-truth/common-known occupancy metrics and optional point-pair connectivity checks.
- `pcd2grid-scans`: occupancy grid from one point cloud per frame (a directory of PCD/PLY/NPY/LAS/LAZ files) and a TUM trajectory. A cell is occupied only if obstacle-height points were seen in it in enough frames over enough path length and few beams of other frames passed through it. Frames and poses are paired by timestamp (including `<index>_<unix time>` file names) or by order; `--sensor-to-body` maps the clouds into the pose frame; the sensor height is estimated or given. Needs poses in a level world frame and a single floor; tested on synthetic scenes and one real corridor recording, not validated in Nav2.
- Parameter validation and regression tests, including exact tiled-ground equivalence against legacy masks.
- PyYAML dependency for evaluation map loading.

### Notes

- The per-tile sorted-slice performance fix was already present in the starting checkout; it is now protected by an equivalence test.
- These changes have not been released or validated as safe Nav2 maps. Heightmap failure modes and approximate raycasting are documented; no BIM accuracy improvement is claimed.

## [0.1.0] - 2026-11-03

### Added

- Height-band projection of point clouds into Nav2 occupancy grids, generalised from the original `pcd_to_grid.py`.
- Readers for ASCII, binary and binary_compressed PCD, PLY, LAS/LAZ (optional `laspy`) and NPY.
- Ground modes `plane`, `grid` and `none`; `--up-axis`, `--transform`, `--crop`, `--z-slab`, `--voxel`, `--inflate`, `--origin` and `--origin-yaw`.
- `--level`: RANSAC floor detection over per-cell lowest points and rotation of tilted maps; the rotation is saved to `<prefix>_level.txt`.
- `--ground-z` to seed the ground fit, and `--max-cells` to refuse very large grids before allocating them.
- `--compare`: side-by-side PNG of the point cloud top view, per-cell point classes and the resulting grid.
- Warnings for a tilted floor, a ground estimate with many points below it, and maps without occupied cells.
- Map modes `trinary`, `scale` (RGBA PNG, transparent = unknown) and `raw`, plus preview PNG and optional `.npy` output.

### Fixed (compared with the original script)

- The ground is the lowest strong peak of the height histogram instead of the highest one, so tripod room scans no longer use the ceiling as the floor.
- `free_thresh` is 0.15 instead of 0.25. With 0.25, `map_server` read unknown cells (gray 205) as free.
