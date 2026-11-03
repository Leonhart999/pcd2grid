# Changelog

All notable changes to pcd2grid are documented here.

## [0.1.0] - 2026-11-03

### Added

- Height-band projection of point clouds into Nav2 occupancy grids, generalised from the original `pcd_to_grid.py`.
- Readers for ASCII, binary and binary_compressed PCD, PLY, LAS/LAZ (optional `laspy`) and NPY.
- Ground modes `plane`, `grid` and `none`; `--up-axis`, `--transform`, `--crop`, `--z-slab`, `--voxel`, `--inflate`, `--origin` and `--origin-yaw`.
- `--level`: RANSAC floor detection over per-cell lowest points and rotation of tilted maps; the rotation is saved to `<prefix>_level.txt`.
- `--ground-z` to seed the ground fit, and `--max-cells` to refuse very large grids before allocating them.
- Warnings for a tilted floor, a ground estimate with many points below it, and maps without occupied cells.
- Map modes `trinary`, `scale` (RGBA PNG, transparent = unknown) and `raw`, plus preview PNG and optional `.npy` output.

### Fixed (compared with the original script)

- The ground is the lowest strong peak of the height histogram instead of the highest one, so tripod room scans no longer use the ceiling as the floor.
- `free_thresh` is 0.15 instead of 0.25. With 0.25, `map_server` read unknown cells (gray 205) as free.
