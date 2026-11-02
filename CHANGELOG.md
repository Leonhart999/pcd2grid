# Changelog

All notable changes to pcd2grid are documented here.

## [0.1.0] - 2026-11-03

### Added

- Height-band projection of point clouds into Nav2 occupancy grids (`.pgm` + `.yaml`), generalised from the original `pcd_to_grid.py`.
- Readers for ASCII, binary and binary_compressed PCD, PLY, LAS/LAZ (optional `laspy`) and NPY.
- Ground modes `plane`, `grid` and `none`; `--up-axis`, `--transform`, `--crop`, `--z-slab`, `--voxel`, `--inflate`, `--origin` and `--origin-yaw`.
- Map modes `trinary`, `scale` and `raw`, plus preview PNG and optional `.npy` output.
- Synthetic-cloud tests covering tilted ground, walls, ceilings, orientation, I/O formats and CLI.
