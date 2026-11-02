"""Writers for the grid: Nav2 map_server pgm+yaml, npy, and a preview PNG."""
import os

import numpy as np
from PIL import Image


def _gray_to_scale(grid):
    """Map the trinary gray values to 0..100 occupancy, -1 for unknown (Nav2 'scale' mode)."""
    out = np.full(grid.shape, -1, dtype=np.int16)
    out[grid == 254] = 0
    out[grid == 0] = 100
    return out


def write_map(result, out_prefix, mode="trinary", origin_yaw=0.0, preview=True, save_npy=False):
    """Write <prefix>.pgm and <prefix>.yaml (plus optional preview PNG and .npy).

    mode: trinary | scale | raw (Nav2 map_server modes).
    Returns the list of written file paths.
    """
    if mode not in ("trinary", "scale", "raw"):
        raise ValueError(f"invalid map mode '{mode}'")
    out_dir = os.path.dirname(os.path.abspath(out_prefix))
    os.makedirs(out_dir, exist_ok=True)
    name = os.path.basename(out_prefix)
    img = result.image  # first row = largest y, as map_server expects

    if mode == "trinary":
        pgm = img
        negate = 0
    else:
        # Grayscale encodes occupancy probability: darker = more occupied.
        # Unknown cells use 205 (gray), which map_server reads as unknown in scale/raw mode.
        pgm = img.copy()
        pgm[img == 0] = 0
        pgm[img == 254] = 254
        negate = 0

    pgm_path = out_prefix + ".pgm"
    yaml_path = out_prefix + ".yaml"
    Image.fromarray(pgm).save(pgm_path)
    with open(yaml_path, "w", newline="\n") as f:
        f.write(f"image: {name}.pgm\n"
                f"mode: {mode}\n"
                f"resolution: {result.res}\n"
                f"origin: [{result.origin[0]:.3f}, {result.origin[1]:.3f}, {origin_yaw}]\n"
                f"negate: {negate}\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.25\n")
    written = [pgm_path, yaml_path]
    if preview:
        png = out_prefix + "_preview.png"
        Image.fromarray(img).save(png)
        written.append(png)
    if save_npy:
        npy = out_prefix + ".npy"
        np.save(npy, _gray_to_scale(result.grid))
        written.append(npy)
    return written
