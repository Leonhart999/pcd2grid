"""Writers for the grid: Nav2 map_server image+yaml, npy, and a preview PNG."""
import os

import numpy as np
from PIL import Image


def _gray_to_scale(grid):
    """Map the trinary gray values to 0..100 occupancy, -1 for unknown."""
    out = np.full(grid.shape, -1, dtype=np.int16)
    out[grid == 254] = 0
    out[grid == 0] = 100
    return out


def write_map(result, out_prefix, mode="trinary", origin_yaw=0.0, preview=True, save_npy=False):
    """Write <prefix>.pgm (or .png in scale mode) and <prefix>.yaml, plus optional preview and .npy.

    mode: trinary | scale | raw (Nav2 map_server modes).
    Returns the list of written file paths.
    """
    if mode not in ("trinary", "scale", "raw"):
        raise ValueError(f"invalid map mode '{mode}'")
    out_dir = os.path.dirname(os.path.abspath(out_prefix))
    os.makedirs(out_dir, exist_ok=True)
    name = os.path.basename(out_prefix)
    img = result.image  # first row = largest y, as map_server expects
    negate = 0

    if mode == "scale":
        # scale: gray maps linearly to occupancy (255 -> 0, 0 -> 100). A PGM has no way to mark
        # unknown cells, so write a gray+alpha PNG: map_server reads alpha = 0 as unknown (-1).
        gray = np.full(img.shape, 255, dtype=np.uint8)
        gray[img == 0] = 0
        alpha = np.where(img == 205, 0, 255).astype(np.uint8)
        gray[img == 205] = 127
        # RGBA rather than LA: map_server's image loader only honours alpha in 4-channel images
        map_img = Image.fromarray(np.dstack([gray, gray, gray, alpha]), "RGBA")
        image_name = name + ".png"
    else:
        if mode == "trinary":
            pgm = img
        else:
            # raw: the pixel value is the occupancy itself (0..100); above 100 means unknown.
            pgm = np.full(img.shape, 255, dtype=np.uint8)
            pgm[img == 254] = 0
            pgm[img == 0] = 100
        map_img = Image.fromarray(pgm)
        image_name = name + ".pgm"

    image_path = os.path.join(out_dir, image_name)
    yaml_path = out_prefix + ".yaml"
    map_img.save(image_path)
    with open(yaml_path, "w", newline="\n") as f:
        f.write(f"image: {image_name}\n"
                f"mode: {mode}\n"
                f"resolution: {result.res}\n"
                f"origin: [{result.origin[0]:.3f}, {result.origin[1]:.3f}, {origin_yaw}]\n"
                f"negate: {negate}\n"
                f"occupied_thresh: 0.65\n"
                # unknown (205 -> p = 0.196) must stay above free_thresh, else map_server reads it as free
                f"free_thresh: 0.15\n")
    written = [image_path, yaml_path]
    if preview:
        png = out_prefix + "_preview.png"
        Image.fromarray(img).save(png)
        written.append(png)
    if save_npy:
        npy = out_prefix + ".npy"
        np.save(npy, _gray_to_scale(result.grid))
        written.append(npy)
    return written
