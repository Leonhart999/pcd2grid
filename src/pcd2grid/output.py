"""Writers for the grid: Nav2 map_server image+yaml, npy, and a preview PNG."""
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# Cell classes from core.convert(..., layers=True) and their colours in the comparison image
_CLASS_COLOURS = {
    0: ((40, 40, 40), "no points"),
    1: ((255, 140, 0), "below ground"),
    2: ((70, 110, 255), "above obstacle band (ignored)"),
    3: ((60, 180, 75), "ground"),
    4: ((230, 40, 40), "obstacle band"),
}
# Height colour ramp: dark blue -> cyan -> green -> yellow -> red
_RAMP = np.array([[48, 18, 160], [30, 160, 230], [60, 200, 90], [250, 220, 40], [220, 40, 30]], dtype=float)


def _gray_to_scale(grid):
    """Map the trinary gray values to 0..100 occupancy, -1 for unknown."""
    out = np.full(grid.shape, -1, dtype=np.int16)
    out[grid == 254] = 0
    out[grid == 0] = 100
    return out


def _font(size):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow < 10.1 has no sized default font
        return ImageFont.load_default()


def _height_colours(top, lo, hi):
    """Colour each cell by its highest point; cells without points are dark gray."""
    t = np.clip((top - lo) / max(hi - lo, 1e-6), 0, 1) * (len(_RAMP) - 1)
    rgb = np.empty(top.shape + (3,))
    for c in range(3):
        rgb[..., c] = np.interp(t, np.arange(len(_RAMP)), _RAMP[:, c])
    rgb[np.isnan(top)] = 40
    return rgb.astype(np.uint8)


def write_compare(result, path, obs_max=None):
    """Write a side-by-side PNG: input top view coloured by height, cell classes, and the grid.

    All three panels share the grid geometry, so a pixel in one panel is the same place in the others.
    """
    if result.layers is None:
        raise ValueError("comparison needs layers; convert with GridParams(layers=True)")
    top = result.layers["top_height"]
    finite = top[np.isfinite(top)]
    lo = float(np.percentile(finite, 2)) if finite.size else 0.0
    hi = float(np.percentile(finite, 98)) if finite.size else 1.0
    if obs_max is not None:
        hi = max(hi, obs_max)
    classes = result.layers["classes"]
    cls_rgb = np.zeros(classes.shape + (3,), dtype=np.uint8)
    for k, (colour, _) in _CLASS_COLOURS.items():
        cls_rgb[classes == k] = colour
    panels = [np.flipud(_height_colours(top, lo, hi)), np.flipud(cls_rgb),
              np.repeat(result.image[..., None], 3, axis=2)]

    # Small maps are enlarged so that the panels are readable
    h, w = result.grid.shape
    scale = max(1, int(np.ceil(360 / min(h, w))))
    panels = [np.kron(p, np.ones((scale, scale, 1), dtype=np.uint8)) for p in panels]
    ph, pw = panels[0].shape[:2]
    pad, title_h, legend_h = 12, 30, 110
    canvas = Image.new("RGB", (3 * pw + 4 * pad, ph + title_h + legend_h + 2 * pad), (255, 255, 255))
    draw = ImageDraw.Draw(canvas)
    font = _font(16)
    titles = ["Point cloud, top view (highest point per cell)", "Point classes per cell", "Occupancy grid"]
    for i, (panel, title) in enumerate(zip(panels, titles)):
        x0 = pad + i * (pw + pad)
        canvas.paste(Image.fromarray(panel), (x0, pad + title_h))
        draw.text((x0, pad), title, fill=(0, 0, 0), font=font)

    y0 = pad + title_h + ph + 10
    # Panel 1 legend: height colour bar
    bar_w = min(pw, 300)
    ramp = _height_colours(np.linspace(lo, hi, bar_w)[None, :].repeat(14, axis=0), lo, hi)
    canvas.paste(Image.fromarray(ramp), (pad, y0))
    draw.text((pad, y0 + 18), f"{lo:.2f} m", fill=(0, 0, 0), font=font)
    draw.text((pad + bar_w - 60, y0 + 18), f"{hi:.2f} m", fill=(0, 0, 0), font=font)
    draw.text((pad, y0 + 40), "height above ground; dark gray = no points", fill=(0, 0, 0), font=font)
    # Panel 2 legend: class colours
    x1 = 2 * pad + pw
    for row, (colour, label) in enumerate(c for k, c in sorted(_CLASS_COLOURS.items(), reverse=True)):
        yy = y0 + row * 20
        draw.rectangle([x1, yy + 2, x1 + 14, yy + 16], fill=colour)
        draw.text((x1 + 22, yy), label, fill=(0, 0, 0), font=font)
    # Panel 3 legend: grid values and a scale bar
    x2 = 3 * pad + 2 * pw
    for row, (colour, label) in enumerate([((0, 0, 0), "occupied"), ((254, 254, 254), "free"),
                                           ((205, 205, 205), "unknown")]):
        yy = y0 + row * 20
        draw.rectangle([x2, yy + 2, x2 + 14, yy + 16], fill=colour, outline=(0, 0, 0))
        draw.text((x2 + 22, yy), label, fill=(0, 0, 0), font=font)
    span = w * result.res
    bar_m = next(m for m in (0.5, 1, 2, 5, 10, 20, 50, 100, 500, 1000) if m >= span / 8)
    bar_px = int(round(bar_m / result.res * scale))
    yy = y0 + 70
    draw.rectangle([x2, yy, x2 + bar_px, yy + 6], fill=(0, 0, 0))
    draw.text((x2 + bar_px + 8, yy - 6), f"{bar_m:g} m", fill=(0, 0, 0), font=font)
    canvas.save(path)
    return path


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
