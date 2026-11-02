"""Point cloud readers. Every reader returns an (N, 3) float64 array of finite XYZ points."""
import os

import numpy as np

_PCD_TYPES = {
    ("F", 4): "<f4", ("F", 8): "<f8",
    ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
    ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
}

_PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "<i2", "int16": "<i2", "ushort": "<u2", "uint16": "<u2",
    "int": "<i4", "int32": "<i4", "uint": "<u4", "uint32": "<u4",
    "float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
}


def _finite(xyz):
    xyz = np.asarray(xyz, dtype=np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def _lzf_decompress(data, out_size):
    """Decompress LZF data as used by PCL's binary_compressed PCD format."""
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        ctrl = data[i]
        i += 1
        if ctrl < 32:
            # Literal run of ctrl + 1 bytes
            out += data[i:i + ctrl + 1]
            i += ctrl + 1
        else:
            length = ctrl >> 5
            if length == 7:
                length += data[i]
                i += 1
            ref = len(out) - ((ctrl & 0x1F) << 8) - data[i] - 1
            i += 1
            if ref < 0:
                raise ValueError("corrupt LZF stream")
            # Byte-by-byte copy: the reference may overlap the output
            for k in range(length + 2):
                out.append(out[ref + k])
    if len(out) != out_size:
        raise ValueError(f"LZF size mismatch: got {len(out)}, expected {out_size}")
    return bytes(out)


def read_pcd(path):
    """Read ASCII, binary and binary_compressed PCD files."""
    with open(path, "rb") as f:
        header = {}
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError("PCD header has no DATA line")
            line = raw.decode("ascii", errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            key, *vals = line.split()
            header[key] = vals
            if key == "DATA":
                break
        fields = header["FIELDS"]
        sizes = list(map(int, header["SIZE"]))
        types = header["TYPE"]
        counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
        n = int(header["POINTS"][0])
        mode = header["DATA"][0]
        for need in ("x", "y", "z"):
            if need not in fields:
                raise ValueError(f"PCD has no '{need}' field")

        if mode == "ascii":
            rows = np.loadtxt(f, dtype=np.float64, ndmin=2, max_rows=n)
            # Expand multi-count fields into column offsets
            col = {}
            c = 0
            for name, cnt in zip(fields, counts):
                col[name] = c
                c += cnt
            xyz = rows[:, [col["x"], col["y"], col["z"]]]
            return _finite(xyz)

        # Per-point record layout; fields with COUNT > 1 become a sub-array
        dt_fields = []
        for name, t, s, cnt in zip(fields, types, sizes, counts):
            base = _PCD_TYPES[(t, s)]
            dt_fields.append((name, base) if cnt == 1 else (name, base, (cnt,)))
        dtype = np.dtype(dt_fields)

        if mode == "binary":
            data = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
        elif mode == "binary_compressed":
            comp_size = int.from_bytes(f.read(4), "little")
            raw_size = int.from_bytes(f.read(4), "little")
            buf = _lzf_decompress(f.read(comp_size), raw_size)
            # Compressed data is stored column-wise (all x, then all y, ...)
            cols, off = {}, 0
            for name, t, s, cnt in zip(fields, types, sizes, counts):
                nbytes = n * s * cnt
                cols[name] = np.frombuffer(buf[off:off + nbytes], dtype=_PCD_TYPES[(t, s)])
                off += nbytes
            xyz = np.stack([cols["x"], cols["y"], cols["z"]], axis=1)
            return _finite(xyz)
        else:
            raise ValueError(f"unsupported PCD DATA mode: {mode}")
    xyz = np.stack([data["x"], data["y"], data["z"]], axis=1)
    return _finite(xyz)


def read_ply(path):
    """Read ASCII and binary (little/big endian) PLY vertex positions."""
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise ValueError("not a PLY file")
        fmt = None
        elements = []  # [name, count, [(prop_name, dtype)]]
        while True:
            line = f.readline().decode("ascii", errors="replace").strip()
            if not line:
                raise ValueError("PLY header has no end_header")
            parts = line.split()
            if parts[0] == "format":
                fmt = parts[1]
            elif parts[0] == "element":
                elements.append([parts[1], int(parts[2]), []])
            elif parts[0] == "property":
                if parts[1] == "list":
                    raise ValueError("PLY list properties are not supported for the vertex element"
                                     if not elements or elements[-1][0] == "vertex" else "")
                elements[-1][2].append((parts[2], _PLY_TYPES[parts[1]]))
            elif parts[0] == "end_header":
                break
        vertex = elements[0] if elements and elements[0][0] == "vertex" else None
        if vertex is None:
            raise ValueError("PLY must start with a 'vertex' element")
        _, n, props = vertex
        names = [p[0] for p in props]
        if fmt == "ascii":
            rows = np.loadtxt(f, dtype=np.float64, ndmin=2, max_rows=n)
            xyz = rows[:, [names.index("x"), names.index("y"), names.index("z")]]
        else:
            endian = "<" if fmt == "binary_little_endian" else ">"
            dtype = np.dtype([(nm, endian + t.lstrip("<>|")) for nm, t in props])
            data = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
            xyz = np.stack([data["x"], data["y"], data["z"]], axis=1)
    return _finite(xyz)


def read_las(path):
    """Read LAS/LAZ through the optional laspy dependency."""
    try:
        import laspy
    except ImportError as e:
        raise ImportError("LAS/LAZ support needs laspy: pip install 'pcd2grid[las]'") from e
    las = laspy.read(path)
    return _finite(np.stack([las.x, las.y, las.z], axis=1))


def read_npy(path):
    """Read an (N, >=3) .npy array; the first three columns are XYZ."""
    arr = np.load(path)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValueError(".npy must be an (N, 3+) array")
    return _finite(arr[:, :3])


_READERS = {".pcd": read_pcd, ".ply": read_ply, ".las": read_las, ".laz": read_las, ".npy": read_npy}


def read_points(path):
    """Dispatch on the file extension."""
    ext = os.path.splitext(path)[1].lower()
    if ext not in _READERS:
        raise ValueError(f"unsupported point cloud format '{ext}' (supported: {', '.join(sorted(_READERS))})")
    return _READERS[ext](path)
