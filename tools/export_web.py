"""Trims a 3D Gaussian Splatting PLY down to only the properties this viewer actually
reads (x,y,z, f_dc_0..2, opacity, scale_0..2, rot_0..3 -- 14 floats/splat instead of the
full 62, which includes normals and 45 unused higher-order spherical-harmonic terms).
Lossless for this renderer (it never reads the dropped fields), and cuts file size by
~77% -- needed to make the web (Emscripten/WebGL2) build's payload shippable.

Usage:
    python export_web.py <input.ply> <output.ply>
"""

import sys
import numpy as np

KEEP_PROPERTIES = [
    "x", "y", "z",
    "f_dc_0", "f_dc_1", "f_dc_2",
    "opacity",
    "scale_0", "scale_1", "scale_2",
    "rot_0", "rot_1", "rot_2", "rot_3",
]


def read_ply(path):
    with open(path, "rb") as f:
        prop_names = []
        vertex_count = 0
        while True:
            line = f.readline()
            if not line:
                raise ValueError("unexpected EOF while reading PLY header")
            text = line.decode("ascii", errors="strict").strip()
            if text == "end_header":
                break
            if text.startswith("format") and "binary_little_endian" not in text:
                raise ValueError(f"unsupported PLY format: {text}")
            elif text.startswith("element vertex"):
                vertex_count = int(text.split()[-1])
            elif text.startswith("property float"):
                prop_names.append(text.split()[-1])
        data = np.fromfile(f, dtype=np.float32, count=vertex_count * len(prop_names))
        data = data.reshape(vertex_count, len(prop_names))
    return prop_names, data


def write_ply(path, prop_names, data):
    header_lines = ["ply", "format binary_little_endian 1.0", f"element vertex {data.shape[0]}"]
    header_lines += [f"property float {name}" for name in prop_names]
    header_lines.append("end_header")
    header = ("\n".join(header_lines) + "\n").encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        data.astype(np.float32).tofile(f)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    in_path, out_path = sys.argv[1], sys.argv[2]

    prop_names, data = read_ply(in_path)
    idx = {name: i for i, name in enumerate(prop_names)}
    missing = [p for p in KEEP_PROPERTIES if p not in idx]
    if missing:
        raise ValueError(f"input PLY is missing required properties: {missing}")

    trimmed = data[:, [idx[p] for p in KEEP_PROPERTIES]]
    write_ply(out_path, KEEP_PROPERTIES, trimmed)

    import os
    before = os.path.getsize(in_path)
    after = os.path.getsize(out_path)
    print(f"{data.shape[0]} splats: {len(prop_names)} -> {len(KEEP_PROPERTIES)} properties")
    print(f"{before/1e6:.1f} MB -> {after/1e6:.1f} MB ({100*after/before:.1f}%)")


if __name__ == "__main__":
    main()
