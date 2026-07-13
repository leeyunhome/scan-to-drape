"""Cleans a 3D Gaussian Splatting PLY by removing local scale outliers: a splat whose
scale is anomalously large relative to its own neighborhood, not a single global
threshold. This is what actually causes the long screen-space streaks in the raw
training checkpoint (see PORTFOLIO_PLAN.md Phase 1.6).

Method: for each splat, find its K=20 nearest neighbors by position, compute the
neighborhood's median max-axis-scale, and flag the splat as an outlier if its own
scale exceeds 4x that local median.

Two other approaches were tried and rejected empirically (not just in theory — actually
rendered and compared by screenshot):
- A single global percentile cutoff on scale (the v0 loader's `--keep-percentile`)
  works reasonably but has more collateral damage on legitimate large splats in
  coarse/flat regions than the local method.
- Open3D's position-based statistical outlier removal (`remove_statistical_outlier`)
  was tested in combination with the local-scale signal. It removes points that sit in
  locally sparse *positions* — but on this capture, locally-sparse-in-position doesn't
  mean "isolated floater noise," it often means "a real but coarsely-sampled surface
  region" (e.g. a large flat panel covered by fewer, bigger splats). Combining it in
  visibly tore holes in legitimate surface coverage when actually rendered — worse than
  not cleaning at all. Confirmed by isolating the two signals into separate exports and
  comparing screenshots side by side; the local-scale-only result preserved body
  coverage while the position-based signal did not. Dropped entirely, not just tuned
  down, because the failure mode is structural (it targets sampling density, not scale)
  rather than a threshold-tuning problem.

Usage:
    python clean_scan.py <input.ply> <output.ply>
"""

import sys
import numpy as np
from scipy.spatial import cKDTree

K_NEIGHBORS = 20
SCALE_RATIO_THRESHOLD = 4.0


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
            if text.startswith("format"):
                if "binary_little_endian" not in text:
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
    n = data.shape[0]
    print(f"loaded {n} splats, {len(prop_names)} properties")

    scale = np.exp(data[:, [idx["scale_0"], idx["scale_1"], idx["scale_2"]]])
    max_scale = scale.max(axis=1)
    positions = data[:, [idx["x"], idx["y"], idx["z"]]]

    tree = cKDTree(positions)
    _, neighbor_idx = tree.query(positions, k=K_NEIGHBORS + 1)  # column 0 is the point itself
    neighbor_idx = neighbor_idx[:, 1:]
    local_median_scale = np.median(max_scale[neighbor_idx], axis=1)
    outlier = max_scale > SCALE_RATIO_THRESHOLD * local_median_scale
    kept = ~outlier

    print(f"local scale outliers (K={K_NEIGHBORS}, ratio>{SCALE_RATIO_THRESHOLD}x local median): "
          f"{int(outlier.sum())} ({100.0 * outlier.sum() / n:.2f}%)")
    print(f"kept: {int(kept.sum())}")

    write_ply(out_path, prop_names, data[kept])
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
