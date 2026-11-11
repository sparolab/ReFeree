"""Per-keyframe ReFeree features and range descriptors of a SLAM session, for tools/make_gif.py.

    python tools/gif_features.py <session> --polar <Oxford-layout polar dir> --out features.npz
"""
import argparse
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from referee.descriptor import RangeDescriptorParams, make_range_descriptor  # noqa: E402
from referee.features import cen2018_features  # noqa: E402
from referee.radar_utils import NavtechModel, load_polar_scan, targets_to_polar_image  # noqa: E402

RES = 0.0596


def one(path):
    s = load_polar_scan(path, NavtechModel.CIR204)
    t = cen2018_features(s.power)
    polar = targets_to_polar_image(s.power.shape, t)
    d = make_range_descriptor(polar, RangeDescriptorParams(legacy_window_alignment=False))
    if t.size == 0:
        return np.zeros((0, 2), np.float32), d
    az, r = s.azimuths[t[:, 0].astype(int)], (t[:, 1] + 0.5) * RES
    return np.c_[r * np.cos(az), r * np.sin(az)].astype(np.float32), d


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("session")
    p.add_argument("--polar", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--workers", type=int, default=6)
    a = p.parse_args()
    files = sorted(Path(a.polar).glob("*.png"))
    idx = np.loadtxt(Path(a.session) / "keyframe_scan_index.txt", dtype=int)
    with Pool(a.workers) as pool:
        res = pool.map(one, [str(files[i]) for i in idx], chunksize=8)
    pts = [r[0] for r in res]
    np.savez_compressed(a.out, counts=np.array([len(q) for q in pts]), points=np.concatenate(pts),
                        descriptors=np.stack([r[1] for r in res]), scans=np.array([files[i].name for i in idx]))
    print(f"{a.out}: {len(idx)} keyframes, {sum(len(q) for q in pts)} points")


if __name__ == "__main__":
    main()
