#!/usr/bin/env python3
"""MulRan polar scans (range x azimuth PNG, as downloaded) to the Oxford layout the odometry reads.

    python3 mulran_to_oxford.py <MulRan>/KAIST_03/polar <out>/KAIST_03/polar [--period 0.25]

Oxford layout: one row per azimuth, 11 metadata bytes (int64 timestamp, uint16 encoder count out of 5600,
uint8 valid = 255) then the range bins. MulRan scans carry no per-azimuth time, so it is spread linearly
over one revolution (--period, 4 Hz for the CIR204 of MulRan) from the file name's time, in the file name's
unit (nanoseconds for MulRan). The azimuth order is reversed to match the Oxford encoder direction.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("src")
    p.add_argument("dst")
    p.add_argument("--period", type=float, default=0.25, help="seconds per revolution")
    a = p.parse_args()
    src, dst = Path(a.src), Path(a.dst)
    dst.mkdir(parents=True, exist_ok=True)
    files = sorted(src.glob("*.png"))
    for k, f in enumerate(files):
        img = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
        if img.shape[0] > img.shape[1]:          # range x azimuth -> azimuth x range
            img = img.T
        # MulRan's azimuths run the other way round from the Oxford encoder count: without this the
        # odometry comes out mirrored against MulRan's ground truth.
        img = img[::-1]
        n_az, n_rng = img.shape
        t0 = int(f.stem)
        out = np.zeros((n_az, 11 + n_rng), np.uint8)
        stamps = (t0 + np.arange(n_az) * (a.period * 1e9 / n_az)).astype(np.int64)
        out[:, 0:8] = stamps.view(np.uint8).reshape(n_az, 8)
        out[:, 8:10] = np.round(np.arange(n_az) * 5600 / n_az).astype(np.uint16).view(np.uint8).reshape(n_az, 2)
        out[:, 10] = 255
        out[:, 11:] = img
        cv2.imwrite(str(dst / f.name), out)
        if k % 500 == 0:
            print(f"  {k} / {len(files)}", flush=True)
    print(f"{len(files)} scans -> {dst}")


if __name__ == "__main__":
    main()
