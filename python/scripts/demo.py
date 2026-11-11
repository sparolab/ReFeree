#!/usr/bin/env python3
"""Match a query scan against a handful of candidates and recover the yaw.

    demo.py [scan_dir]      default: ../cpp/examples

Replaces the original test.py. Its heading estimation never applied the shift it
was looping over - `shift_index` went unused, so every iteration scored the same
pair, and the reported heading was always zero. It also took argmin of a
similarity, where the best match is the largest. Both are fixed here; the
descriptors themselves are unchanged.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import referee


def describe(path: str):
    scan = referee.load_polar_scan(path, referee.NavtechModel.CIR204)
    targets = referee.cen2018_features(scan.power)
    polar = referee.targets_to_polar_image(scan.power.shape, targets)
    return referee.make_range_descriptor(polar), referee.make_angle_descriptor(polar)


def main() -> int:
    scan_dir = Path(sys.argv[1]) if len(sys.argv) > 1 \
        else Path(__file__).resolve().parents[2] / "cpp/examples"

    names = referee.radar_file_names(str(scan_dir))
    if len(names) < 2:
        print(f"need at least two scans under {scan_dir}")
        return 1

    query_name, candidate_names = names[0], names[1:]
    print(f"query      : {query_name}")
    print(f"candidates : {', '.join(candidate_names)}\n")

    range_db = referee.RangeDescriptorDatabase()
    angle_db = referee.AngleDescriptorDatabase()
    for name in candidate_names:
        r, a = describe(str(scan_dir / name))
        range_db.add(r)
        angle_db.add(a)

    query_range, query_angle = describe(str(scan_dir / query_name))
    match = range_db.detect_loop_closure(query_range)
    if not match:
        print("no candidate cleared the similarity threshold")
        return 0

    yaw = angle_db.yaw_difference(query_angle, match.index)
    print(f"matched    : {candidate_names[match.index]}")
    print(f"similarity : {match.similarity:.6f}")
    print(f"yaw        : {math.degrees(yaw):+.2f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
