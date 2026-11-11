"""Checks the package against the descriptors the published C++ produced.

    python3 -m pytest tests/ -v
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))
import referee  # noqa: E402

EXAMPLES = ROOT / "cpp/examples"
REFERENCE = ROOT / "cpp/tools/reference/published_descriptors.txt"


def load_reference():
    if not REFERENCE.exists():
        pytest.skip(f"reference fixture missing: {REFERENCE}")
    out = {}
    for line in REFERENCE.read_text().splitlines():
        if not line.strip():
            continue
        parts = line.split()
        out[(parts[0], parts[1])] = np.array(parts[2:], dtype=np.float64)
    return out


def describe(name):
    scan = referee.load_polar_scan(str(EXAMPLES / name), referee.NavtechModel.CIR204)
    targets = referee.cen2018_features(scan.power)
    polar = referee.targets_to_polar_image(scan.power.shape, targets)
    return referee.make_range_descriptor(polar), referee.make_angle_descriptor(polar)


@pytest.mark.parametrize("name", ["000000.png", "000001.png", "000010.png"])
def test_matches_published_cpp(name):
    """The package reproduces the published C++ descriptors, off-by-one errors
    and all - they are on by default so the RA-L results stay reproducible."""
    reference = load_reference()
    if not (EXAMPLES / name).exists():
        pytest.skip(f"{name} not available")
    range_desc, angle_desc = describe(name)
    for kind, actual in (("R", range_desc), ("A", angle_desc)):
        expected = reference[(name, kind)]
        assert actual.shape == expected.shape
        assert np.max(np.abs(actual - expected)) / np.max(np.abs(expected)) < 1e-8


def test_fixed_window_differs_from_legacy():
    """The window fix is a real change, not a cosmetic one: band 0 holds one
    range bin in the published version and a full window in the fixed one."""
    if not (EXAMPLES / "000000.png").exists():
        pytest.skip("examples not available")
    scan = referee.load_polar_scan(str(EXAMPLES / "000000.png"), referee.NavtechModel.CIR204)
    polar = referee.targets_to_polar_image(
        scan.power.shape, referee.cen2018_features(scan.power))

    legacy = referee.make_range_descriptor(polar)
    fixed = referee.make_range_descriptor(
        polar, referee.RangeDescriptorParams(legacy_window_alignment=False))
    assert legacy[0] * 10 < fixed[0]          # one bin versus a full window
    assert referee.cosine_similarity(legacy, fixed) > 0.9


def test_yaw_is_recovered():
    """A rotated scan should come back with a non-zero yaw. The original demo
    never applied its shift, so this was always zero."""
    if not (EXAMPLES / "000000.png").exists():
        pytest.skip("examples not available")
    scan = referee.load_polar_scan(str(EXAMPLES / "000000.png"), referee.NavtechModel.CIR204)
    polar = referee.targets_to_polar_image(
        scan.power.shape, referee.cen2018_features(scan.power))

    angle = referee.make_angle_descriptor(polar)
    sectors = len(angle)
    shift = sectors // 4
    rotated = np.roll(polar, shift * (polar.shape[0] // sectors), axis=0)
    rotated_angle = referee.make_angle_descriptor(rotated)

    db = referee.AngleDescriptorDatabase()
    db.add(angle)
    yaw = db.yaw_difference(rotated_angle, 0)
    expected = 2.0 * np.pi * shift / sectors
    expected = expected - 2 * np.pi if expected > np.pi else expected
    assert abs(yaw - expected) < 2.0 * np.pi / sectors + 1e-9
