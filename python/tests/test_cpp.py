"""The C++ core and this package give the same descriptors (needs the C++ tool
``referee_describe``: build cpp/ into cpp/build, or set REFEREE_CPP_DESCRIBE)."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))
import referee  # noqa: E402

TOOL = Path(os.environ.get("REFEREE_CPP_DESCRIBE", ROOT / "cpp/build/referee_describe"))
EXAMPLES = sorted((ROOT / "cpp/examples").glob("*.png"))


@pytest.mark.skipif(not TOOL.exists(), reason="C++ tool not built")
def test_cpp_matches_python():
    out = subprocess.run([str(TOOL), *map(str, EXAMPLES)], check=True, capture_output=True, text=True).stdout
    cpp = {}
    for line in out.splitlines():
        name, kind, *values = line.split()
        cpp[(Path(name).name, kind)] = np.array(values, dtype=np.float64)
    assert len(cpp) == 2 * len(EXAMPLES)
    for f in EXAMPLES:
        scan = referee.load_polar_scan(str(f), referee.NavtechModel.CIR204)
        polar = referee.targets_to_polar_image(scan.power.shape, referee.cen2018_features(scan.power))
        for kind, py in (("R", referee.make_range_descriptor(polar)), ("A", referee.make_angle_descriptor(polar))):
            c = cpp[(f.name, kind)]
            assert c.shape == py.shape
            assert np.max(np.abs(c - py)) / max(np.max(np.abs(c)), 1e-12) < 1e-6
