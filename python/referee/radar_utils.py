"""Reading Navtech polar radar scans and turning detections back into a polar
image. Mirrors ``cpp/include/referee/radar_utils.hpp``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

ENCODER_SIZE = 5600
HEADER_BYTES = 11
#: The published implementation leaves the first 42 bins at zero; they sit
#: inside the sensor's blind zone and carry only receiver ringing.
FIRST_USABLE_BIN = 42


class NavtechModel(Enum):
    CTS350 = 3768  # range bins
    CIR204 = 3360  # MulRan


@dataclass
class PolarScan:
    timestamps: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.int64))
    azimuths: np.ndarray = field(default_factory=lambda: np.zeros(0))
    valid: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
    power: np.ndarray = field(default_factory=lambda: np.zeros((0, 0), dtype=np.float32))


def load_polar_scan(path: str, model: NavtechModel = NavtechModel.CTS350) -> PolarScan:
    """Read the Oxford-format polar png: one row per azimuth, the first eleven
    bytes carrying timestamp, encoder value and a validity flag."""
    if cv2 is None:
        raise ImportError("opencv-python is required to load radar scans")
    raw = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if raw is None:
        raise FileNotFoundError(f"cannot read {path}")

    range_bins = model.value
    timestamps = raw[:, :8].copy().view(np.int64).ravel()
    encoder = raw[:, 8:10].copy().view(np.uint16).ravel().astype(np.float64)

    power = np.zeros((raw.shape[0], range_bins), dtype=np.float32)
    usable = raw[:, HEADER_BYTES + FIRST_USABLE_BIN : HEADER_BYTES + range_bins]
    power[:, FIRST_USABLE_BIN : FIRST_USABLE_BIN + usable.shape[1]] = usable / 255.0

    return PolarScan(
        timestamps=timestamps,
        azimuths=encoder * 2.0 * math.pi / ENCODER_SIZE,
        valid=raw[:, 10] == 255,
        power=power,
    )


def targets_to_polar_image(power_shape, targets: np.ndarray) -> np.ndarray:
    """Detections back onto an (azimuth, range) grid: 1 where a feature was
    found. ``targets`` is the ``(azimuth_bin, range_bin)`` array the extractor
    returns."""
    polar = np.zeros(power_shape, dtype=np.float64)
    if targets.size:
        polar[targets[:, 0].astype(int), targets[:, 1].astype(int)] = 1.0
    return polar


def radar_file_names(directory: str, extension: str = "png") -> list[str]:
    """Radar scans in ``directory``, sorted by the epoch time in their name."""
    files = [p.name for p in Path(directory).iterdir()
             if p.is_file() and (not extension or p.suffix == "." + extension)]
    return sorted(files, key=lambda name: int(Path(name).stem))
