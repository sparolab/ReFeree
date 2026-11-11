"""The ReFeree descriptors. Mirrors ``cpp/include/referee/descriptor.hpp``.

Both describe the free space around a place rather than the things in it: the
space a radar reports nothing in is as characteristic as what it reports, and is
far less affected by a false detection.

Both reproduce the published C++ bit for bit by default. Two off-by-one errors
were found in it; each is fixed behind a flag rather than silently, so results
in the RA-L paper stay reproducible.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RangeDescriptorParams:
    window: int = 80  # range bins per band

    #: The published implementation reset its accumulator at the *start* of each
    #: window rather than the end. Band 0 therefore held one range bin instead
    #: of ``window`` of them - two orders of magnitude smaller than every other
    #: band, so it contributed nothing to any distance - every other band was
    #: shifted by one bin, and the trailing ``window - 1`` bins were dropped.
    legacy_window_alignment: bool = True


@dataclass(frozen=True)
class AngleDescriptorParams:
    dims: int = 50  # angular sectors

    #: The published implementation scanned each ray from the far end down to
    #: bin 1, never examining bin 0.
    legacy_skip_nearest_bin: bool = True


def free_space_per_range_bin(polar: np.ndarray) -> np.ndarray:
    """Azimuths with no detection at each range bin."""
    return polar.shape[0] - polar.sum(axis=0)


def make_range_descriptor(polar: np.ndarray,
                          params: RangeDescriptorParams = RangeDescriptorParams()) -> np.ndarray:
    """Rotation-invariant: free space summed over each band of range bins."""
    if polar.size == 0:
        raise ValueError("empty polar image")
    free_space = free_space_per_range_bin(polar)
    num_bands = len(free_space) // params.window
    if num_bands <= 0:
        raise ValueError("fewer range bins than one window")

    desc = np.zeros(num_bands, dtype=np.float32)
    if params.legacy_window_alignment:
        acc = 0.0
        for bin_index, value in enumerate(free_space):
            acc += value
            if bin_index % params.window == 0:
                band = bin_index // params.window
                if band < num_bands:
                    desc[band] = acc
                acc = 0.0
    else:
        desc[:] = free_space[: num_bands * params.window].reshape(num_bands, params.window).sum(1)
    return desc


def make_angle_descriptor(polar: np.ndarray,
                          params: AngleDescriptorParams = AngleDescriptorParams()) -> np.ndarray:
    """Free space per angular sector, as a fraction of the sector's pixels.
    Counted from the outermost detection inwards, so everything beyond the last
    return counts as free. Not rotation invariant - that is what makes it useful
    for recovering yaw."""
    if polar.size == 0:
        raise ValueError("empty polar image")
    rows_per_sector = polar.shape[0] // params.dims
    range_bins = polar.shape[1]
    if rows_per_sector <= 0:
        raise ValueError("fewer azimuths than sectors")

    last_bin = 1 if params.legacy_skip_nearest_bin else 0
    sector_pixels = float(rows_per_sector * range_bins)

    desc = np.zeros(params.dims, dtype=np.float32)
    for sector in range(params.dims):
        rows = polar[sector * rows_per_sector : (sector + 1) * rows_per_sector, last_bin:]
        # Bins from the outermost detection inwards, per ray.
        has_return = rows != 0.0
        counts = np.where(has_return.any(axis=1),
                          rows.shape[1] - np.argmax(has_return[:, ::-1], axis=1),
                          0)
        desc[sector] = counts.sum() / sector_pixels
    return desc


def norm(desc: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(desc, dtype=np.float64)))


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    """Zero when either descriptor is empty or degenerate."""
    a = np.asarray(a, dtype=np.float64).ravel()
    b = np.asarray(b, dtype=np.float64).ravel()
    if a.shape != b.shape or a.size == 0:
        return 0.0
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na == 0.0 or nb == 0.0:
        return 0.0
    return float(a @ b / (na * nb))
