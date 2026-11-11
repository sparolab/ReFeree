"""Feature extraction after Cen and Newman, ICRA 2018.

A faithful port of ``cpp/src/features.cpp``, which is the code the RA-L
results were produced with. Two of its details are surprising and are kept
deliberately, because changing either changes published numbers:

* the smoothing kernel is a parabola, not a Gaussian. The Gaussian line is
  present in the C++ but commented out, and ``(i - mu)**2 / sigma**2``
  normalised to sum one is what actually runs.
* OpenCV's ``filter2D`` correlates rather than convolves. The kernel is
  symmetric, so this happens not to matter, but the port correlates too.
"""

from __future__ import annotations

import numpy as np
from scipy.ndimage import correlate1d

#: Fallback noise estimate for an azimuth whose samples are all non-negative.
DEFAULT_SIGMA = 0.034


def _smoothing_kernel(sigma_gauss: int) -> np.ndarray:
    if sigma_gauss % 2 != 1:
        raise ValueError("sigma_gauss must be odd")
    size = sigma_gauss * 3
    mu = size // 2
    kernel = ((np.arange(size) - mu) ** 2) / float(sigma_gauss**2)
    return kernel / kernel.sum()


def cen2018_features(power: np.ndarray, zq: float = 3.0, sigma_gauss: int = 17,
                     min_range: int = 58) -> np.ndarray:
    """Feature locations as an ``(N, 2)`` array of ``(azimuth_bin, range_bin)``."""
    power = np.asarray(power, dtype=np.float32)

    # Estimate the bias per azimuth and subtract it.
    q = power - power.mean(axis=1, keepdims=True)

    # BORDER_REFLECT101 in OpenCV is "mirror" in scipy, not "reflect".
    p = correlate1d(q, _smoothing_kernel(sigma_gauss), axis=1, mode="mirror")

    # Noise standard deviation per azimuth, from the negative samples only.
    negative = q < 0
    counts = negative.sum(axis=1)
    sigma_q = np.where(
        counts > 0,
        np.sqrt(2.0 * np.where(negative, q.astype(np.float64) ** 2, 0.0).sum(axis=1)
                / np.maximum(counts, 1)),
        DEFAULT_SIGMA,
    )

    with np.errstate(divide="ignore", invalid="ignore"):
        nqp = np.exp(-0.5 * ((q - p) / sigma_q[:, None]) ** 2)
        npp = np.exp(-0.5 * (p / sigma_q[:, None]) ** 2)
    y = q * (1.0 - nqp) + p * (nqp - npp)

    above = y > (zq * sigma_q)[:, None]
    above[:, :min_range] = False

    # Each run of bins above the threshold contributes its middle bin.
    targets = []
    for azimuth in range(above.shape[0]):
        run_start = None
        for bin_index in range(min_range, above.shape[1]):
            if above[azimuth, bin_index]:
                if run_start is None:
                    run_start = bin_index
            elif run_start is not None:
                run = bin_index - run_start
                targets.append((azimuth, run_start + run // 2))
                run_start = None
        if run_start is not None:
            run = above.shape[1] - run_start
            targets.append((azimuth, run_start + run // 2))

    return np.array(targets, dtype=np.int64) if targets else np.zeros((0, 2), dtype=np.int64)
