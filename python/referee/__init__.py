"""ReFeree: a radar global descriptor built from features and free space.

This package mirrors the C++ core under ``cpp/`` module for module, and
``tests/`` checks the two against the descriptors the published implementation
produced. The C++ side is what the ROS 2 nodes use.
"""

from .descriptor import (
    AngleDescriptorParams,
    RangeDescriptorParams,
    cosine_similarity,
    free_space_per_range_bin,
    make_angle_descriptor,
    make_range_descriptor,
    norm,
)
from .features import cen2018_features
from .radar_utils import (
    NavtechModel,
    PolarScan,
    load_polar_scan,
    radar_file_names,
    targets_to_polar_image,
)
from .retrieval import (
    AngleDescriptorDatabase,
    LoopCandidate,
    RangeDescriptorDatabase,
    RetrievalParams,
    best_circular_shift,
)

__version__ = "2.0.0"

__all__ = [
    "__version__",
    "AngleDescriptorDatabase", "AngleDescriptorParams", "LoopCandidate", "NavtechModel",
    "PolarScan", "RangeDescriptorDatabase", "RangeDescriptorParams", "RetrievalParams",
    "best_circular_shift", "cen2018_features", "cosine_similarity",
    "free_space_per_range_bin", "load_polar_scan", "make_angle_descriptor",
    "make_range_descriptor", "norm", "radar_file_names", "targets_to_polar_image",
]
