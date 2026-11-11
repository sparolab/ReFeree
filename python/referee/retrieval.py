"""Place lookup over ReFeree descriptors. Mirrors
``cpp/include/referee/retrieval.hpp``.

The range descriptor is rotation invariant, so finding a place is a similarity
search over it. The angular descriptor is not, and that is the point: the
circular shift that best aligns two angular descriptors is the yaw between them.

Thresholds the published implementation left as module-level globals are
parameters here, with the published values as defaults.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .descriptor import cosine_similarity, norm


@dataclass
class RetrievalParams:
    #: Entries at the end of the database to ignore, so a query does not match
    #: the frames it just came from. The published implementation used 0, which
    #: only makes sense when the query is not itself in the database.
    num_exclude_recent: int = 0
    #: Minimum cosine similarity to accept. The published implementation used 0,
    #: which accepts any candidate overlapping the query at all.
    similarity_threshold: float = 0.0


@dataclass
class LoopCandidate:
    index: int = -1  # -1 when nothing cleared the threshold
    similarity: float = 0.0

    def __bool__(self) -> bool:
        return self.index >= 0


class RangeDescriptorDatabase:
    """Database of rotation-invariant range descriptors."""

    def __init__(self, params: RetrievalParams | None = None) -> None:
        self.params = params or RetrievalParams()
        self._entries: list[np.ndarray] = []

    def add(self, desc: np.ndarray) -> int:
        self._entries.append(np.asarray(desc, dtype=np.float64).ravel())
        return len(self._entries) - 1

    def __len__(self) -> int:
        return len(self._entries)

    def at(self, index: int) -> np.ndarray:
        return self._entries[index]

    def detect_loop_closure(self, query: np.ndarray) -> LoopCandidate:
        """Most similar entry, or an empty candidate when none clears the
        threshold. Brute force: the descriptor is short enough that a tree costs
        more than it saves."""
        query = np.asarray(query, dtype=np.float64).ravel()
        if query.size == 0 or not self._entries:
            return LoopCandidate()

        searchable = len(self._entries) - self.params.num_exclude_recent
        if searchable <= 0:
            return LoopCandidate()

        scores = [cosine_similarity(query, self._entries[i]) for i in range(searchable)]
        best_index = int(np.argmax(scores))
        if scores[best_index] <= self.params.similarity_threshold:
            return LoopCandidate()
        return LoopCandidate(index=best_index, similarity=float(scores[best_index]))


def best_circular_shift(query: np.ndarray, reference: np.ndarray) -> int:
    """Circular shift of ``query`` that best aligns it onto ``reference``, in bins."""
    query = np.asarray(query, dtype=np.float64).ravel()
    reference = np.asarray(reference, dtype=np.float64).ravel()
    if query.shape != reference.shape or query.size == 0:
        return 0
    denom = norm(query) * norm(reference)
    if denom == 0.0:
        return 0
    scores = [float(np.roll(query, -shift) @ reference) / denom for shift in range(query.size)]
    return int(np.argmax(scores))


class AngleDescriptorDatabase:
    """Database of angular descriptors, used to recover yaw once a place matches."""

    def __init__(self) -> None:
        self._entries: list[np.ndarray] = []

    def add(self, desc: np.ndarray) -> int:
        self._entries.append(np.asarray(desc, dtype=np.float64).ravel())
        return len(self._entries) - 1

    def __len__(self) -> int:
        return len(self._entries)

    def at(self, index: int) -> np.ndarray:
        return self._entries[index]

    def yaw_difference(self, query: np.ndarray, index: int) -> float:
        """Yaw of ``query`` relative to entry ``index``, radians in (-pi, pi].
        Returns 0 for an invalid index, which is how a rejected loop is
        signalled."""
        if index < 0 or index >= len(self._entries):
            return 0.0
        query = np.asarray(query, dtype=np.float64).ravel()
        shift = best_circular_shift(query, self._entries[index])
        yaw = 2.0 * math.pi / query.size * shift
        return yaw - 2.0 * math.pi if yaw > math.pi else yaw
