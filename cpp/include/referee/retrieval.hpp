#pragma once
// Place lookup over ReFeree descriptors.
//
// The range descriptor is rotation invariant, so finding a place is a plain
// similarity search over it. The angular descriptor is not, and that is the
// point: once a place has been matched, the circular shift that best aligns the
// two angular descriptors is the yaw between them.
//
// Thresholds that the published implementation left as file-scope globals are
// parameters here, with the published values as defaults.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "referee/descriptor.hpp"

namespace referee {

struct RetrievalParams {
  // Entries at the end of the database to ignore, so a query does not match the
  // frames it just came from. The published implementation used 0, which only
  // makes sense when the query is not itself in the database.
  int num_exclude_recent = 0;

  // Minimum cosine similarity to accept. The published implementation used 0,
  // which accepts any candidate that overlaps the query at all.
  double similarity_threshold = 0.0;
};

struct LoopCandidate {
  int index = -1;          // -1 when nothing cleared the threshold
  double similarity = 0.0;

  explicit operator bool() const { return index >= 0; }
};

// Database of rotation-invariant range descriptors.
class RangeDescriptorDatabase {
 public:
  explicit RangeDescriptorDatabase(RetrievalParams params = {}) : params_(params) {}

  int add(std::vector<float> desc) {
    entries_.push_back(std::move(desc));
    return static_cast<int>(entries_.size()) - 1;
  }

  std::size_t size() const { return entries_.size(); }
  bool empty() const { return entries_.empty(); }
  const std::vector<float>& at(int index) const {
    return entries_.at(static_cast<std::size_t>(index));
  }
  const RetrievalParams& params() const { return params_; }

  // Most similar entry to `query`, or an empty candidate when none clears the
  // threshold. Brute force: the descriptor is short enough that a tree costs
  // more than it saves.
  // `limit`, when positive, caps how many leading entries may be matched. The
  // caller usually knows better than a fixed count how much of the recent past
  // to exclude - how far the vehicle has travelled, say.
  LoopCandidate detectLoopClosure(const std::vector<float>& query, int limit = -1) const {
    LoopCandidate best;
    if (query.empty() || entries_.empty()) return best;

    int searchable = static_cast<int>(entries_.size()) - params_.num_exclude_recent;
    if (limit >= 0) searchable = std::min(searchable, limit);
    if (searchable <= 0) return best;

    double best_similarity = std::numeric_limits<double>::lowest();
    int best_index = -1;
    for (int i = 0; i < searchable; ++i) {
      const double similarity = cosineSimilarity(query, entries_[static_cast<std::size_t>(i)]);
      if (similarity > best_similarity) {
        best_similarity = similarity;
        best_index = i;
      }
    }

    if (best_index < 0 || best_similarity <= params_.similarity_threshold) return best;
    best.index = best_index;
    best.similarity = best_similarity;
    return best;
  }

 private:
  RetrievalParams params_;
  std::vector<std::vector<float>> entries_;
};

// Circular shift of `query` that best aligns it onto `reference`, in bins.
// Both descriptors must be the same length.
inline int bestCircularShift(const std::vector<float>& query,
                             const std::vector<float>& reference) {
  if (query.size() != reference.size() || query.empty()) return 0;
  const double denom = norm(query) * norm(reference);
  if (denom == 0.0) return 0;

  const std::size_t n = query.size();
  double best_score = 0.0;
  int best_shift = 0;
  for (std::size_t shift = 0; shift < n; ++shift) {
    double dot = 0.0;
    for (std::size_t i = 0; i < n; ++i)
      dot += static_cast<double>(query[(shift + i) % n]) * reference[i];
    if (dot == 0.0) continue;
    const double score = dot / denom;
    if (score > best_score) {
      best_score = score;
      best_shift = static_cast<int>(shift);
    }
  }
  return best_shift;
}

// Database of angular descriptors, used to recover yaw once a place matches.
class AngleDescriptorDatabase {
 public:
  int add(std::vector<float> desc) {
    entries_.push_back(std::move(desc));
    return static_cast<int>(entries_.size()) - 1;
  }

  std::size_t size() const { return entries_.size(); }
  bool empty() const { return entries_.empty(); }
  const std::vector<float>& at(int index) const {
    return entries_.at(static_cast<std::size_t>(index));
  }

  // Yaw of `query` relative to entry `index`, radians in (-pi, pi]. Returns 0
  // for an invalid index, which is how a rejected loop is signalled.
  double yawDifference(const std::vector<float>& query, int index) const {
    if (index < 0 || static_cast<std::size_t>(index) >= entries_.size()) return 0.0;
    const int shift = bestCircularShift(query, entries_[static_cast<std::size_t>(index)]);
    double yaw = 2.0 * M_PI / static_cast<double>(query.size()) * shift;
    if (yaw > M_PI) yaw -= 2.0 * M_PI;
    return yaw;
  }

 private:
  std::vector<std::vector<float>> entries_;
};

}  // namespace referee
