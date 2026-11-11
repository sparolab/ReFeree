#pragma once
// Inter-session loop proposal backed by the ReFeree descriptors.
//
// Takes the place of LT-SLAM's Scan Context lookup: "which node of the central session does this query node
// look like?". The range descriptor (rotation invariant) finds the place, by cosine similarity over all
// central nodes, exactly as the single-session back end does (referee::RangeDescriptorDatabase); the angle
// descriptor gives the yaw between the two, which seeds the nano-GICP verification. The similarity itself
// says little (ReFeree cosines sit near 1 whether or not the places match), so the geometric check after it is
// what accepts or rejects the loop.

#include <cstddef>
#include <vector>

#include "referee/retrieval.hpp"

struct RefereeMatch {
  int index = -1;          // central-session node, -1 if none
  double yaw = 0.0;        // query relative to that node, from the angle descriptors (rad)
  double similarity = 0.0;
};

class RefereeIndex {
 public:
  // Nodes must be added in node-index order: the position in the database is the node index.
  void add(std::vector<float> range_descriptor, std::vector<float> angle_descriptor) {
    range_.push_back(range_descriptor);
    angle_.push_back(angle_descriptor);
    range_db_.add(std::move(range_descriptor));
    angle_db_.add(std::move(angle_descriptor));
  }

  std::size_t size() const { return range_.size(); }
  const std::vector<float>& rangeDescriptor(int i) const { return range_.at(static_cast<std::size_t>(i)); }
  const std::vector<float>& angleDescriptor(int i) const { return angle_.at(static_cast<std::size_t>(i)); }

  RefereeMatch match(const std::vector<float>& range_query, const std::vector<float>& angle_query) const {
    RefereeMatch m;
    const auto c = range_db_.detectLoopClosure(range_query);
    if (!c) return m;
    m.index = c.index;
    m.similarity = c.similarity;
    m.yaw = angle_db_.yawDifference(angle_query, c.index);
    return m;
  }

 private:
  referee::RangeDescriptorDatabase range_db_{};
  referee::AngleDescriptorDatabase angle_db_{};
  std::vector<std::vector<float>> range_, angle_;
};
