#pragma once
// Everything the pose-graph node knows about the keyframes it has seen.
//
// The published implementation kept this as a dozen file-scope vectors guarded
// by three separate mutexes, with the invariant that they all stay the same
// length left to the reader. Here they are one object behind one lock, and the
// invariant is enforced when a keyframe is added.

#include <mutex>
#include <optional>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "referee/descriptor.hpp"
#include "referee/retrieval.hpp"
#include "referee_slam/pose_types.hpp"

namespace referee_slam {

using PointType = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointType>;

struct LoopProposal {
  int previous_index = -1;
  int current_index = -1;
  double yaw_difference = 0.0;
};

class KeyframeStore {
 public:
  explicit KeyframeStore(referee::RetrievalParams params = {}) : range_db_(params) {}

  // Must be called before the first keyframe is added.
  void setRetrievalParams(const referee::RetrievalParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    range_db_ = referee::RangeDescriptorDatabase(params);
  }

  // Adds a keyframe and its descriptors together, so the pose, the cloud and
  // the two descriptor databases cannot fall out of step.
  int add(const Pose6D& pose, double stamp, Cloud::Ptr cloud,
          std::vector<float> range_descriptor, std::vector<float> angle_descriptor) {
    std::lock_guard<std::mutex> lock(mutex_);
    travelled_.push_back(poses_.empty() ? 0.0
                                        : travelled_.back() + translationBetween(poses_.back(), pose));
    poses_.push_back(pose);
    optimised_poses_.push_back(pose);
    stamps_.push_back(stamp);
    clouds_.push_back(std::move(cloud));
    range_descriptors_.push_back(range_descriptor);
    angle_descriptors_.push_back(angle_descriptor);
    range_db_.add(std::move(range_descriptor));
    angle_db_.add(std::move(angle_descriptor));
    return static_cast<int>(poses_.size()) - 1;
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return poses_.size();
  }

  Pose6D pose(int index) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return poses_.at(static_cast<std::size_t>(index));
  }
  Pose6D optimisedPose(int index) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return optimised_poses_.at(static_cast<std::size_t>(index));
  }
  Cloud::Ptr cloud(int index) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return clouds_.at(static_cast<std::size_t>(index));
  }
  double stamp(int index) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stamps_.at(static_cast<std::size_t>(index));
  }

  std::vector<Pose6D> poses() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return poses_;
  }
  std::vector<Pose6D> optimisedPoses() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return optimised_poses_;
  }

  void setOptimisedPose(int index, const Pose6D& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    optimised_poses_.at(static_cast<std::size_t>(index)) = pose;
  }

  // How far the vehicle must have travelled since a keyframe before it may be
  // proposed as a loop. This is a distance, not a count: excluding a fixed
  // number of keyframes means excluding whatever distance the keyframe spacing
  // happens to give, so the same setting rejects real revisits at one spacing
  // and accepts the road just travelled at another.
  void setMinLoopTravel(double metres) {
    std::lock_guard<std::mutex> lock(mutex_);
    min_loop_travel_ = metres;
  }

  // The most recent keyframe against the earlier ones. Returns nothing when no
  // candidate clears the threshold.
  std::optional<LoopProposal> proposeLoop() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (poses_.empty()) return std::nullopt;
    const int current = static_cast<int>(poses_.size()) - 1;

    // Candidates are the keyframes at least min_loop_travel_ back along the path.
    int eligible = current;
    while (eligible > 0 && travelled_[current] - travelled_[eligible - 1] < min_loop_travel_)
      --eligible;
    if (eligible <= 0) return std::nullopt;

    const auto candidate = range_db_.detectLoopClosure(range_descriptors_.at(current), eligible);
    if (!candidate) return std::nullopt;

    return LoopProposal{candidate.index, current,
                        angle_db_.yawDifference(angle_descriptors_.at(current), candidate.index)};
  }

 private:
  mutable std::mutex mutex_;
  std::vector<Pose6D> poses_, optimised_poses_;
  std::vector<double> stamps_;
  std::vector<double> travelled_;   // cumulative path length, one per keyframe
  double min_loop_travel_ = 100.0;  // metres
  std::vector<Cloud::Ptr> clouds_;
  std::vector<std::vector<float>> range_descriptors_, angle_descriptors_;
  referee::RangeDescriptorDatabase range_db_;
  referee::AngleDescriptorDatabase angle_db_;
};

}  // namespace referee_slam
