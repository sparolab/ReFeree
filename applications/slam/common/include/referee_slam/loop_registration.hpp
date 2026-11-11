#pragma once
// Turning a loop proposal into a relative-pose constraint.
//
// The place lookup only says two keyframes look alike; this works out how they
// are actually related. The angular descriptor already gives the yaw, so the
// source cloud is pre-rotated by it and GICP only has to close the remaining
// gap - which is what lets a plain fitness threshold reject a wrong match
// instead of GICP quietly converging to one.

#include <optional>

#include <gtsam/geometry/Pose3.h>

#include "referee_slam/keyframe_store.hpp"

namespace referee_slam {

struct RegistrationParams {
  double max_correspondence_distance = 150.0;
  int max_iterations = 100;
  int num_threads = 8;
  double transformation_epsilon = 1e-6;
  double euclidean_fitness_epsilon = 1e-6;
  // A fixed low value is safe: a wrong match scores far above it.
  double fitness_threshold = 200.0;
  // Use the yaw from the angular descriptor as GICP's starting point.
  bool use_descriptor_yaw = true;

  // Largest relative translation a loop may claim, metres. Two keyframes at the
  // same place cannot be far apart, so this rejects a wrong match on geometry
  // alone, without touching the descriptor.
  //
  // The descriptor cannot do this job itself: cosine similarity between ReFeree
  // range descriptors sits in [0.999995, 1] whether or not the places match -
  // the ordering is informative, the absolute value is not - so a similarity
  // threshold rejects nothing. On MulRan KAIST_03, 44 per cent of accepted
  // loops were wrong until this gate was added; true loops measure 0.78 m at
  // the median and 2.58 m at p95, false ones 11.46 m and 34 m, and a 3 m cutoff
  // keeps 96 per cent of the true ones at 94 per cent precision.
  double max_translation = 3.0;
};

// The transform from the current keyframe to the previous one, or nothing when
// the fit is not good enough to trust.
std::optional<gtsam::Pose3> registerLoop(const KeyframeStore& store, const LoopProposal& proposal,
                                         const RegistrationParams& params);

}  // namespace referee_slam
