#pragma once
// Pose representations shared by the pose-graph node and its writers.

#include <cmath>

#include <gtsam/geometry/Pose3.h>

namespace referee_slam {

struct Pose6D {
  double x = 0.0, y = 0.0, z = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
};

inline gtsam::Pose3 toGtsam(const Pose6D& p) {
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
}

inline double translationBetween(const Pose6D& a, const Pose6D& b) {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

}  // namespace referee_slam
