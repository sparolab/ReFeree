#pragma once
// Writing the pose graph out.
//
// The g2o file is what a multi-session optimiser reads back as one session, so
// its shape is an interface, not a debug dump: "VERTEX_SE3:QUAT idx x y z qx qy
// qz qw" for every keyframe, then one "EDGE_SE3:QUAT from to ..." per
// constraint. The KITTI files are the 3x4 pose matrices, for trajectory
// evaluation tools.

#include <string>
#include <vector>

#include <gtsam/nonlinear/Values.h>

#include "referee_slam/pose_types.hpp"

namespace referee_slam {

std::string vertexLine(int node_index, const gtsam::Pose3& pose);
std::string edgeLine(int from_index, int to_index, const gtsam::Pose3& relative_pose);

// Odometry poses as vertices, plus the edges accumulated so far.
void savePoseGraphG2o(const std::string& path, const std::vector<Pose6D>& poses,
                      const std::vector<std::string>& edges);

void saveVerticesKitti(const std::string& path, const std::vector<Pose6D>& poses);
void saveVerticesKitti(const std::string& path, const gtsam::Values& estimates);

}  // namespace referee_slam
