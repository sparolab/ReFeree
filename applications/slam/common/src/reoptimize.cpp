// Re-solves a saved pose graph with different loop noise.
//
//   reoptimize <posegraph.g2o> <out_kitti.txt> [loop_noise_score] [cauchy_k]
//                                               [odom_rot_var] [odom_trans_var]
//
// The loop constraints are the measurements most likely to be wrong, so they
// carry a robust kernel. How hard that kernel bites depends on how large the
// residuals are, which depends on the drift being corrected - so the setting
// that works on a short, accurate trajectory can reject every loop on a long
// one. This re-solves without re-running the front end, so the two can be
// weighed against each other in seconds.
//
// Edges between consecutive indices are odometry; anything else is a loop.

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "referee_slam/graph_io.hpp"

namespace {

gtsam::Pose3 poseFrom(std::istringstream& in) {
  double x, y, z, qx, qy, qz, qw;
  in >> x >> y >> z >> qx >> qy >> qz >> qw;
  return gtsam::Pose3(gtsam::Rot3(gtsam::Quaternion(qw, qx, qy, qz)), gtsam::Point3(x, y, z));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <posegraph.g2o> <out_kitti.txt> [loop_noise_score] [cauchy_k]\n",
                 argv[0]);
    return 2;
  }
  const double loop_score = (argc > 3) ? std::stod(argv[3]) : 0.5;
  const double cauchy_k = (argc > 4) ? std::stod(argv[4]) : 1.0;
  const double odom_rot = (argc > 5) ? std::stod(argv[5]) : 1e-6;
  const double odom_trans = (argc > 6) ? std::stod(argv[6]) : 1e-4;

  std::ifstream file(argv[1]);
  if (!file) { std::fprintf(stderr, "cannot open %s\n", argv[1]); return 1; }

  const gtsam::SharedNoiseModel prior_noise =
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(1e-12));
  const gtsam::SharedNoiseModel odometry_noise = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << odom_rot, odom_rot, odom_rot,
                           odom_trans, odom_trans, odom_trans).finished());
  const gtsam::SharedNoiseModel loop_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(cauchy_k),
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(loop_score)));

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  int num_odometry = 0, num_loops = 0;

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream in(line);
    std::string tag;
    in >> tag;
    if (tag == "VERTEX_SE3:QUAT") {
      int index; in >> index;
      const gtsam::Pose3 pose = poseFrom(in);
      initial.insert(index, pose);
      if (index == 0) graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose, prior_noise));
    } else if (tag == "EDGE_SE3:QUAT") {
      int from, to; in >> from >> to;
      const gtsam::Pose3 relative = poseFrom(in);
      const bool is_odometry = (to == from + 1);
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(from, to, relative,
                                                   is_odometry ? odometry_noise : loop_noise));
      (is_odometry ? num_odometry : num_loops)++;
    }
  }

  std::printf("%zu poses, %d odom, %d loops | loop %.3g k %.3g | odom rot %.3g trans %.3g\n",
              initial.size(), num_odometry, num_loops, loop_score, cauchy_k, odom_rot, odom_trans);

  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(100);
  const gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();

  std::printf("error %.4g -> %.4g\n", graph.error(initial), graph.error(result));
  referee_slam::saveVerticesKitti(argv[2], result);
  return 0;
}
