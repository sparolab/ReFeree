#include "referee_slam/loop_registration.hpp"

#include <cmath>

#include <pcl/common/transforms.h>
#include <nano_gicp/nano_gicp.hpp>

namespace referee_slam {
namespace {

// The clouds are stored in the sensor frame, so a loop is registered between
// two local clouds and the result is a relative pose directly.
Cloud::Ptr rotatedAboutZ(const Cloud::Ptr& cloud, double yaw) {
  Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
  const float c = static_cast<float>(std::cos(yaw)), s = static_cast<float>(std::sin(yaw));
  rotation(0, 0) = c;  rotation(0, 1) = -s;
  rotation(1, 0) = s;  rotation(1, 1) = c;

  auto out = std::make_shared<Cloud>();
  pcl::transformPointCloud(*cloud, *out, rotation);
  return out;
}

}  // namespace

std::optional<gtsam::Pose3> registerLoop(const KeyframeStore& store, const LoopProposal& proposal,
                                         const RegistrationParams& params) {
  const Cloud::Ptr target = store.cloud(proposal.previous_index);
  Cloud::Ptr source = store.cloud(proposal.current_index);
  if (!target || !source || target->empty() || source->empty()) return std::nullopt;

  if (params.use_descriptor_yaw) source = rotatedAboutZ(source, proposal.yaw_difference);

  nano_gicp::NanoGICP<PointType, PointType> gicp;
  gicp.setMaxCorrespondenceDistance(params.max_correspondence_distance);
  gicp.setNumThreads(params.num_threads);
  gicp.setMaximumIterations(params.max_iterations);
  gicp.setTransformationEpsilon(params.transformation_epsilon);
  gicp.setEuclideanFitnessEpsilon(params.euclidean_fitness_epsilon);
  gicp.setRANSACIterations(0);

  gicp.setInputSource(source);
  gicp.calculateSourceCovariances();
  gicp.setInputTarget(target);
  gicp.calculateTargetCovariances();

  Cloud aligned;
  gicp.align(aligned);

  if (!gicp.hasConverged() || gicp.getFitnessScore() > params.fitness_threshold)
    return std::nullopt;

  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f correction(gicp.getFinalTransformation());
  pcl::getTranslationAndEulerAngles(correction, x, y, z, roll, pitch, yaw);

  if (params.max_translation > 0.0 &&
      std::hypot(x, y) > params.max_translation)
    return std::nullopt;

  const gtsam::Pose3 from(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
  return from.between(gtsam::Pose3());
}

}  // namespace referee_slam
