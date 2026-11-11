#include "referee_slam/graph_io.hpp"

#include <fstream>
#include <stdexcept>

namespace referee_slam {
namespace {

std::string poseFields(const gtsam::Pose3& pose) {
  const gtsam::Point3 t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  return std::to_string(t.x()) + " " + std::to_string(t.y()) + " " + std::to_string(t.z()) + " " +
         std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z()) + " " +
         std::to_string(q.w());
}

void writeKittiRow(std::ostream& out, const gtsam::Pose3& pose) {
  const gtsam::Point3 t = pose.translation();
  const gtsam::Rot3 R = pose.rotation();
  const auto c1 = R.column(1), c2 = R.column(2), c3 = R.column(3);
  out << c1.x() << " " << c2.x() << " " << c3.x() << " " << t.x() << " "
      << c1.y() << " " << c2.y() << " " << c3.y() << " " << t.y() << " "
      << c1.z() << " " << c2.z() << " " << c3.z() << " " << t.z() << "\n";
}

std::ofstream openOrThrow(const std::string& path) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("referee_slam: cannot write " + path);
  return out;
}

}  // namespace

std::string vertexLine(int node_index, const gtsam::Pose3& pose) {
  return "VERTEX_SE3:QUAT " + std::to_string(node_index) + " " + poseFields(pose);
}

std::string edgeLine(int from_index, int to_index, const gtsam::Pose3& relative_pose) {
  return "EDGE_SE3:QUAT " + std::to_string(from_index) + " " + std::to_string(to_index) + " " +
         poseFields(relative_pose);
}

void savePoseGraphG2o(const std::string& path, const std::vector<Pose6D>& poses,
                      const std::vector<std::string>& edges) {
  auto out = openOrThrow(path);
  for (std::size_t i = 0; i < poses.size(); ++i)
    out << vertexLine(static_cast<int>(i), toGtsam(poses[i])) << "\n";
  for (const auto& edge : edges) out << edge << "\n";
}

void saveVerticesKitti(const std::string& path, const std::vector<Pose6D>& poses) {
  auto out = openOrThrow(path);
  for (const auto& pose : poses) writeKittiRow(out, toGtsam(pose));
}

void saveVerticesKitti(const std::string& path, const gtsam::Values& estimates) {
  auto out = openOrThrow(path);
  for (const auto& key_value : estimates)
    writeKittiRow(out, estimates.at<gtsam::Pose3>(key_value.key));
}

}  // namespace referee_slam
