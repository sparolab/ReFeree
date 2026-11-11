// Radar odometry over a recorded sequence, publishing what the pose-graph node
// needs: the relative motion, the feature cloud, and the ReFeree descriptors.
//
// A ROS 2 port of yeti_radar_odometry's odometry.cpp. The estimation is
// unchanged - keypoints from Cen and Newman's detector, matched with ORB
// descriptors, then motion-distorted RANSAC with Doppler correction. What
// changed is the shape: parameters instead of hard-coded constants, a node
// instead of a bare main, and the work on its own thread so the executor stays
// responsive.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/features2d.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "referee/descriptor.hpp"
#include "referee_interfaces/msg/referee_descriptor.hpp"
#include "referee_odometry/association.hpp"
#include "referee_odometry/features.hpp"
#include "referee_odometry/radar_utils.hpp"

namespace {

using PointType = pcl::PointXYZI;

// The z of every feature is lifted off the ground plane so that downstream
// consumers which assume a 3-D cloud do not see a degenerate one.
constexpr float kFeatureHeight = 1.0f;

// Scan file names are epoch times, but the unit varies by dataset: Oxford
// writes microseconds, MulRan nanoseconds. The published implementation assumed
// microseconds unconditionally, which made every MulRan interval come out a
// thousand times too long - 249.6 s between two scans of a 4 Hz radar - and fed
// that straight into the motion-distortion and Doppler correction.
//
// The magnitude says which unit it is: a plausible epoch is ~1e9 s, ~1e12 ms,
// ~1e15 us or ~1e18 ns.
int64_t epochNanosecondsFromFileName(const std::string& name) {
  const int64_t value = std::stoll(name.substr(0, name.find('.')));
  if (value <= 0) return 0;
  for (int64_t scale : {1000000000LL, 1000000LL, 1000LL, 1LL}) {
    const int64_t seconds = value / (1000000000LL / scale);
    if (seconds > 1000000000LL && seconds < 4000000000LL) return value * scale;
  }
  return value;  // not an epoch at all, e.g. the 000000.png examples
}

// Seconds per unit of the per-azimuth header timestamps. They carry the same
// unit as the file names, and the motion estimator needs the two consistent:
// the published implementation hard-coded 1e-6 in both places, so on MulRan
// both were a thousand times out and the error cancelled.
double headerTimeScale(const std::vector<int64_t>& stamps) {
  if (stamps.empty() || stamps.front() <= 0) return 1.0e-6;
  const int64_t nanoseconds = epochNanosecondsFromFileName(std::to_string(stamps.front()));
  if (nanoseconds <= 0) return 1.0e-6;
  return 1.0e-9 * static_cast<double>(nanoseconds) / static_cast<double>(stamps.front());
}

}  // namespace

class RadarOdometryNode : public rclcpp::Node {
 public:
  RadarOdometryNode() : Node("referee_odometry") {
    sequence_dir_ = declare_parameter<std::string>("sequence_dir", "");
    scan_subdir_ = declare_parameter<std::string>("scan_subdir", "polar");
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "radar");

    min_range_ = declare_parameter<int>("min_range", 58);
    // Metres per range bin. Entangled with the encoder convention the azimuths
    // are read under, so it is left where the published code had it.
    radar_resolution_ = declare_parameter<double>("radar_resolution", 0.0432);
    cart_resolution_ = declare_parameter<double>("cart_resolution", 0.2592);
    cart_pixel_width_ = declare_parameter<int>("cart_pixel_width", 964);
    zq_ = declare_parameter<double>("cen2018_zq", 3.0);
    sigma_gauss_ = declare_parameter<int>("cen2018_sigma_gauss", 17);
    patch_size_ = declare_parameter<int>("orb_patch_size", 21);
    nndr_ = declare_parameter<double>("orb_nndr", 0.80);
    ransac_threshold_ = declare_parameter<double>("ransac_threshold", 0.35);
    inlier_ratio_ = declare_parameter<double>("ransac_inlier_ratio", 0.90);
    max_iterations_ = declare_parameter<int>("ransac_max_iterations", 100);
    max_gn_iterations_ = declare_parameter<int>("mdransac_max_gn_iterations", 10);
    doppler_beta_ = declare_parameter<double>("doppler_beta", 0.049);
    // The published implementation slept 100 ms per scan so the pose-graph node
    // could keep up. Kept, and adjustable.
    period_ms_ = declare_parameter<int>("scan_period_ms", 100);

    if (sequence_dir_.empty()) {
      RCLCPP_FATAL(get_logger(), "sequence_dir is required");
      throw std::runtime_error("referee_odometry: sequence_dir is required");
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    local_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_local", 100);
    global_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_global", 100);
    descriptor_pub_ =
        create_publisher<referee_interfaces::msg::RefereeDescriptor>("referee", 100);

    worker_ = std::thread(&RadarOdometryNode::run, this);
  }

  ~RadarOdometryNode() override {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

 private:
  void run() {
    const std::string scan_dir = sequence_dir_ + "/" + scan_subdir_;
    std::vector<std::string> scans;
    get_file_names(scan_dir, scans);
    if (scans.size() < 2) {
      RCLCPP_ERROR(get_logger(), "need at least two scans under %s", scan_dir.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "%zu scans under %s", scans.size(), scan_dir.c_str());

    auto detector = cv::ORB::create();
    detector->setPatchSize(patch_size_);
    detector->setEdgeThreshold(patch_size_);
    auto matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    Eigen::MatrixXd pose = Eigen::MatrixXd::Identity(4, 4);
    cv::Mat previous_image, current_image, previous_desc, current_desc;
    std::vector<cv::KeyPoint> previous_kp, current_kp;
    Eigen::MatrixXd previous_targets, current_targets;
    std::vector<int64_t> previous_times, current_times;

    for (std::size_t i = 0; i + 1 < scans.size() && running_ && rclcpp::ok(); ++i) {
      if (i > 0) {
        previous_times = current_times;
        previous_desc = current_desc.clone();
        previous_targets = current_targets;
        previous_kp = current_kp;
        current_image.copyTo(previous_image);
      }

      std::vector<int64_t> times;
      std::vector<double> azimuths;
      std::vector<bool> valid;
      cv::Mat power;
      load_radar(scan_dir + "/" + scans[i], times, azimuths, valid, power, CIR204);
      const std::vector<int64_t>& power_stamps = times;

      Eigen::MatrixXd targets;
      cen2018features(power, static_cast<float>(zq_), sigma_gauss_, min_range_, targets);
      const Eigen::MatrixXd polar = targets_to_polar_image(power, targets);

      const rclcpp::Time stamp = scanStamp(scans[i]);
      publishDescriptor(polar, stamp);

      radar_polar_to_cartesian(azimuths, power, static_cast<float>(radar_resolution_),
                               static_cast<float>(cart_resolution_), cart_pixel_width_, true,
                               current_image, CV_8UC1);
      polar_to_cartesian_points(azimuths, times, targets, static_cast<float>(radar_resolution_),
                                current_targets, current_times);
      const Eigen::MatrixXd feature_cloud = current_targets;
      convert_to_bev(current_targets, static_cast<float>(cart_resolution_), cart_pixel_width_,
                     patch_size_, current_kp, current_times);
      detector->compute(current_image, current_kp, current_desc);

      if (i == 0) continue;

      const Eigen::MatrixXd motion =
          estimateMotion(matcher, previous_desc, current_desc, previous_targets, current_targets,
                         previous_times, current_times,
                         (epochNanosecondsFromFileName(scans[i + 1]) -
                          epochNanosecondsFromFileName(scans[i])) / 1e9,
                         headerTimeScale(power_stamps),
                         static_cast<int>(i));
      pose = pose * motion;

      publishPose(pose, stamp);
      publishClouds(feature_cloud, pose, stamp);

      if (period_ms_ > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms_));
    }
    RCLCPP_INFO(get_logger(), "sequence finished");
  }

  // Motion-distorted RANSAC with Doppler correction, which is what the
  // published implementation used for its trajectory.
  Eigen::MatrixXd estimateMotion(const cv::Ptr<cv::DescriptorMatcher>& matcher,
                                 const cv::Mat& desc1, const cv::Mat& desc2,
                                 const Eigen::MatrixXd& targets1,
                                 const Eigen::MatrixXd& targets2,
                                 const std::vector<int64_t>& times1,
                                 const std::vector<int64_t>& times2, double delta_t,
                                 double time_scale, int seed) {
    std::vector<std::vector<cv::DMatch>> knn;
    matcher->knnMatch(desc1, desc2, knn, 2);

    std::vector<cv::DMatch> good;
    for (const auto& pair : knn)
      if (pair.size() == 2 && pair[0].distance < nndr_ * pair[1].distance) good.push_back(pair[0]);

    Eigen::MatrixXd p1 = Eigen::MatrixXd::Zero(2, good.size());
    Eigen::MatrixXd p2 = p1;
    std::vector<int64_t> t1(good.size()), t2(good.size());
    for (std::size_t j = 0; j < good.size(); ++j) {
      p1(0, j) = targets1(0, good[j].queryIdx);
      p1(1, j) = targets1(1, good[j].queryIdx);
      p2(0, j) = targets2(0, good[j].trainIdx);
      p2(1, j) = targets2(1, good[j].trainIdx);
      t1[j] = times1[good[j].queryIdx];
      t2[j] = times2[good[j].trainIdx];
    }

    MotionDistortedRansac mdransac(p2, p1, t2, t1, std::pow(ransac_threshold_, 2), inlier_ratio_,
                                   max_iterations_, time_scale);
    mdransac.setMaxGNIterations(max_gn_iterations_);
    mdransac.correctForDoppler(true);
    mdransac.setDopplerParameter(doppler_beta_);
    std::srand(seed);
    mdransac.computeModel();

    Eigen::MatrixXd transform = Eigen::MatrixXd::Zero(4, 4);
    mdransac.getTransform(delta_t, transform);
    return transform.inverse();
  }

  // The scans are replayed from disk, so their own epoch times are the only
  // stamps that mean anything downstream.
  static rclcpp::Time scanStamp(const std::string& file_name) {
    return rclcpp::Time(epochNanosecondsFromFileName(file_name));
  }

  void publishDescriptor(const Eigen::MatrixXd& polar, const rclcpp::Time& stamp) {
    referee_interfaces::msg::RefereeDescriptor msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = child_frame_id_;
    msg.range_descriptor = referee::makeRangeDescriptor(polar);
    msg.angle_descriptor = referee::makeAngleDescriptor(polar);
    descriptor_pub_->publish(msg);
  }

  void publishPose(const Eigen::MatrixXd& pose, const rclcpp::Time& stamp) {
    const Eigen::Matrix3d rotation = pose.block(0, 0, 3, 3);
    const double yaw = rotation.eulerAngles(0, 1, 2)(2);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = tf2::toMsg(q);
    odom_pub_->publish(odom);
  }

  void publishClouds(const Eigen::MatrixXd& features, const Eigen::MatrixXd& pose,
                     const rclcpp::Time& stamp) {
    pcl::PointCloud<PointType> local, global;
    local.reserve(features.cols());
    global.reserve(features.cols());

    for (int j = 0; j < features.cols(); ++j) {
      const double x = features(0, j), y = features(1, j), z = features(2, j) + kFeatureHeight;
      local.push_back(PointType{});
      local.back().x = static_cast<float>(x);
      local.back().y = static_cast<float>(y);
      local.back().z = static_cast<float>(z);

      global.push_back(PointType{});
      global.back().x = static_cast<float>(pose(0, 0) * x + pose(0, 1) * y + pose(0, 2) * z + pose(0, 3));
      global.back().y = static_cast<float>(pose(1, 0) * x + pose(1, 1) * y + pose(1, 2) * z + pose(1, 3));
      global.back().z = static_cast<float>(pose(2, 0) * x + pose(2, 1) * y + pose(2, 2) * z + pose(2, 3));
    }

    for (auto* entry : {&local, &global}) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*entry, msg);
      msg.header.stamp = stamp;
      msg.header.frame_id = (entry == &local) ? child_frame_id_ : frame_id_;
      (entry == &local ? local_cloud_pub_ : global_cloud_pub_)->publish(msg);
    }
  }

  std::string sequence_dir_, scan_subdir_, frame_id_, child_frame_id_;
  int min_range_, cart_pixel_width_, sigma_gauss_, patch_size_, max_iterations_,
      max_gn_iterations_, period_ms_;
  double radar_resolution_, cart_resolution_, zq_, nndr_, ransac_threshold_, inlier_ratio_,
      doppler_beta_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_cloud_pub_, global_cloud_pub_;
  rclcpp::Publisher<referee_interfaces::msg::RefereeDescriptor>::SharedPtr descriptor_pub_;

  std::atomic<bool> running_{true};
  std::thread worker_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
