#pragma once
// Reading Navtech polar radar scans and turning detections back into a polar
// image.

#include <Eigen/Dense>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace referee {

// Range-bin count differs by sensor generation.
enum class NavtechModel {
  CTS350,  // 3768 range bins
  CIR204,  // 3360 range bins - MulRan
};

struct PolarScan {
  std::vector<std::int64_t> timestamps;  // one per azimuth, microseconds
  std::vector<double> azimuths;          // one per azimuth, radians
  std::vector<bool> valid;               // one per azimuth
  cv::Mat power;                         // CV_32F, (azimuth, range), 0..1
};

// Reads the Oxford-format polar png: each row is one azimuth, the first eleven
// bytes carrying timestamp, encoder value and a validity flag.
PolarScan loadPolarScan(const std::string& path, NavtechModel model = NavtechModel::CTS350);

// Detections back onto an (azimuth, range) grid: 1 where a feature was found.
// `targets` is the (azimuth_bin, range_bin, 1) x N matrix the extractor returns.
Eigen::MatrixXd targetsToPolarImage(const cv::Mat& power, const Eigen::MatrixXd& targets);

// Radar scans in `directory`, sorted by the epoch time in their filename.
std::vector<std::string> radarFileNames(const std::string& directory,
                                        const std::string& extension = "png");

}  // namespace referee
