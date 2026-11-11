#include "referee/radar_utils.hpp"

#include <algorithm>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>

namespace referee {
namespace {

constexpr int kEncoderSize = 5600;
constexpr int kHeaderBytes = 11;
// The published implementation left the first 42 bins at zero; they sit inside
// the sensor's blind zone and carry only receiver ringing.
constexpr int kFirstUsableBin = 42;

int rangeBinsFor(NavtechModel model) {
  return model == NavtechModel::CIR204 ? 3360 : 3768;
}

}  // namespace

PolarScan loadPolarScan(const std::string& path, NavtechModel model) {
  const cv::Mat raw = cv::imread(path, cv::IMREAD_GRAYSCALE);
  if (raw.empty()) throw std::runtime_error("referee: cannot read " + path);

  const int num_azimuths = raw.rows;
  const int range_bins = rangeBinsFor(model);

  PolarScan scan;
  scan.timestamps.assign(num_azimuths, 0);
  scan.azimuths.assign(num_azimuths, 0.0);
  scan.valid.assign(num_azimuths, true);
  scan.power = cv::Mat::zeros(num_azimuths, range_bins, CV_32F);

  // The published implementation wrote `#pragma omp parallel` without `for`,
  // so every thread ran the whole loop and wrote the same values. Harmless but
  // pure waste; this is what was meant.
#pragma omp parallel for
  for (int i = 0; i < num_azimuths; ++i) {
    const uchar* row = raw.ptr<uchar>(i);
    scan.timestamps[i] = *reinterpret_cast<const std::int64_t*>(row);
    scan.azimuths[i] =
        *reinterpret_cast<const std::uint16_t*>(row + 8) * 2.0 * M_PI / kEncoderSize;
    scan.valid[i] = row[10] == 255;
    for (int bin = kFirstUsableBin; bin < range_bins; ++bin)
      scan.power.at<float>(i, bin) = static_cast<float>(row[kHeaderBytes + bin]) / 255.0f;
  }
  return scan;
}

Eigen::MatrixXd targetsToPolarImage(const cv::Mat& power, const Eigen::MatrixXd& targets) {
  Eigen::MatrixXd polar = Eigen::MatrixXd::Zero(power.rows, power.cols);
  for (int i = 0; i < targets.cols(); ++i)
    polar(static_cast<int>(targets(0, i)), static_cast<int>(targets(1, i))) = 1.0;
  return polar;
}

std::vector<std::string> radarFileNames(const std::string& directory,
                                        const std::string& extension) {
  namespace fs = std::filesystem;
  std::vector<std::string> files;
  for (const auto& entry : fs::directory_iterator(directory)) {
    if (!entry.is_regular_file()) continue;
    if (!extension.empty() && entry.path().extension() != "." + extension) continue;
    files.push_back(entry.path().filename().string());
  }
  // Filenames are epoch times, so numeric order is time order.
  std::sort(files.begin(), files.end(), [](const std::string& a, const std::string& b) {
    return std::stoll(a) < std::stoll(b);
  });
  return files;
}

}  // namespace referee
