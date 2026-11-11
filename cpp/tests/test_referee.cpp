// Retrieval and yaw recovery on the example scans.
//   test_referee <examples_dir>
#include <cmath>
#include <cstdio>
#include <string>

#include "referee/descriptor.hpp"
#include "referee/features.hpp"
#include "referee/radar_utils.hpp"
#include "referee/retrieval.hpp"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

static Eigen::MatrixXd describe(const cv::Mat& power) {
  Eigen::MatrixXd targets;
  referee::cen2018features(power, 3.0f, 17, 58, targets);
  return referee::targetsToPolarImage(power, targets);
}

int main(int argc, char** argv) {
  const std::string dir = argc > 1 ? argv[1] : "examples";
  referee::RangeDescriptorDatabase db;
  referee::AngleDescriptorDatabase yaw_db;
  std::vector<cv::Mat> powers;
  for (const char* name : {"000000.png", "000001.png", "000010.png"}) {
    const auto scan = referee::loadPolarScan(dir + "/" + name, referee::NavtechModel::CIR204);
    powers.push_back(scan.power);
    const auto polar = describe(scan.power);
    db.add(referee::makeRangeDescriptor(polar));
    yaw_db.add(referee::makeAngleDescriptor(polar));
  }
  // A scan matches itself, and the range descriptor ignores rotation.
  for (int i = 0; i < 3; ++i) {
    const auto c = db.detectLoopClosure(db.at(i));
    CHECK(c && c.index == i && std::abs(c.similarity - 1.0) < 1e-9);
  }
  // Rotate scan 2 by a quarter turn (azimuth rows): same place, and the yaw comes back.
  const cv::Mat& p = powers[2];
  const int rows = p.rows, quarter = rows / 4;
  cv::Mat rotated(p.size(), p.type());
  for (int r = 0; r < rows; ++r) p.row(r).copyTo(rotated.row((r + quarter) % rows));
  const auto polar = describe(rotated);
  const auto q = referee::makeRangeDescriptor(polar);
  const auto c = db.detectLoopClosure(q);
  CHECK(c && c.index == 2);
  const double yaw = std::abs(yaw_db.yawDifference(referee::makeAngleDescriptor(polar), 2));
  std::printf("recovered |yaw| %.1f deg for a 90 deg rotation\n", yaw * 180 / M_PI);
  CHECK(std::abs(yaw - M_PI / 2) < 10 * M_PI / 180);
  std::printf(failures ? "%d failures\n" : "all passed\n", failures);
  return failures ? 1 : 0;
}
