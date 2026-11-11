// Prints the ReFeree descriptors of polar radar scans, for the parity check with Python
// (python/tests/test_cpp.py).
//
//   referee_describe <scan.png>...   ->   "<file> R v0 v1 ..." and "<file> A v0 v1 ..." per scan
#include <cstdio>
#include <string>

#include "referee/descriptor.hpp"
#include "referee/features.hpp"
#include "referee/radar_utils.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: referee_describe <scan.png>...\n");
    return 2;
  }
  for (int i = 1; i < argc; ++i) {
    const auto scan = referee::loadPolarScan(argv[i], referee::NavtechModel::CIR204);
    Eigen::MatrixXd targets;
    referee::cen2018features(scan.power, 3.0f, 17, 58, targets);
    const Eigen::MatrixXd polar = referee::targetsToPolarImage(scan.power, targets);
    for (const auto& [kind, d] : {std::make_pair("R", referee::makeRangeDescriptor(polar)),
                                  std::make_pair("A", referee::makeAngleDescriptor(polar))}) {
      std::printf("%s %s", argv[i], kind);
      for (float v : d) std::printf(" %.9g", v);
      std::printf("\n");
    }
  }
  return 0;
}
