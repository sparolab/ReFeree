// Checks the descriptors against the output of the published implementation.
//
//   validate_descriptor <examples_dir> [reference.txt]
//
// The reference was produced by the code released with the RA-L paper, before
// this rewrite. Anything that changes these numbers changes published results,
// which is why the two known off-by-one errors are fixed behind flags rather
// than by default.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "referee/descriptor.hpp"
#include "referee/features.hpp"
#include "referee/radar_utils.hpp"

namespace {

// "<file> <R|A> v0 v1 ..." per line.
std::map<std::string, std::vector<float>> loadReference(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open " + path);
  std::map<std::string, std::vector<float>> reference;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::istringstream row(line);
    std::string name, kind;
    row >> name >> kind;
    std::vector<float> values;
    float v;
    while (row >> v) values.push_back(v);
    reference[name + " " + kind] = values;
  }
  return reference;
}

double worstRelative(const std::vector<float>& a, const std::vector<float>& b) {
  double peak = 0.0, worst = 0.0;
  for (float v : b) peak = std::max(peak, std::fabs(static_cast<double>(v)));
  for (std::size_t i = 0; i < a.size(); ++i)
    worst = std::max(worst, std::fabs(static_cast<double>(a[i]) - b[i]));
  return peak > 0.0 ? worst / peak : worst;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string examples = (argc > 1) ? argv[1] : "examples";
  const std::string reference_path =
      (argc > 2) ? argv[2] : "tools/reference/published_descriptors.txt";

  const auto reference = loadReference(reference_path);
  const auto files = referee::radarFileNames(examples);
  if (files.empty()) {
    std::cerr << "no radar scans under " << examples << "\n";
    return 1;
  }

  double worst = 0.0;
  int compared = 0;
  for (const auto& name : files) {
    const auto scan = referee::loadPolarScan(examples + "/" + name, referee::NavtechModel::CIR204);
    Eigen::MatrixXd targets;
    referee::cen2018features(scan.power, 3.0f, 17, 58, targets);
    const Eigen::MatrixXd polar = referee::targetsToPolarImage(scan.power, targets);

    const std::vector<std::pair<std::string, std::vector<float>>> actual = {
        {"R", referee::makeRangeDescriptor(polar)},
        {"A", referee::makeAngleDescriptor(polar)},
    };
    for (const auto& [kind, values] : actual) {
      const auto it = reference.find(name + " " + kind);
      if (it == reference.end()) continue;
      if (it->second.size() != values.size()) {
        std::cerr << name << " " << kind << ": length " << values.size() << " vs "
                  << it->second.size() << "\n";
        return 1;
      }
      worst = std::max(worst, worstRelative(values, it->second));
      ++compared;
    }
  }

  std::printf("descriptors compared : %d\n", compared);
  std::printf("max relative error   : %.3e\n", worst);
  const bool ok = compared > 0 && worst < 1e-6;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
