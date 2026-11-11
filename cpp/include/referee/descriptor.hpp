#pragma once
// The ReFeree descriptors.
//
// Both are built from a binary polar image - one row per azimuth, one column
// per range bin, set where a feature was detected - and both describe the free
// space around that: the space a radar reports nothing in is as characteristic
// of a place as the things it reports, and is far less affected by a false
// detection.
//
//   RangeDescriptor  free space per range band, summed over azimuth. Rotating
//                    the sensor permutes azimuths without changing the sum, so
//                    this is rotation invariant.
//   AngleDescriptor  free space per angular sector, which is not rotation
//                    invariant and is therefore what recovers the yaw between
//                    two places once they have been matched.
//
// Both reproduce the published implementation bit for bit by default. Two
// off-by-one errors were found in it; each is fixed behind a flag rather than
// silently, so that results in the RA-L paper stay reproducible. See
// RangeDescriptorParams::legacy_window_alignment and
// AngleDescriptorParams::legacy_skip_nearest_bin.

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace referee {

// One row per azimuth, one column per range bin; 1 where a feature was detected.
using PolarFeatureImage = Eigen::MatrixXd;

struct RangeDescriptorParams {
  int window = 80;  // range bins per band

  // The published implementation reset its accumulator at the *start* of each
  // window rather than the end. Band 0 therefore held one range bin instead of
  // `window` of them - two orders of magnitude smaller than every other band,
  // so it contributed nothing to any distance - every other band was shifted by
  // one bin, and the trailing `window - 1` bins were dropped entirely.
  //
  // On by default: the published results were produced with it.
  bool legacy_window_alignment = true;
};

struct AngleDescriptorParams {
  int dims = 50;  // angular sectors

  // The published implementation scanned each ray from the far end down to bin
  // 1, never examining bin 0. Harmless in practice, since the nearest bin is
  // inside the sensor's blind zone, but it makes the normalising pixel count
  // disagree with the number of bins actually visited.
  bool legacy_skip_nearest_bin = true;
};

// Free space per range bin: azimuths with no detection at that range.
inline Eigen::VectorXd freeSpacePerRangeBin(const PolarFeatureImage& polar) {
  Eigen::VectorXd free_space(polar.cols());
  for (int bin = 0; bin < polar.cols(); ++bin)
    free_space(bin) = polar.rows() - polar.col(bin).sum();
  return free_space;
}

// Rotation-invariant descriptor: free space summed over each band of range bins.
inline std::vector<float> makeRangeDescriptor(const PolarFeatureImage& polar,
                                              const RangeDescriptorParams& p = {}) {
  if (polar.size() == 0) throw std::invalid_argument("referee: empty polar image");
  if (p.window <= 0) throw std::invalid_argument("referee: window must be > 0");

  const Eigen::VectorXd free_space = freeSpacePerRangeBin(polar);
  const int num_bands = static_cast<int>(free_space.size()) / p.window;
  if (num_bands <= 0) throw std::invalid_argument("referee: fewer range bins than one window");

  std::vector<float> desc(static_cast<std::size_t>(num_bands), 0.0f);

  if (p.legacy_window_alignment) {
    double acc = 0.0;
    for (int bin = 0; bin < free_space.size(); ++bin) {
      acc += free_space(bin);
      if (bin % p.window == 0) {
        const int band = bin / p.window;
        if (band < num_bands) desc[static_cast<std::size_t>(band)] = static_cast<float>(acc);
        acc = 0.0;
      }
    }
  } else {
    for (int band = 0; band < num_bands; ++band)
      desc[static_cast<std::size_t>(band)] = static_cast<float>(
          free_space.segment(band * p.window, p.window).sum());
  }
  return desc;
}

// Free space per angular sector, as a fraction of the sector's pixels. Free
// space is counted from the outermost detection inwards, so everything beyond
// the last return counts as free.
inline std::vector<float> makeAngleDescriptor(const PolarFeatureImage& polar,
                                              const AngleDescriptorParams& p = {}) {
  if (polar.size() == 0) throw std::invalid_argument("referee: empty polar image");
  if (p.dims <= 0) throw std::invalid_argument("referee: dims must be > 0");

  const int rows_per_sector = static_cast<int>(polar.rows()) / p.dims;
  const int range_bins = static_cast<int>(polar.cols());
  if (rows_per_sector <= 0) throw std::invalid_argument("referee: fewer azimuths than sectors");

  const int last_bin = p.legacy_skip_nearest_bin ? 1 : 0;
  const double sector_pixels = static_cast<double>(rows_per_sector) * range_bins;

  std::vector<float> desc(static_cast<std::size_t>(p.dims), 0.0f);
  for (int sector = 0; sector < p.dims; ++sector) {
    double free_space = 0.0;
    for (int row = 0; row < rows_per_sector; ++row) {
      const auto ray = polar.row(sector * rows_per_sector + row);
      bool past_farthest_return = false;
      for (int bin = range_bins - 1; bin >= last_bin; --bin) {
        if (!past_farthest_return && ray(bin) != 0.0) past_farthest_return = true;
        if (past_farthest_return) free_space += 1.0;
      }
    }
    desc[static_cast<std::size_t>(sector)] = static_cast<float>(free_space / sector_pixels);
  }
  return desc;
}

inline double norm(const std::vector<float>& desc) {
  double sum = 0.0;
  for (float v : desc) sum += static_cast<double>(v) * v;
  return std::sqrt(sum);
}

// Cosine similarity; zero when either descriptor is empty or degenerate.
inline double cosineSimilarity(const std::vector<float>& a, const std::vector<float>& b) {
  if (a.size() != b.size() || a.empty()) return 0.0;
  double dot = 0.0, na = 0.0, nb = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    dot += static_cast<double>(a[i]) * b[i];
    na += static_cast<double>(a[i]) * a[i];
    nb += static_cast<double>(b[i]) * b[i];
  }
  if (na == 0.0 || nb == 0.0) return 0.0;
  return dot / (std::sqrt(na) * std::sqrt(nb));
}

}  // namespace referee
