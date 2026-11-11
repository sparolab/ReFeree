#pragma once
#include <Eigen/Dense>
#include <vector>
#include <opencv2/core.hpp>
#include "referee/radar_utils.hpp"

namespace referee {

/*!
   \brief Extract features from polar radar data using the method described in cen_icra18
   \param fft_data Polar radar power readings
   \param zq If y(i, j) > zq * sigma_q then it is considered a potential target point
   \param sigma_gauss std dev of the gaussian filter uesd to smooth the radar signal
   \param min_range We ignore the range bins less than this
   \param targets [out] Matrix of feature locations (azimuth_bin, range_bin, 1) x N
*/
/// Extract features from polar radar power using the method of Cen and Newman,
/// ICRA 2018.
/// \param power       polar radar power, (azimuth, range)
/// \param zq          a bin is a candidate target when y(i,j) > zq * sigma_q
/// \param sigma_gauss std dev of the gaussian used to smooth the signal
/// \param min_range   range bins below this are ignored
/// \param targets     [out] feature locations, (azimuth_bin, range_bin, 1) x N
/// \return time taken, seconds
double cen2018features(const cv::Mat& power, float zq, int sigma_gauss, int min_range,
                       Eigen::MatrixXd& targets);

}  // namespace referee
