#pragma once

// De-ROS'd config server: same interface as the original RosParamServer but
// reads parameters from a flat YAML file (set via setConfigPath()) instead of
// the ROS parameter server. LTslam inherits this, so the class name is kept.

#include "ltslam/utility.h"
#include <string>

// set by main() from argv before constructing LTslam
void setLtslamConfigPath(const std::string& path);

class RosParamServer
{
public:
    std::string sessions_dir_;
    std::string central_sess_name_;
    std::string query_sess_name_;
    std::string save_directory_;

    bool is_display_debug_msgs_;

    int kNumSCLoopsUpperBound;
    int kNumRSLoopsUpperBound;
    int numberOfCores;

    float loopFitnessScoreThreshold;  // nano-GICP fitness, as the single-session back end (200)
    float loopMaxTranslation;         // residual translation after GICP, metres (3)

public:
    RosParamServer();
};
