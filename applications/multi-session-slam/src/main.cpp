// Multi-session radar SLAM: aligns a query session to a central one (both written by the ReFeree SLAM)
// through inter-session ReFeree loops and anchor nodes. File in / file out; no ROS.
//
//   multi_session_node <config.yaml>
#include "ltslam/LTslam.h"
#include "ltslam/RosParamServer.h"
#include <chrono>
#include <iostream>

int main(int argc, char** argv)
{
    std::string config = (argc > 1) ? argv[1] : "config/kaist03_split.yaml";
    setLtslamConfigPath(config);

    std::cout << "\033[1;32m----> ReFeree multi-session starts. config=" << config << "\033[0m" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    LTslam multi_session;
    multi_session.run();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "\033[1;33mElapsed time: " << elapsed.count() << " sec\033[0m" << std::endl;
    std::cout << "\033[1;32m----> ReFeree multi-session done.\033[0m" << std::endl;
    return 0;
}
