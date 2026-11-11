# Multi-session radar SLAM (LT-SLAM / LT-mapper style): aligns a query session to a central one through
# inter-session ReFeree loops verified by nano-GICP. ROS-free: builds with plain CMake (any ROS 1 / ROS 2 image,
# or none) and, when ament is around, as a colcon package too.
#   cmake -S applications/multi-session-slam -B build/ms && cmake --build build/ms -j8
#   build/ms/multi_session_node applications/multi-session-slam/config/kaist03_split.yaml
cmake_minimum_required(VERSION 3.16)
project(referee_multi_session)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake QUIET)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP)

set(REFEREE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "ReFeree repository root")
if(NOT EXISTS "${REFEREE_ROOT}/cpp/CMakeLists.txt")
  message(FATAL_ERROR "referee core not found under ${REFEREE_ROOT}/cpp")
endif()
set(REFEREE_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory("${REFEREE_ROOT}/cpp" "${CMAKE_CURRENT_BINARY_DIR}/referee" EXCLUDE_FROM_ALL)
# the loop verification and graph files of the single-session back end
set(COMMON "${REFEREE_ROOT}/applications/slam/common")

add_library(referee_slam_core STATIC
  ${COMMON}/src/loop_registration.cpp ${COMMON}/src/graph_io.cpp ${COMMON}/src/nano_gicp.cc ${COMMON}/src/nanoflann.cc)
target_include_directories(referee_slam_core PUBLIC ${COMMON}/include ${PCL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(referee_slam_core PUBLIC referee::referee gtsam ${PCL_LIBRARIES} Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(referee_slam_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(multi_session_node
  src/main.cpp
  src/RosParamServer.cpp
  src/LTslam.cpp
  src/Session.cpp
  src/utility.cpp)
target_include_directories(multi_session_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(multi_session_node referee_slam_core ${OpenCV_LIBS})

if(ament_cmake_FOUND)
  install(TARGETS multi_session_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  ament_package()
else()
  install(TARGETS multi_session_node RUNTIME DESTINATION bin)
endif()
