cmake_minimum_required(VERSION 3.10)
project(referee_radar_slam)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unused-parameter -Wno-sign-compare")

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs nav_msgs sensor_msgs roscpp std_msgs message_generation image_transport cv_bridge tf nano_gicp)

add_message_files(FILES CurrReferee.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs message_runtime nano_gicp
               DEPENDS EIGEN3 PCL INCLUDE_DIRS include)

# ReFeree itself comes from the repository's ROS-free core library.
set(REFEREE_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../cpp" CACHE PATH "Path to the referee C++ core")
set(REFEREE_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory("${REFEREE_CORE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/referee_core" EXCLUDE_FROM_ALL)

include_directories(include third_party/nanoflann ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})

# Yeti radar odometry (Keenan Burnett), vendored
add_library(yeti_odometry_core src/features.cpp src/radar_utils.cpp src/association.cpp src/key_operation.cpp)
target_link_libraries(yeti_odometry_core ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen Threads::Threads
                      OpenMP::OpenMP_CXX)
add_dependencies(yeti_odometry_core ${PROJECT_NAME}_generate_messages_cpp)

add_executable(referee_odometry src/odometry.cpp)
target_link_libraries(referee_odometry yeti_odometry_core referee::referee ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(referee_odometry ${PROJECT_NAME}_generate_messages_cpp)

add_executable(referee_posegraph src/posegraph.cpp)
target_link_libraries(referee_posegraph referee::referee ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)
add_dependencies(referee_posegraph ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS referee_odometry referee_posegraph RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch rviz config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
