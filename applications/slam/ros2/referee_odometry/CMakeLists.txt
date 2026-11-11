cmake_minimum_required(VERSION 3.8)
project(referee_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(referee_interfaces REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenMP)

# The descriptor itself comes from the repository's C++ core, which knows
# nothing about ROS.
set(REFEREE_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../cpp"
    CACHE PATH "Path to the referee C++ core")
if(NOT EXISTS "${REFEREE_CORE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "referee core not found at ${REFEREE_CORE_DIR}")
endif()
add_subdirectory("${REFEREE_CORE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/referee_core" EXCLUDE_FROM_ALL)

# Estimation, vendored from yeti_radar_odometry. No ROS API inside.
add_library(referee_odometry_core
  src/radar_utils.cpp
  src/features.cpp
  src/association.cpp
  src/key_operation.cpp)
target_include_directories(referee_odometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(referee_odometry_core PUBLIC
  ${OpenCV_LIBS} ${PCL_LIBRARIES} Eigen3::Eigen Boost::system Boost::filesystem)
if(OpenMP_CXX_FOUND)
  target_link_libraries(referee_odometry_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node referee_odometry_core referee)
ament_target_dependencies(odometry_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs pcl_conversions
  referee_interfaces)

install(TARGETS odometry_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
