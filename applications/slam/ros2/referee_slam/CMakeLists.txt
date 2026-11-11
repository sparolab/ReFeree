cmake_minimum_required(VERSION 3.8)
project(referee_slam)

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
find_package(nano_gicp REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# The descriptor and the place lookup come from the repository's C++ core.
set(REFEREE_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../cpp"
    CACHE PATH "Path to the referee C++ core")
if(NOT EXISTS "${REFEREE_CORE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "referee core not found at ${REFEREE_CORE_DIR}")
endif()
add_subdirectory("${REFEREE_CORE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/referee_core" EXCLUDE_FROM_ALL)

add_executable(pose_graph_node
  src/pose_graph_node.cpp
  src/loop_registration.cpp
  src/graph_io.cpp)
target_include_directories(pose_graph_node PRIVATE
  include ${PCL_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR})
target_link_libraries(pose_graph_node referee gtsam ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(pose_graph_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs pcl_conversions
  referee_interfaces nano_gicp)

add_executable(reoptimize src/reoptimize.cpp src/graph_io.cpp)
target_include_directories(reoptimize PRIVATE include ${GTSAM_INCLUDE_DIR})
target_link_libraries(reoptimize gtsam Eigen3::Eigen)

install(TARGETS pose_graph_node reoptimize DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
foreach(dir launch config)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
