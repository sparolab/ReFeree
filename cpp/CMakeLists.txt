cmake_minimum_required(VERSION 3.16)
project(referee VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# The descriptors and the lookup are header-only; only the reader and the
# feature extractor need compiling. No ROS, no Boost, no PCL.
add_library(referee src/radar_utils.cpp src/features.cpp)
add_library(referee::referee ALIAS referee)
target_include_directories(referee PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(referee PUBLIC ${OpenCV_LIBS} Eigen3::Eigen)
set_target_properties(referee PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(OpenMP_CXX_FOUND)
  target_link_libraries(referee PUBLIC OpenMP::OpenMP_CXX)
endif()

option(REFEREE_BUILD_TOOLS "Build the validation/CLI tools and the tests" ON)
if(REFEREE_BUILD_TOOLS)
  add_executable(validate_descriptor tools/validate_descriptor.cpp)
  target_link_libraries(validate_descriptor PRIVATE referee)
  add_executable(referee_describe tools/describe.cpp)
  target_link_libraries(referee_describe PRIVATE referee)

  enable_testing()
  add_executable(test_referee tests/test_referee.cpp)
  target_link_libraries(test_referee PRIVATE referee)
  add_test(NAME published_descriptors
           COMMAND validate_descriptor ${CMAKE_CURRENT_SOURCE_DIR}/examples
                   ${CMAKE_CURRENT_SOURCE_DIR}/tools/reference/published_descriptors.txt)
  add_test(NAME retrieval COMMAND test_referee ${CMAKE_CURRENT_SOURCE_DIR}/examples)
endif()

include(GNUInstallDirs)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS referee EXPORT refereeTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(EXPORT refereeTargets NAMESPACE referee:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/referee)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/refereeConfig.cmake
  "include(CMakeFindDependencyMacro)\nfind_dependency(OpenCV)\nfind_dependency(Eigen3)\ninclude(\${CMAKE_CURRENT_LIST_DIR}/refereeTargets.cmake)\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/refereeConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/referee)
