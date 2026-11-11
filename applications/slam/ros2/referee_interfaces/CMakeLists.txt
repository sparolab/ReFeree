cmake_minimum_required(VERSION 3.8)
project(referee_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/RefereeDescriptor.msg
  DEPENDENCIES std_msgs
)

ament_package()
