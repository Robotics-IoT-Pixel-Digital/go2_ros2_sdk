cmake_minimum_required(VERSION 3.10)
project(unitree_go)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IMU.msg"

  "msg/BmsState.msg"
  "msg/MotorState.msg"
  "msg/TimeSpec.msg"
  "msg/SportModeState.msg"
  "msg/LowState.msg"
  "msg/VoxelMapCompressed.msg"
  "msg/Go2FrontVideoData.msg"
  
  DEPENDENCIES geometry_msgs
)

ament_package()
