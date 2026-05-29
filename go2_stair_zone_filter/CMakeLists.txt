cmake_minimum_required(VERSION 3.10)
project(go2_stair_zone_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(stair_zone_filter SHARED
  src/stair_zone_filter.cpp
)
target_include_directories(stair_zone_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(stair_zone_filter PUBLIC cxx_std_17)

ament_target_dependencies(stair_zone_filter
  rclcpp nav2_costmap_2d nav2_util nav2_msgs pluginlib
  geometry_msgs nav_msgs tf2 tf2_ros
)

pluginlib_export_plugin_description_file(nav2_costmap_2d
  stair_zone_filter_plugin.xml
)

install(TARGETS stair_zone_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
