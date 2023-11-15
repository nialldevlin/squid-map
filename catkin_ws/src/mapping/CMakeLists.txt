cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_BUILD_TYPE Release)
project(mapping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
)

catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(transform_pointcloud_node src/transform_pointcloud.cpp)

## Add cmake target dependencies of the executable
add_dependencies(transform_pointcloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(transform_pointcloud_node
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS transform_pointcloud_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
