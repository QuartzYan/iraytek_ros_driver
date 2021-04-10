cmake_minimum_required(VERSION 3.0.2)
project(iraytek_ros_driver)

## Compile as C++11, supported in ROS Kinetic and newer
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
add_compile_options(-DSYS_LINUX)
add_compile_options(-Wall)
add_compile_options(-DVS_TRANMIST -pthread)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

#find_package(Eigen3 3.1.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES iraytek_ros_driver
  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/iraytek_driver.cpp
  src/LinuxDef.cpp
  src/debug.cpp
  src/PtrArray.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/x64/libIRNetClient.so
)