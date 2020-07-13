cmake_minimum_required(VERSION 2.8.3)
project(boxing)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  drive_msgs
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  dynamic_reconfigure
  debug_tools
)

## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -O2 -fno-omit-frame-pointer -Wall -Wextra -Wchar-subscripts -Wchkp -Wdouble-promotion -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

#########################
## Dynamic Reconfigure ##
#########################

generate_dynamic_reconfigure_options(
  cfg/boxing.cfg
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS drive_msgs roscpp rospy std_msgs pcl_conversions pcl_ros debug_tools
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# wall following node 
add_executable(boxing src/boxing.cpp src/rviz_geometry_publisher.cpp )
target_link_libraries(boxing ${catkin_LIBRARIES})
add_dependencies(boxing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
