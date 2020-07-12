cmake_minimum_required(VERSION 2.8.3)
project(wall_detection)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  drive_msgs
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  pcl_conversions
  pcl_ros
  geometric_math
)

## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra -Wchar-subscripts -Wchkp -Wdouble-promotion -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

#########################
## Dynamic Reconfigure ##
#########################

generate_dynamic_reconfigure_options(
  cfg/wall_detection.cfg
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS drive_msgs roscpp rospy std_msgs pcl_conversions pcl_ros
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
add_executable(wall_detection src/wall_detection.cpp)
target_link_libraries(wall_detection ${catkin_LIBRARIES})
add_dependencies(wall_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
