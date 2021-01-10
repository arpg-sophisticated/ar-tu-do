cmake_minimum_required(VERSION 2.8.3)
project(avoid_static_obstacles)

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
)

## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchar-subscripts -Wchkp -Wdouble-promotion -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

#########################
## Dynamic Reconfigure ##
#########################



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
add_executable(avoid_static_obstacles src/avoid_static_obstacles.cpp)
target_link_libraries(avoid_static_obstacles ${catkin_LIBRARIES})
add_dependencies(avoid_static_obstacles ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
