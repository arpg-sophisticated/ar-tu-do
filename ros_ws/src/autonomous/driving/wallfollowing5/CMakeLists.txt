cmake_minimum_required(VERSION 2.8.3)
project(wallfollowing5)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  drive_msgs
  roscpp
  rospy
  std_msgs
  car_config
  vesc_sim
  pcl_conversions
  pcl_ros
  geometric_math
  dynamic_reconfigure
  boxing
)


## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchar-subscripts -Wchkp -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

#########################
## Dynamic Reconfigure ##
#########################

generate_dynamic_reconfigure_options(
  cfg/wallfollowing.cfg
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS drive_msgs roscpp rospy std_msgs car_config vesc_sim pcl_conversions pcl_ros geometric_math boxing dynamic_reconfigure
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wallfollowing   
  src/wallfollowing.cpp
  src/speed_controller.cpp
  src/rviz_geometry.cpp
  src/pid_controller.cpp
  src/process_track.cpp
  src/steering_controller.cpp
)

target_link_libraries(wallfollowing ${catkin_LIBRARIES})
add_dependencies(wallfollowing ${PROJECT_NAME}_gencfg)
add_dependencies(wallfollowing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})