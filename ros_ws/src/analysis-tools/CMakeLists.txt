cmake_minimum_required(VERSION 2.8.3)
project(analysis-tools)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  drive_msgs
  geometric_math
  std_msgs
  gazebo_msgs
  car_config
  vesc_sim
  geometric_math
)


## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchar-subscripts -Wchkp -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp drive_msgs std_msgs car_config vesc_sim geometric_math
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(log_stats   
  src/log_stats.cpp
)

target_link_libraries(log_stats ${catkin_LIBRARIES})
add_dependencies(log_stats ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare speed info node executable
add_executable(speed_info
  src/speed_info.cpp
  src/process_track.cpp
)
target_link_libraries(speed_info ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(speed_info ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})