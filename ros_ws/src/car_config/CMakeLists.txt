cmake_minimum_required(VERSION 2.8.3)
project(car_config)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamic_reconfigure
)

#########################
## Dynamic Reconfigure ##
#########################

generate_dynamic_reconfigure_options(
  cfg/dynamic_physical_properties.cfg
)

###################################
## catkin specific configuration ##
###################################

## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs dynamic_reconfigure
)

## Errors and Warnings
set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wchar-subscripts -Wchkp -Wdouble-promotion -Wformat -Wnonnull -Wmain -Wswitch-bool -Winvalid-memory-model -Wunknown-pragmas -Warray-bounds -Wfloat-equal -Wlogical-op -Wpacked ")
# -Wpedantic cant be used because of ROS

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_physical_properties   
  src/dynamic_physical_properties.cpp
)

target_link_libraries(dynamic_physical_properties ${catkin_LIBRARIES})
add_dependencies(dynamic_physical_properties ${PROJECT_NAME}_gencfg)
add_dependencies(dynamic_physical_properties ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(physical_properties
  src/physical_properties.cpp
)

target_link_libraries(physical_properties ${catkin_LIBRARIES})
add_dependencies(physical_properties ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_package(INCLUDE_DIRS include
               LIBRARIES physical_properties)

install(TARGETS physical_properties
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})