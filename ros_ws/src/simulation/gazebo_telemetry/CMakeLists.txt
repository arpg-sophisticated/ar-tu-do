cmake_minimum_required(VERSION 2.8.3)
project(gazebo_telemetry)

find_package(catkin REQUIRED COMPONENTS 
  rospy
  drive_msgs
  gazebo_msgs
)

## Generate services in the 'srv' folder
# add_service_files(
#  FILES  # e.g. Floats.srv HeaderString.srv
#)

catkin_package(
  CATKIN_DEPENDS
)