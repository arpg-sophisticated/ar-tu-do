#!/usr/bin/env python

import rospy
import sys
from simulation_tools.track import track
from simulation_tools.reset_car import Point
from drive_msgs.msg import drive_param,gazebo_state_telemetry
from gazebo_msgs.msg import ModelStates

TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_GAZEBO_MODEL_STATE = "/gazebo/model_states"
TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry"

def __init__(self):
    global last_drive_message
    global current_speed
    global car_position
    global car_orientation
    global track_position
    last_drive_message = None
    current_speed = 0
    car_position = None
    car_orientation = 0
    track_position = 0

def drive_param_callback(message):
    global last_drive_message 
    last_drive_message = message

 
def speed_callback(speed_message):
    print("speed")
    global current_speed
    current_speed = speed_message.wheel_speed
    log_message()

def on_model_state_callback(message):
    if len(message.pose) < 2:
        return
    global car_position
    global car_orientation
    global track_position
    car_position = Point(
        message.pose[1].position.x,
        message.pose[1].position.y)
    car_orientation = message.pose[1].orientation
    track_position = track.localize(car_position)

def log_message():
    velocity = 0
    angle = 0
    global last_drive_message
    global track_position
    global car_position
    global current_speed
    if(last_drive_message is not None):
        velocity = last_drive_message.velocity
        angle = last_drive_message.angle
    time = rospy.get_time()
    print("log stats")
    log_stats_file = open("/home/marvin/code/ar-tu-do/ros_ws/log_stats.txt", "a")
    log_stats_file.write(str(time)+", "+str(track_position)+", "+str(car_position)+", "+str(velocity)+", "+str(angle)+", "+str(current_speed))

rospy.init_node('log_stats', anonymous=True)
rospy.Subscriber(TOPIC_GAZEBO_MODEL_STATE, ModelStates, on_model_state_callback)
rospy.Subscriber(TOPIC_GAZEBO_STATE_TELEMETRY, gazebo_state_telemetry, speed_callback)
rospy.Subscriber(TOPIC_DRIVE_PARAMETERS, drive_param, drive_param_callback)

while not rospy.is_shutdown():
    rospy.spin()