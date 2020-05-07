#!/usr/bin/env python

import rospy
import sys
from simulation_tools.track import track
from simulation_tools.reset_car import Point
from drive_msgs.msg import drive_param,gazebo_state_telemetry

from topics import TOPIC_GAZEBO_MODEL_STATE, TOPIC_GAZEBO_STATE_TELEMETRY,TOPIC_DRIVE_PARAMETERS


def __init__(self):
    self.last_drive_message = None
    self.current_speed = 0
    self.car_position = None
    self.car_orientation = 0
    self.track_position = 0

def drive_param_callback(self,message):
    self.last_drive_message = message

 
def speed_callback(self,speed_message):
    self.current_speed = speed_message.wheel_speed
    self.log_message()

def on_model_state_callback(self, message):
    if len(message.pose) < 2:
        return
    self.car_position = Point(
        message.pose[1].position.x,
        message.pose[1].position.y)
    self.car_orientation = message.pose[1].orientation
    self.track_position = track.localize(self.car_position)

def log_message(self):
    velocity = 0
    angle = 0
    if(self.last_drive_message is not None):
        velocity = self.last_drive_message.velocity
        angle = self.last_drive_message.angle
    time = rospy.get_time()
    with open("..\..\..\test.txt", "a") as myfile:
        myfile.write(str(time)+", "+str(self.track_position)+", "+str(self.car_position)+", "+str(velocity)+", "+str(angle)+", "+str(self.current_speed))




rospy.init_node('log_stats', anonymous=True)
rospy.Subscriber(TOPIC_GAZEBO_MODEL_STATE, ModelStates, self.on_model_state_callback)
rospy.Subscriber(TOPIC_GAZEBO_STATE_TELEMETRY, gazebo_state_telemetry, self.speed_callback)
rospy.Subscriber(TOPIC_DRIVE_PARAMETERS, drive_param, self.drive_param_callback)