#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param


TOPIC_LASER_SCAN = "/scan"
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"

last_speed = 0



def laser_callback(scan_message):
    global last_scan_message = scan_message

def laser_callback(drive_message):
    global last_drive_message = drive_message


rospy.init_node('store_training_data', anonymous=True)

rospy.Subscriber(TOPIC_LASER_SCAN, LaserScan, laser_callback)
rospy.Subscriber(TOPIC_DRIVE_PARAMETERS, drive_param, drive_callback)

while not rospy.is_shutdown():
    rospy.spin()
