#!/usr/bin/env python

import rospy
import os
import sys
from datetime import datetime
from rospkg import RosPack
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
    global max_speed
    global car_position
    global car_orientation
    global track_position
    global logpath
    global logentry
    
    last_drive_message = None
    current_speed = 0
    max_speed = 0
    car_position = None
    car_orientation = 0
    track_position = 0
    logentry = 0

def getStatsPath():
    fullpath = RosPack().get_path("simulation_tools").split("/")
    fullpath.reverse()
    relativepath = "/stats"
    insert = False
    for part in fullpath:
        if part == "ros_ws":
            insert = True
        if insert:
            relativepath = "/" + part + relativepath
    return relativepath.replace("//","/")

def drive_param_callback(message):
    global last_drive_message 
    last_drive_message = message

 
def speed_callback(speed_message):
    global current_speed
    global max_speed
    current_speed = speed_message.wheel_speed
    # TODO: get real maximum speed from node meanwhile assume we're 10% slower than possible
    max_speed = current_speed * 1.1
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
    global max_speed
    global logpath
    global logentry
    
    # logfile handlers overall
    global logfile_handler_csv_speed;
    global logfile_handler_dat_speed;
    
    if(last_drive_message is not None):
        velocity = last_drive_message.velocity
        angle = last_drive_message.angle
    time = rospy.get_time()
    
    # now handle logging - start logging in round o
    
       
#    Path("/my/directory").mkdir(parents=True, exist_ok=True)
    
    # current overall file implementation
    #log_stats_file_path= os.path.join(RosPack().get_path("simulation_tools"), "log_stats.txt")
#    log_stats_file = open(log_stats_file_path, "a")
#    logfile_handler_csv_speed.write(str(time)+", "+str(track_position)+", "+str(car_position)+", "+str(velocity)+", "+str(angle)+", "+str(current_speed) + "\n")
    if logentry == 0:
        logbase = getStatsPath()
        dateTimeObj = datetime.now()    
        timestampStr = dateTimeObj.strftime("%Y%m%d-%H%M%S")
        logpath = logbase + "/" + timestampStr
        
        # create logpath for session if nonexistant
        try:
            os.makedirs(logbase)
        except:
            pass
        try:
            os.makedirs(logpath)
        except:
            pass
        
        # create handlers
        logfile_handler_csv_speed = open(logpath + "/speed_over_time.csv", "a")
        logfile_handler_dat_speed = open(logpath + "/speed_over_time.dat", "a")
        logfile_handler_csv_speed_angle = open(logpath + "/speed_angle_over_time.csv", "a")
        logfile_handler_dat_speed_angle = open(logpath + "/speed_angle_over_time.dat", "a")
        logfile_handler_csv_speed.write("Datapoint;Time;Speed;Maxspeed\n")
        logfile_handler_dat_speed.write("X t v vmax\n")
        logfile_handler_csv_speed_angle.write("Datapoint;Time;Speed;Maxspeed;Angle\n")
        logfile_handler_dat_speed_angle.write("X t v vmax angle\n")


    logfile_handler_csv_speed = open(logpath + "/speed_over_time.csv", "a")
    logfile_handler_dat_speed = open(logpath + "/speed_over_time.dat", "a")
    logfile_handler_csv_speed_angle = open(logpath + "/speed_angle_over_time.csv", "a")
    logfile_handler_dat_speed_angle = open(logpath + "/speed_angle_over_time.dat", "a")
        
    logstring_speed = str(logentry) + ";" + str('%.2f' % time)+ ";" + str('%.2f' % current_speed) + ";" + str('%.2f' % max_speed) + "\n"
    logfile_handler_csv_speed.write(logstring_speed)
    logfile_handler_dat_speed.write(logstring_speed.replace(";"," "))
    
    logstring_speed_angle = str(logentry) + ";" + str('%.2f' % time)+ ";" + str('%.2f' % current_speed) + ";" + str('%.2f' % max_speed) + ";" + str('%.2f' % angle) + "\n"
    logfile_handler_csv_speed_angle.write(logstring_speed_angle)
    logfile_handler_dat_speed_angle.write(logstring_speed_angle.replace(";"," "))
    
    logentry += 1


global logentry
logentry = 0

rospy.init_node('log_stats', anonymous=True)
rospy.Subscriber(TOPIC_GAZEBO_MODEL_STATE, ModelStates, on_model_state_callback)
rospy.Subscriber(TOPIC_GAZEBO_STATE_TELEMETRY, gazebo_state_telemetry, speed_callback)
rospy.Subscriber(TOPIC_DRIVE_PARAMETERS, drive_param, drive_param_callback)

while not rospy.is_shutdown():
    rospy.spin()
