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
    global logfile_handler_csv;
    global logfile_handler_dat;
    
    global last_drive_message
    global car_position
    global car_orientation
    global track_position
    global logpath
    global logentry
    
    global speed_current
    global speed_last
    global speed_delta
    global speed_avg
    
    global maxspeed_current
    global maxspeed_last
    global maxspeed_delta
    global maxspeed_avg
    
    global time_current
    global time_last
    global time_delta
    
    global distance_current
    global distance_last
    global distance_delta
    
    global acceleration_current
    global acceleration_last
    global acceleration_delta
    
    global angle_current
    global angle_last
    global angle_delta
    
    last_drive_message = None
    car_position = None
    car_orientation = 0
    track_position = 0
    logentry = 0
    
    speed_current = 0
    speed_last = 0
    speed_delta = 0
    speed_avg = 0
    
    maxspeed_current = 0
    maxspeed_last = 0
    maxspeed_delta = 0
    maxspeed_avg = 0
    
    time_current = 0
    time_last = 0
    time_delta = 0
    
    distance_current = 0
    distance_last = 0
    distance_delta = 0
    
    acceleration_current = 0
    acceleration_last = 0
    acceleration_delta = 0
    
    angle_current = 0
    angle_last = 0
    angle_delta = 0
    
    
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
    global speed_current
    global speed_last
    global speed_delta
    global speed_avg
    
    speed_last = speed_current
    speed_current = speed_message.wheel_speed
    speed_delta = speed_current - speed_last
    if logentry > 0:
        speed_avg = ( speed_avg * ( logentry - 1 ) + speed_current ) / logentry
    else:
        speed_avg += speed_current
            
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
    global last_drive_message
    global track_position
    global car_position
    global logpath
    global logentry
    
    global speed_current
    global speed_last
    global speed_delta
    global speed_avg
    
    global maxspeed_current
    global maxspeed_last
    global maxspeed_delta
    global maxspeed_avg
    
    global time_current
    global time_last
    global time_delta
    
    global distance_current
    global distance_last
    global distance_delta
    
    global acceleration_current
    global acceleration_last
    global acceleration_delta
    
    global angle_current
    global angle_last
    global angle_delta
    
    global logfile_handler_csv;
    global logfile_handler_dat;
    
    if(last_drive_message is not None):
        maxspeed_last = maxspeed_current
        maxspeed_current = last_drive_message.velocity
        maxspeed_delta = maxspeed_current - maxspeed_last
        if logentry > 0:
            maxspeed_avg = ( maxspeed_avg * ( logentry - 1 ) + maxspeed_current ) / logentry
        else:
            maxspeed_avg += maxspeed_current
        
        angle_last = angle_current
        angle_current = last_drive_message.angle
        angle_delta = angle_current - angle_last
        
        
    time_last = time_current
    time_current = rospy.get_time()
    time_delta = time_current - time_last
    
    distance_delta = time_delta * speed_current
    distance_last = distance_current
    distance_current += distance_delta
    
    acceleration_last = acceleration_current
    acceleration_current = 0
    if time_delta > 0:
        acceleration_current = speed_delta / time_delta
    acceleration_delta = acceleration_current - acceleration_last
    
    if logentry == 0:
        logbase = getStatsPath()
        dateTimeObj = datetime.now()    
        timestampStr = dateTimeObj.strftime("%Y%m%d-%H%M%S")
        loginstance = ""
        if len(sys.argv) > 1:
            loginstance = sys.argv[1] + "_"
        logpath = logbase + "/" + loginstance + timestampStr
        
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
        logfile_handler_csv = open(logpath + "/speed_over_time.csv", "a")
        logfile_handler_dat = open(logpath + "/speed_over_time.dat", "a")
        logstring_header = "Datapoint;\
            Time;\
            TimeDelta;\
            Speed;\
            SpeedDelta;\
            SpeedAverage;\
            Maxspeed;\
            MaxspeedDelta;\
            MaxspeedAverage;\
            Angle;\
            AngleDelta;\
            Acceleration;\
            AccelerationDelta;\
            Distance;\
            DistanceDelta\
            \n"
        logfile_handler_csv.write(logstring_header)
        logfile_handler_dat.write(logstring_header.replace("Datapoint","x").lower())


    logfile_handler_csv = open(logpath + "/speed_over_time.csv", "a")
    logfile_handler_dat = open(logpath + "/speed_over_time.dat", "a")
        
    logstring = str(logentry) + \
        ";" + str('%.2f' % time_current) + \
        ";" + str('%.2f' % time_delta) + \
        ";" + str('%.2f' % speed_current) + \
        ";" + str('%.2f' % speed_delta) + \
        ";" + str('%.2f' % speed_avg) + \
        ";" + str('%.2f' % maxspeed_current) + \
        ";" + str('%.2f' % maxspeed_delta) + \
        ";" + str('%.2f' % maxspeed_avg) + \
        ";" + str('%.2f' % angle_current) + \
        ";" + str('%.2f' % angle_delta) + \
        ";" + str('%.2f' % acceleration_current) + \
        ";" + str('%.2f' % acceleration_delta) + \
        ";" + str('%.2f' % distance_current) + \
        ";" + str('%.2f' % distance_delta) + \
        "\n"
    logfile_handler_csv.write(logstring)
    logfile_handler_dat.write(logstring.replace(";"," "))
    
    logentry += 1


global logentry
logentry = 0
    
global speed_current
global speed_last
global speed_delta
global speed_avg

global maxspeed_current
global maxspeed_last
global maxspeed_delta
global maxspeed_avg

global time_current
global time_last
global time_delta

global distance_current
global distance_last
global distance_delta

global acceleration_current
global acceleration_last
global acceleration_delta

global angle_current
global angle_last
global angle_delta

speed_current = 0
speed_last = 0
speed_delta = 0
speed_avg = 0

maxspeed_current = 0
maxspeed_last = 0
maxspeed_delta = 0
maxspeed_avg = 0

time_current = 0
time_last = 0
time_delta = 0

distance_current = 0
distance_last = 0
distance_delta = 0

acceleration_current = 0
acceleration_last = 0
acceleration_delta = 0

angle_current = 0
angle_last = 0
angle_delta = 0

rospy.init_node('log_stats', anonymous=True)
rospy.Subscriber(TOPIC_GAZEBO_MODEL_STATE, ModelStates, on_model_state_callback)
rospy.Subscriber(TOPIC_GAZEBO_STATE_TELEMETRY, gazebo_state_telemetry, speed_callback)
rospy.Subscriber(TOPIC_DRIVE_PARAMETERS, drive_param, drive_param_callback)

while not rospy.is_shutdown():
    rospy.spin()
