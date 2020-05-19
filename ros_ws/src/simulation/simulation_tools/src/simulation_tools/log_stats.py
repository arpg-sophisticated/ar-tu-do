#!/usr/bin/env python
# arguments:
# 1: log prefix (string), default: none
# 2: length for min/max arrays (int), default: 100
# 3: smoothing value of acceleration (int), default: 5
# 4: whether its simulation or not (string: yes|no), default: no
# 5: whether write statistics or not (string: yes|no), default: no

import rospy
import os
import sys
import numpy as np
from datetime import datetime
from rospkg import RosPack
from simulation_tools.track import track
from simulation_tools.reset_car import Point
from drive_msgs.msg import drive_param,gazebo_state_telemetry
from gazebo_msgs.msg import ModelStates
from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface

TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_GAZEBO_MODEL_STATE = "/gazebo/model_states"
TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry"

# file handler
global logfile_handler_csv;
global logfile_handler_dat;

# misc car messages
global last_drive_message
last_drive_message = None
# disabled as we don't need it here
#global car_position
#car_position = None
#global car_orientation
#car_orientation = 0
#global track_position
#track_position = 0

# logging path
global logpath
logpath = ""

# log entry number
global logentry
logentry = 0

# interface for HUD topic
global text_interface

# current speed
global speed_current
speed_current = 0
# last speed value
global speed_last
speed_last = 0
# realitve change of speed
global speed_delta
speed_delta = 0
# overall average speed
global speed_avg
speed_avg = 0
# average speed last TIME (arg) intervals
global speed_avgtime
speed_avgtime = []
# overall average speed smooth
global speed_smooth
speed_smooth = []
# total top speed
global speed_max
speed_max = 0
# top speed last TIME (arg) intervals
global speed_maxtime
speed_maxtime = []

# current maximum speed given from algorithm
global maxspeed_current
maxspeed_current = 0
# last maximum speed given from algorithm
global maxspeed_last
maxspeed_last = 0
# relative change of maximum speed given from algorithm
global maxspeed_delta
maxspeed_delta = 0
# average maximum speed given from algorithm
global maxspeed_avg
maxspeed_avg = 0

# current time stamp
global time_current
time_current = 0
# last time stamp
global time_last
time_last = 0
# relative change in timestamp
global time_delta
time_delta = 0

# driven distance
global distance_current
distance_current = 0
# driven distance last interval
global distance_last
distance_last = 0
# relative driven distance since last interval
global distance_delta
distance_delta = 0

# current acceleration
global acceleration_current
acceleration_current = 0
# acceleration last intervals
global acceleration_last
acceleration_last = 0
# smoothed acceleration
global acceleration_smooth
acceleration_smooth = []
# minimal acceleration overall
global acceleration_min
acceleration_min = 0
# maximum acceleration overall
global acceleration_max
acceleration_max = 0
# minimal acceleration last TIME (arg) intervals
global acceleration_mintime
acceleration_mintime = []
# maximum acceleration last TIME (arg) intervals
global acceleration_maxtime
acceleration_maxtime = []
# change of acceleration since last interval
global acceleration_delta
acceleration_delta = 0

# current angle of wheels
global angle_current
angle_current = 0
# last intervals angle of wheels
global angle_last
angle_last = 0
# relative change in wheels angle
global angle_delta
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
    
    # is it simulation?
    simulation = False
    if len(sys.argv) > 4: 
        if str(sys.argv[4]) == "yes":
            simulation = True
        
    # set current drive based on if it is simulation
    if simulation:
        last_drive_message = message
    else:
        # TODO insert real drive information
        last_drive_message = message

 
def speed_callback(speed_message):
    global speed_current
    global speed_last
    global speed_delta
    global speed_avg
    global speed_avgtime
    global speed_max
    global speed_maxtime
    global speed_smooth
    
    speed_last = speed_current
    
    # is it simulation?
    simulation = False
    if len(sys.argv) > 4: 
        if str(sys.argv[4]) == "yes":
            simulation = True
        
    # set current speed based on if it is simulation
    if simulation:
        speed_current = speed_message.wheel_speed
    else:
        # TODO insert real speed information
        speed_current = speed_message.wheel_speed
        
    speed_delta = speed_current - speed_last
    if logentry > 0:
        speed_avg = ( speed_avg * ( logentry - 1 ) + speed_current ) / logentry
    else:
        speed_avg += speed_current
    
    if speed_current > speed_max:
        speed_max = speed_current
    
    time = 100
    if len(sys.argv) > 2: 
        time = int(sys.argv[2])
    
    smooth = 5
    if len(sys.argv) > 3: 
        smooth = int(sys.argv[3])
    
    speed_maxtime.append(speed_current)
    if len(speed_maxtime) > time:
        speed_maxtime.pop(0)
    
    speed_avgtime.append(speed_current)
    if len(speed_avgtime) > time:
        speed_avgtime.pop(0)
    
    speed_smooth.append(speed_current)
    if len(speed_smooth) > smooth:
        speed_smooth.pop(0)
            
    log_message()

# disabled as we don't need this information
#def on_model_state_callback(message):
#    if len(message.pose) < 2:
#        return
#    global car_position
#    global car_orientation
#    global track_position
#    car_position = Point(
#        message.pose[1].position.x,
#        message.pose[1].position.y)
#    car_orientation = message.pose[1].orientation
#    track_position = track.localize(car_position)

def log_message():
    global last_drive_message
# disabled as we don't need it here
#    global track_position
#    global car_position
    global logpath
    global logentry
    
    global speed_current
    global speed_last
    global speed_delta
    global speed_avg
    global speed_avgtime
    global speed_max
    global speed_maxtime
    global speed_smooth
    
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
    global acceleration_min
    global acceleration_max
    global acceleration_mintime
    global acceleration_maxtime
    global acceleration_smooth
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
    
    if acceleration_current > acceleration_max:
        acceleration_max = acceleration_current
    
    if acceleration_current < acceleration_min:
        acceleration_min = acceleration_current
            
    time = 100
    if len(sys.argv) > 2: 
        time = int(sys.argv[2])
            
    smooth = 5
    if len(sys.argv) > 3: 
        smooth = int(sys.argv[3])
    
    acceleration_maxtime.append(acceleration_current)
    if len(acceleration_maxtime) > time:
        acceleration_maxtime.pop(0)
    
    acceleration_mintime.append(acceleration_current)
    if len(acceleration_mintime) > time:
        acceleration_mintime.pop(0)
    
    acceleration_mintime.append(acceleration_current)
    if len(acceleration_mintime) > time:
        acceleration_mintime.pop(0)
    
    acceleration_smooth.append(acceleration_current)
    if len(acceleration_smooth) > smooth:
        acceleration_smooth.pop(0)
    
    logbase = getStatsPath()
    if logentry == 0:
        # log writing enabled
        logwriting = False
        if len(sys.argv) > 5: 
            if str(sys.argv[5]) == "yes":
                logwriting = True
        
        if logwriting:
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
            logstring_header = \
                "Datapoint;" + \
                "AverageTime;" + \
                "AverageSmooth;" + \
                "Time;" + \
                "TimeDelta;" + \
                "Speed;" + \
                "SpeedDelta;" + \
                "SpeedAverage;" + \
                "SpeedAverageTime;" + \
                "SpeedMax;" + \
                "SpeedMaxTime;" + \
                "Maxspeed;" + \
                "MaxspeedDelta;" + \
                "MaxspeedAverage;" + \
                "Angle;" + \
                "AngleDelta;" + \
                "Acceleration;" + \
                "AccelerationSmooth;" + \
                "AccelerationDelta;" + \
                "AccelerationMin;" + \
                "AccelerationMax;" + \
                "AccelerationMinTime;" + \
                "AccelerationMaxTime;" + \
                "Distance;" + \
                "DistanceDelta" + \
                "\n"
            logfile_handler_csv.write(logstring_header)
            logfile_handler_dat.write(logstring_header.replace(";"," ").replace("Datapoint","x").lower())

    # log writing enabled
    logwriting = False
    if len(sys.argv) > 5: 
        if str(sys.argv[5]) == "yes":
            logwriting = True
            
    if logwriting:
        logfile_handler_csv = open(logpath + "/speed_over_time.csv", "a")
        logfile_handler_dat = open(logpath + "/speed_over_time.dat", "a")

        logstring = \
            str(logentry) + \
            ";" + str(time) + \
            ";" + str(smooth) + \
            ";" + str('%.2f' % time_current) + \
            ";" + str('%.2f' % time_delta) + \
            ";" + str('%.2f' % speed_current) + \
            ";" + str('%.2f' % speed_delta) + \
            ";" + str('%.2f' % speed_avg) + \
            ";" + str('%.2f' % np.mean(speed_avgtime)) + \
            ";" + str('%.2f' % speed_max) + \
            ";" + str('%.2f' % max(speed_maxtime)) + \
            ";" + str('%.2f' % maxspeed_current) + \
            ";" + str('%.2f' % maxspeed_delta) + \
            ";" + str('%.2f' % maxspeed_avg) + \
            ";" + str('%.2f' % angle_current) + \
            ";" + str('%.2f' % angle_delta) + \
            ";" + str('%.2f' % acceleration_current) + \
            ";" + str('%.2f' % np.mean(acceleration_smooth)) + \
            ";" + str('%.2f' % acceleration_delta) + \
            ";" + str('%.2f' % acceleration_min) + \
            ";" + str('%.2f' % acceleration_max) + \
            ";" + str('%.2f' % max(acceleration_maxtime)) + \
            ";" + str('%.2f' % min(acceleration_mintime)) + \
            ";" + str('%.2f' % distance_current) + \
            ";" + str('%.2f' % distance_delta) + \
            "\n"
        logfile_handler_csv.write(logstring)
        logfile_handler_dat.write(logstring.replace(";"," "))
    
    hudstring = \
        "Time: " + str('%.2f' % time_current) + " s\n" + \
        "Vcur: " + str('%.2f' % speed_current) + " m/s\n" + \
        "Vsmo: " + str('%.2f' % np.mean(speed_smooth)) + " m/s\n" + \
        "Vavg: " + str('%.2f' % speed_avg) + " m/s\n" + \
        "Vav+: " + str('%.2f' % np.mean(speed_avgtime)) + " m/s\n" + \
        "Vtop: " + str('%.2f' % speed_max) + " m/s\n" + \
        "Vto+: " + str('%.2f' % max(speed_maxtime)) + " m/s\n" + \
        "Vmax: " + str('%.2f' % maxspeed_current) + " m/s\n" + \
        "Angl: " + str('%.2f' % angle_current) + "\n" + \
        "Acur: " + str('%.2f' % acceleration_current) + " m/s^2\n" + \
        "Asmo: " + str('%.2f' % np.mean(acceleration_smooth)) + " m/s^2\n" + \
        "Amin: " + str('%.2f' % acceleration_min) + " m/s^2\n" + \
        "Amax: " + str('%.2f' % acceleration_max) + " m/s^2\n" + \
        "Ato+: " + str('%.2f' % max(acceleration_maxtime)) + " m/s^2\n" + \
        "Ato-: " + str('%.2f' % min(acceleration_mintime)) + " m/s^2\n" + \
        "Dist: " + str('%.2f' % distance_current) + " m\n" + \
        "Iter: " + str(logentry) + "\n" + \
        "AvgT: " + str(time) + " (Vto+ Vav+ Ato+ Ato-)\n" + \
        "AvgS: " + str(smooth) + " (Vsmo Asmo)\n"
    text_interface.publish(str(hudstring))

    logentry += 1


rospy.init_node('log_stats', anonymous=False)
# disabled as we don't need it here
#rospy.Subscriber(TOPIC_GAZEBO_MODEL_STATE, ModelStates, on_model_state_callback)
rospy.Subscriber(TOPIC_GAZEBO_STATE_TELEMETRY, gazebo_state_telemetry, speed_callback)
rospy.Subscriber(TOPIC_DRIVE_PARAMETERS, drive_param, drive_param_callback)

global text_interface
text_interface = OverlayTextInterface("hud")

while not rospy.is_shutdown():
    rospy.spin()