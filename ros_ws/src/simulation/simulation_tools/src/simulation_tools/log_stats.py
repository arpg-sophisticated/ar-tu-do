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
from drive_msgs.msg import drive_param, gazebo_state_telemetry
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface
from std_msgs.msg import ColorRGBA, Float32, Int32
from time import strftime
from time import gmtime

try:
    from jsk_rviz_plugins.msg import *
except BaseException:
    import roslib
    roslib.load_manifest("jsk_rviz_plugins")
    from jsk_rviz_plugins.msg import *


TOPIC_MAX_SPEED = "/speed_info/max_speed"
TOPIC_DRIVE_PARAMETERS = "/commands/controlled_drive_param"
TOPIC_GAZEBO_MODEL_STATE = "/gazebo/model_states"
TOPIC_GAZEBO_STATE_TELEMETRY = "/gazebo/state_telemetry"
TOPIC_CONTROLLED_DRIVE_PARAM = "/commands/controlled_drive_param"

# file handler
global logfile_handler_csv
global logfile_handler_dat

# misc car messages
global last_drive_message
last_drive_message = None

# max possible speed
global last_max_speed_message
last_max_speed_message = None

# logging path
global logpath
logpath = ""

# log entry number
global logentry
logentry = 0

# interface for HUD topic
global hud_text_interface

# interface for HUD speed value
global hud_speed_value
# interface for HUD max speed value
global hud_maxspeed_value
# interface for HUD rpm value
global hud_rpm_value
# interface for HUD acceleration value
global hud_acceleration_value
# interface for HUD angle value
global hud_angle_value
# interface for HUD distance value
global hud_distance_value
# interface for HUD clock value
global hud_clock_value

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
# rpm factor derived from code
global speed_rpmfactor
speed_rpmfactor = 3118.138

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
# current maximum speed given from algorithm smoothed
global maxspeed_smooth
maxspeed_smooth = []

# current time bias only needed in real mode
global time_bias
time_bias = 0
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

# current turn of wheels (-1 ... 1)
global turn_current
turn_current = 0
# last intervals turn of wheels
global turn_last
turn_last = 0
# relative change in wheels turn
global turn_delta
turn_delta = 0

# current angle of wheels (-maxangle ... 0 ... maxangle in degree)
global angle_current
angle_current = 0
# last intervals angle of wheels
global angle_last
angle_last = 0
# relative change in wheels angle
global angle_delta
angle_delta = 0
# this is a constant value measured on max turn angle the car may do (40 deg)
global angle_max
angle_max = 40
# current maximum speed given from algorithm smoothed
global angle_smooth
angle_smooth = []


def getDataPath():
    fullpath = RosPack().get_path("simulation_tools").split("/")
    fullpath.reverse()
    relativepath = "../data"
    insert = False
    for part in fullpath:
        if part == "ros_ws":
            insert = True
        if insert:
            relativepath = part + "/" + relativepath
    return relativepath.replace("//", "/")


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


def max_speed_callback(message):
    global last_max_speed_message

    # is it simulation?
    simulation = False
    if len(sys.argv) > 4:
        if str(sys.argv[4]) == "yes":
            simulation = True

    # set current drive based on if it is simulation
    if simulation:
        last_max_speed_message = message
    else:
        # TODO insert real drive information
        last_max_speed_message = message


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
        speed_current = speed_message.velocity

    speed_delta = speed_current - speed_last
    if logentry > 0:
        speed_avg = (speed_avg * (logentry - 1) + speed_current) / logentry
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


def log_message():
    global last_max_speed_message
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
    global speed_rpmfactor

    global maxspeed_current
    global maxspeed_last
    global maxspeed_delta
    global maxspeed_avg
    global maxspeed_smooth

    global time_bias
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

    global turn_current
    global turn_last
    global turn_delta

    global angle_current
    global angle_last
    global angle_delta
    global angle_max
    global angle_smooth

    global logfile_handler_csv
    global logfile_handler_dat

    global hud_text_interface

    # interface for HUD speed value
    global hud_speed_value
    # interface for HUD max speed value
    global hud_maxspeed_value
    # interface for HUD rpm value
    global hud_rpm_value
    # interface for HUD acceleration value
    global hud_acceleration_value
    # interface for HUD angle value
    global hud_angle_value
    # interface for HUD distance value
    global hud_distance_value
    # interface for HUD clock value
    global hud_clock_value

    smooth = 5
    if len(sys.argv) > 3:
        smooth = int(sys.argv[3])

    if(last_max_speed_message is not None):
        maxspeed_last = maxspeed_current
        maxspeed_current = last_max_speed_message.data
        maxspeed_delta = maxspeed_current - maxspeed_last
        if logentry > 0:
            maxspeed_avg = (maxspeed_avg * (logentry - 1) +
                            maxspeed_current) / logentry
        else:
            maxspeed_avg += maxspeed_current

        maxspeed_smooth.append(maxspeed_current)
        if len(maxspeed_smooth) > smooth:
            maxspeed_smooth.pop(0)

        turn_last = turn_current
        turn_current = last_drive_message.angle
        turn_delta = turn_current - turn_last

        angle_last = angle_current
        angle_current = last_drive_message.angle * angle_max
        angle_delta = angle_current - angle_last

        angle_smooth.append(angle_current)
        if len(angle_smooth) > smooth:
            angle_smooth.pop(0)

    # calculate bias on first run
    if (time_bias == 0):
        time_bias = rospy.get_time()

    time_last = time_current
    # is it simulation?
    simulation = False
    if len(sys.argv) > 4:
        if str(sys.argv[4]) == "yes":
            simulation = True

    if simulation:
        time_current = rospy.get_time()

    else:
        time_current = rospy.get_time() - time_bias

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

    datapath = getDataPath()
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
            logpath = datapath + "/telemetry/" + loginstance + timestampStr

            # create logpath for session if nonexistant
            try:
                os.makedirs(logpath)
            except BaseException:
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
                "DistanceDelta;" + \
                "Turn;" + \
                "TurnDelta;" + \
                "RPM" + \
                "\n"
            logfile_handler_csv.write(logstring_header)
            logfile_handler_dat.write(
                logstring_header.replace(
                    ";", " ").replace(
                    "Datapoint", "x").lower())

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
            ";" + str('%.2f' % turn_current) + \
            ";" + str('%.2f' % turn_delta) + \
            ";" + str('%.2f' % (speed_current * speed_rpmfactor)) + \
            "\n"
        logfile_handler_csv.write(logstring)
        logfile_handler_dat.write(logstring.replace(";", " "))

    hudstring = \
        "Time: " + str('%.2f' % time_current) + " s\n" + \
        "Rcur: " + str('%.2f' % (speed_current * speed_rpmfactor)) + " ^-1\n" + \
        "Rsmo: " + str('%.2f' % (np.mean(speed_smooth) * speed_rpmfactor)) + " ^-1\n" + \
        "Vcur: " + str('%.2f' % speed_current) + " m/s\n" + \
        "Vsmo: " + str('%.2f' % np.mean(speed_smooth)) + " m/s\n" + \
        "Vavg: " + str('%.2f' % speed_avg) + " m/s\n" + \
        "Vav+: " + str('%.2f' % np.mean(speed_avgtime)) + " m/s\n" + \
        "Vtop: " + str('%.2f' % speed_max) + " m/s\n" + \
        "Vto+: " + str('%.2f' % max(speed_maxtime)) + " m/s\n" + \
        "Vmax: " + str('%.2f' % maxspeed_current) + " m/s\n" + \
        "Vmsm: " + str('%.2f' % np.mean(maxspeed_smooth)) + " m/s\n" + \
        "Angl: " + str('%.2f' % angle_current) + "\n" + \
        "Ansm: " + str('%.2f' % np.mean(angle_current)) + "\n" + \
        "Turn: " + str('%.2f' % turn_current) + "\n" + \
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

    hud_text = OverlayText()
    hud_text.width = 400
    hud_text.height = 500
    hud_text.left = 280
    hud_text.top = 20
    hud_text.text_size = 10
    hud_text.line_width = 3
    hud_text.font = "Cousine"
    hud_text.text = str(hudstring)
    hud_text.fg_color = ColorRGBA(0.93, 0.16, 0.16, 1.0)
    hud_text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
    hud_text_interface.publish(hud_text)

    hud_clock = OverlayText()
    hud_clock.width = 300
    hud_clock.height = 30
    hud_clock.left = 39
    hud_clock.top = 20
    hud_clock.text_size = 24
    hud_clock.line_width = 3
    hud_clock.font = "Cousine"
    hud_clock.text = str(strftime("%H:%M:%S", gmtime(time_current))) + "."
    if (time_current * 100) % 100 > 9:
        hud_clock.text += str('%.0f' % ((time_current * 100) % 100))
    else:
        hud_clock.text += "0" + str('%.0f' % ((time_current * 100) % 100))
    hud_clock.fg_color = ColorRGBA(0.09, 1.0, 0.9, 1.0)
    hud_clock.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
    hud_clock_value.publish(hud_clock)

    hud_distance = OverlayText()
    hud_distance.width = 300
    hud_distance.height = 30
    if distance_current > 10000:
        hud_distance.left = 58
    elif distance_current >= 1000:
        hud_distance.left = 77
    elif distance_current >= 100:
        hud_distance.left = 96
    elif distance_current >= 10:
        hud_distance.left = 115
    else:
        hud_distance.left = 134
    hud_distance.top = 50
    hud_distance.text_size = 24
    hud_distance.line_width = 3
    hud_distance.font = "Cousine"
    hud_distance.text = str('%.2f' % distance_current) + " m"
    hud_distance.fg_color = ColorRGBA(0.09, 1.0, 0.9, 1.0)
    hud_distance.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
    hud_distance_value.publish(hud_distance)

    hud_speed_value.publish(np.mean(speed_smooth))
    hud_maxspeed_value.publish(np.mean(maxspeed_smooth))
    hud_rpm_value.publish(int((np.mean(speed_smooth) * speed_rpmfactor)))
    hud_acceleration_value.publish(np.mean(acceleration_smooth))
    hud_angle_value.publish(np.mean(angle_current) * (-1))

    logentry += 1


rospy.init_node('log_stats', anonymous=False)
rospy.Subscriber(TOPIC_MAX_SPEED, Float64, max_speed_callback)
rospy.Subscriber(TOPIC_DRIVE_PARAMETERS, drive_param, drive_param_callback)

# is it simulation?
if len(sys.argv) > 4:
    if str(sys.argv[4]) == "yes":
        # then we fetch current speed from gazebo node
        rospy.Subscriber(
            TOPIC_GAZEBO_STATE_TELEMETRY,
            gazebo_state_telemetry,
            speed_callback)
    else:
        # else we fetch current speed from acceleration controller
        rospy.Subscriber(
            TOPIC_CONTROLLED_DRIVE_PARAM,
            drive_param,
            speed_callback,
            queue_size=1)


# interface for HUD topic
global hud_text_interface
hud_text_interface = rospy.Publisher('hud', OverlayText, queue_size=1)

# interface for HUD speed value
global hud_speed_value
hud_speed_value = rospy.Publisher('hud_speed_value', Float32, queue_size=1)

# interface for HUD max speed value
global hud_maxspeed_value
hud_maxspeed_value = rospy.Publisher(
    'hud_maxspeed_value', Float32, queue_size=1)

# interface for HUD rpm value
global hud_rpm_value
hud_rpm_value = rospy.Publisher('hud_rpm_value', Float32, queue_size=1)

# interface for HUD acceleration value
global hud_acceleration_value
hud_acceleration_value = rospy.Publisher(
    'hud_acceleration_value', Float32, queue_size=1)

# interface for HUD angle value
global hud_angle_value
hud_angle_value = rospy.Publisher('hud_angle_value', Float32, queue_size=1)

# interface for HUD distance value
global hud_distance_value
hud_distance_value = rospy.Publisher(
    'hud_distance_value', OverlayText, queue_size=1)

# interface for HUD clock value
global hud_clock_value
hud_clock_value = rospy.Publisher('hud_clock_value', OverlayText, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()
