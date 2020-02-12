#!/usr/bin/env python

import sys
import numpy
import rospy
import datetime
import psycopg2
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


TOPIC_LASER_SCAN = "/scan"
TOPIC_VOXEL = "/scan/voxels"
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"

voxel_resolution = 0.2

last_voxel_message = None
last_voxel_message_time = None

def connect_to_database():
    try:
        global connection
        connection = psycopg2.connect(user = "postgres",
                                    password = "postgres",
                                    host = "localhost",
                                    port = "5432",
                                    database = "testdb")

        global cursor
        cursor = connection.cursor()
        # Print PostgreSQL Connection properties
        print ( connection.get_dsn_parameters(),"\n")

        # Print PostgreSQL version
        cursor.execute("SELECT version();")
        record = cursor.fetchone()
        print("You are connected to - ", record,"\n")

    except (Exception, psycopg2.Error) as error :
        print ("Error while connecting to PostgreSQL", error)
    
def write_entry_to_db(current_time,current_speed,voxel,velocity,angle):
    current_time_formated = current_time.strftime("'%m-%d-%Y %H:%M:%S.%f'") #"'01-01-2000 00:00:00.000'"
    numpy.set_printoptions(threshold=sys.maxsize)
    voxel_string = numpy.array2string(voxel, separator=',')
    sector_number=0
    sector_time=0

    voxel_string=voxel_string.replace('[','{').replace(']','}').replace('False','f').replace('True','t')
    voxel_string="'"+voxel_string+"'"


    values_string = "VALUES("+current_time_formated+", "+str(current_speed)+","+voxel_string+", "+str(velocity)+","+str(angle)+","+str(sector_time)+","+str(sector_number)+")"

    try:
        cursor.execute("INSERT INTO public.training_data (ts,speed,voxel,calc_velocity,calc_angle,sector_time,sector_number) "+ values_string+";")
    except (Exception, psycopg2.Error) as error :
        print ("Error while storing training data in PostgreSQL", error)


def createDBVoxelArray(voxel_message):
    voxel = numpy.zeros((101, 101), dtype=bool)

    p_gen = pc2.read_points(voxel_message, field_names = ("x", "y", "z", "score"), skip_nans=True)

    for p in p_gen:
        x= p[0]
        y= p[1]
        z= p[2]
        score= p[3]
        x_index = x/voxel_resolution +50
        y_index = y/voxel_resolution +50
        if (abs(x_index)<=100 and abs(y_index)<=100):
            voxel[int(x_index),int(y_index)] = 1        #TODO bisher nicht wirklich effizient

    return voxel

def laser_callback(scan_message):
    last_scan_message = scan_message
    last_scan_time = datetime.datetime.now()
    
    cursor = connection.cursor()
    cursor.execute("INSERT INTO public.test_table (test) VALUES(1);")
    connection.commit()
    cursor.close()

def voxel_callback(voxel_message):
    global last_voxel_message
    last_voxel_message = voxel_message


def drive_callback(drive_message):
    current_speed = 0
    current_time = datetime.datetime.now()

    velocity = drive_message.velocity
    angle = drive_message.angle
    if(last_voxel_message is not None):
        dbVoxelArray = createDBVoxelArray(last_voxel_message)
        write_entry_to_db(current_time, current_speed,dbVoxelArray,velocity,angle)



rospy.init_node('store_training_data', anonymous=True)
connect_to_database()

#rospy.Subscriber(TOPIC_LASER_SCAN, LaserScan, laser_callback)
rospy.Subscriber(TOPIC_VOXEL, PointCloud2, voxel_callback)
rospy.Subscriber(TOPIC_DRIVE_PARAMETERS, drive_param, drive_callback)

while not rospy.is_shutdown():
    rospy.spin()

if(connection):
    connection.commit()
    cursor.close()
    connection.close()
    print("PostgreSQL connection is closed")
