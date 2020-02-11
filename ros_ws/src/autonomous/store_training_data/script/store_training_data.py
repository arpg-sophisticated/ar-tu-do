#!/usr/bin/env python

import rospy
import datetime
#import timezone
import psycopg2
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
from std_msgs.msg import String


TOPIC_LASER_SCAN = "/scan"
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"

connection = None

def connect_to_database():
    try:
        global connection
        connection = psycopg2.connect(user = "postgres",
                                    password = "postgres",
                                    host = "localhost",
                                    port = "5432",
                                    database = "testdb")

        cursor = connection.cursor()
        # Print PostgreSQL Connection properties
        print ( connection.get_dsn_parameters(),"\n")

        # Print PostgreSQL version
        cursor.execute("SELECT version();")
        record = cursor.fetchone()
        print("You are connected to - ", record,"\n")

    except (Exception, psycopg2.Error) as error :
        print ("Error while connecting to PostgreSQL", error)
    
def write_entry_to_db(current_time,current_speed,voxel,new_velocity,new_angle):
    cursor = connection.cursor()

    #current_time_formated = current_time.astimezone(timezone.utc).strftime("%m-%d-%Y %H:%M:%S.%f")

    current_time_formated = "'01-01-2000 00:00:00-00'"

    voxel_string= "'{"

    for i in voxel:
        voxel_string = voxel_string + "{"
        for j in i:
            voxel_string = voxel_string +  str(j) + ","
        voxel_string = voxel_string[:-1] + "}" + ","
    voxel_string = voxel_string[:-1] + "}'"
            

    values_string = "VALUES("+current_time_formated+", "+str(current_speed)+","+voxel_string+", "+str(new_velocity)+","+str(new_angle)+")"

    cursor.execute("INSERT INTO public.training_data (ts,speed,voxel,calc_velocity,calc_angle) "+ values_string+";")
    connection.commit()
    cursor.close()

def laser_callback(scan_message):
    last_scan_message = scan_message
    last_scan_time = datetime.datetime.now()
    
    cursor = connection.cursor()
    cursor.execute("INSERT INTO public.test_table (test) VALUES(1);")
    connection.commit()
    cursor.close()

def voxel_callback(voxel_message):
    current_speed = 0
    current_time = datetime.datetime.now()
    voxel = [[False,False,False],[False,False,False],[False,False,False]]

    new_velocity = 0
    new_angle = 0

    write_entry_to_db(current_time, current_speed,voxel,new_velocity,new_angle)


def drive_callback(drive_message):
    last_drive_message = drive_message
    last_drive_message_time = datetime.datetime.now()


rospy.init_node('store_training_data', anonymous=True)
connect_to_database()

voxel_callback("") # just a test, TODO: remove this line

rospy.Subscriber(TOPIC_LASER_SCAN, LaserScan, laser_callback)
rospy.Subscriber(TOPIC_DRIVE_PARAMETERS, drive_param, drive_callback)

while not rospy.is_shutdown():
    rospy.spin()

if(connection):
    connection.close()
    print("PostgreSQL connection is closed")
