#!/usr/bin/env python

import rospy
import datetime
import psycopg2
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param
from std_msgs.msg import String


TOPIC_LASER_SCAN = "/scan"
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"

last_speed = 0

def connect_to_database(self):
    try:
        connection = psycopg2.connect(user = "postgres",
                                    password = "postgres",
                                    host = "127.0.0.1",
                                    port = "5433",
                                    database = "my_db")

        cursor = connection.cursor()
        # Print PostgreSQL Connection properties
        print ( connection.get_dsn_parameters(),"\n")

        # Print PostgreSQL version
        cursor.execute("SELECT version();")
        record = cursor.fetchone()
        print("You are connected to - ", record,"\n")

    except (Exception, psycopg2.Error) as error :
        print ("Error while connecting to PostgreSQL", error)
    finally:
        #closing database connection.
            if(connection):
                cursor.close()
                connection.close()
                print("PostgreSQL connection is closed")


def laser_callback(scan_message):
    last_scan_message = scan_message
    last_scan_time = datetime.datetime.now()
    
    cursor = connection.cursor()
    cursor.execute("INSERT INTO test_table (test) VALUES(1)")
    cursor.close()
    conn.close()

def laser_callback(drive_message):
    last_drive_message = drive_message
    last_drive_message_time = datetime.datetime.now()


rospy.init_node('store_training_data', anonymous=True)
connect_to_database()

rospy.Subscriber(TOPIC_LASER_SCAN, LaserScan, laser_callback)
rospy.Subscriber(TOPIC_DRIVE_PARAMETERS, drive_param, drive_callback)

while not rospy.is_shutdown():
    rospy.spin()
