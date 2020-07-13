#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
import sys
import numpy
import rospy
import datetime
from drive_msgs.msg import drive_param
from std_msgs.msg import Empty
from drive_msgs.msg import gazebo_state_telemetry
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import os, rospkg
from utilities import createDBVoxelArray
import math
import tensorflow as tf
from tensorflow.python.client import device_lib
import psycopg2


import simulation_tools.reset_car as reset_car

rospack = rospkg.RosPack()

numpy.set_printoptions(threshold=sys.maxsize)

TOPIC_LASER_SCAN = "/scan"
TOPIC_CRASH = "/crash"
TOPIC_STATE_TELEMETRY = "/gazebo/state_telemetry"
TOPIC_VOXEL = "/scan/voxels"
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"

last_state_telemetry_message = None
last_laser_values = None
model = None

voxel = False
laser_sample_count = 128

def voxel_callback(voxel_message):
      
    print (str(datetime.datetime.now())+" createVoxelArray")
    voxel_arry = createDBVoxelArray(voxel_message)
    voxel_arry = np.expand_dims(voxel_arry ,-1)
    voxel_arry = np.expand_dims(voxel_arry ,0)

    if(last_state_telemetry_message is not None):
        wheel_speed = last_state_telemetry_message.wheel_speed
    else:
        wheel_speed =0
    wheel_speed = np.array([(wheel_speed*1.547)+4.853])


    print (str(datetime.datetime.now())+" predict")
    prediction = model.predict([voxel_arry,wheel_speed],use_multiprocessing=True)


    velocity_stdev=1.446
    angle_stdev=0.262
    velocity_avg=4.603
    angle_avg=-0.094

    message = drive_param()
    message.velocity = (prediction[0,0]*velocity_stdev)+velocity_avg

    message.velocity = message.velocity*0.1
    message.angle = (prediction[0,1]*angle_stdev)+angle_avg

    print("vel: "+ str(message.velocity) + ", angle: "+ str(message.angle))

    print (str(datetime.datetime.now())+" publish")
    drive_parameters_publisher.publish(message)

def state_telemetry_callback(state_telemetry_message):
    global last_state_telemetry_message
    last_state_telemetry_message = state_telemetry_message


def laser_callback(scan_message):
    global last_laser_values
    global model
    if( model is None):
        return
    scan_indices = [int(i * (len(scan_message.ranges) - 1) / (laser_sample_count - 1)) for i in range(laser_sample_count)]  # nopep8

    values = [scan_message.ranges[i] for i in scan_indices]
    values = [v if not math.isinf(v) else 100 for v in values]
    last_laser_values = values

    if(last_state_telemetry_message is not None):
        wheel_speed = last_state_telemetry_message.wheel_speed
    else:
        wheel_speed =0
    
    values = [wheel_speed] + values
    np_values = np.asarray(values, dtype=np.float32)

    np_values = np.expand_dims(np_values ,0)

    print (str(datetime.datetime.now())+" predict")

    with tf.device('/gpu:0'):
        prediction = model.predict(np_values)

    message = drive_param()
    #message.velocity = prediction[0,0]*12 # divide by 12 because of normalization in training
    message.velocity = 0.5
    #smessage.angle = prediction[0,1]
    message.angle = prediction[0,0]

    print("vel: "+ str(message.velocity) + ", angle: "+ str(message.angle))

    print (str(datetime.datetime.now())+" publish")
    drive_parameters_publisher.publish(message)

def load_model():
    global model

    with tf.device('/gpu:0'):
        tf.keras.backend.clear_session()

        print("---------------------Start Loading model from disk-------------------------")
        model_folder =os.path.join(rospack.get_path("reinforcement_learning2"), "model")

        json_file = open(os.path.join(model_folder,'model30-06-2020_20:00.json'), 'r')
        print("---------------------Start reading json-------------------------")
        loaded_model_json = json_file.read()
        json_file.close()
        print("---------------------Finished reading json-------------------------")

        print("-----------------------Load model from json-file-------------------------")
        model = tf.keras.models.model_from_json(loaded_model_json)

        # load weights into new model
        print("----------------------Load model.h5-file------------------------")
        model.load_weights(os.path.join(model_folder,'model30-06-2020_20:00.h5'))
        print("---------------------------Loaded model from disk------------------------")
        tf.keras.backend.set_learning_phase(0)
    print(model.summary())

def on_crash(crash_message):
    print("--------------reset car--------------")
    reset_car.reset(progress=0)

def set_keras_conf():
    print(device_lib.list_local_devices())

    tf.debugging.set_log_device_placement(True)

def test():
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
    print("executing db-query")
    query = '''select td.speed || lv.values as values from training_data as td 
                join (
                    select ts, array_agg(value order by value_id asc) as values
                        from (
                        select ts, value, value_id
                        from training_data_laser_values
                        ) t
                        group by ts
                        order by ts) as lv 
                on td.ts = lv.ts 
                order by td.ts desc'''

    cursor.execute(query)
    records = np.asarray(cursor.fetchall(), dtype=np.float32) 

    records = np.squeeze(records, axis=1)
    np_values = np.asarray(records, dtype=np.float32)

    with tf.device('/gpu:0'):
        prediction = model.predict(np_values)
    
    print(prediction)

set_keras_conf()

load_model()

test()

rospy.init_node('drive_nn', anonymous=True)

reset_car.register_service()

rospy.Subscriber(TOPIC_CRASH, Empty, on_crash)

if(voxel):
    rospy.Subscriber(TOPIC_VOXEL, PointCloud2, voxel_callback)
else:
    rospy.Subscriber(TOPIC_LASER_SCAN, LaserScan, laser_callback)
rospy.Subscriber(TOPIC_STATE_TELEMETRY, gazebo_state_telemetry, state_telemetry_callback)

drive_parameters_publisher = rospy.Publisher(
    TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()