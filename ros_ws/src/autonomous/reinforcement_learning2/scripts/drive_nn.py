#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
import sys
import numpy
import rospy
import datetime
from drive_msgs.msg import drive_param
from std_msgs.msg import Empty
from drive_msgs.msg import gazebo_state_telemetry
from tensorflow.keras import models, backend
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import os, rospkg
from utilities import createDBVoxelArray


import simulation_tools.reset_car as reset_car

rospack = rospkg.RosPack()

numpy.set_printoptions(threshold=sys.maxsize)

TOPIC_CRASH = "/crash"
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_STATE_TELEMETRY = "/gazebo/state_telemetry"
TOPIC_VOXEL = "/scan/voxels"

last_state_telemetry_message = None

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




def load_model():
    print("Start Loading model from disk")
    global model
    model_folder =os.path.join(rospack.get_path("reinforcement_learning2"), "model")

    json_file = open(os.path.join(model_folder,'model13-03-2020_09:22.json'), 'r')
    loaded_model_json = json_file.read()
    json_file.close()

    print("Load model.json-file")
    model = models.model_from_json(loaded_model_json)

    # load weights into new model
    print("Load model.h5-file")
    model.load_weights(os.path.join(model_folder,'model13-03-2020_09:22.h5'))
    print("Loaded model from disk")
    backend.set_learning_phase(0)
    #model.compile()

def on_crash(crash_message):
    print("--------------reset car--------------")
    reset_car.reset(progress=0.25)

load_model()

rospy.init_node('drive_nn', anonymous=True)

reset_car.register_service()

rospy.Subscriber(TOPIC_CRASH, Empty, on_crash)

rospy.Subscriber(TOPIC_VOXEL, PointCloud2, voxel_callback, queue_size=1)
rospy.Subscriber(TOPIC_STATE_TELEMETRY, gazebo_state_telemetry, state_telemetry_callback)
drive_parameters_publisher = rospy.Publisher(TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()