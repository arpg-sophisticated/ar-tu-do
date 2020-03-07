#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
import sys
import numpy
import rospy
from drive_msgs.msg import drive_param
from drive_msgs.msg import gazebo_state_telemetry
from tensorflow.keras import models
import sensor_msgs.point_cloud2 as pc2
import numpy as np


TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_STATE_TELEMETRY = "/gazebo/state_telemetry"
TOPIC_VOXEL = "/scan/voxels"
voxel_resolution = 0.1

last_state_telemetry_message = None

def voxel_callback(voxel_message):  
    voxel_arry = createDBVoxelArray(voxel_message)
    voxel_arry = np.expand_dims(voxel_arry ,-1)
    voxel_arry = np.expand_dims(voxel_arry ,0)

    if(last_state_telemetry_message is not None):
        wheel_speed = last_state_telemetry_message.wheel_speed
    else:
        wheel_speed =0
    print(wheel_speed)
    wheel_speed = np.array([wheel_speed])
    prediction = model.predict([voxel_arry,wheel_speed])
    message = drive_param()
    message.velocity = prediction[0,0]/20 
    message.angle = prediction[0,1]/100

    drive_parameters_publisher.publish(message)

def state_telemetry_callback(state_telemetry_message):
    global last_state_telemetry_message
    last_state_telemetry_message = state_telemetry_message


def createDBVoxelArray(voxel_message):

    voxel_array_size = 71
    voxel = numpy.zeros((voxel_array_size, voxel_array_size), dtype=bool)

    p_gen = pc2.read_points(voxel_message, field_names = ("x", "y", "z", "score"), skip_nans=True)

    for p in p_gen:
        x= p[0]
        y= p[1]
        z= p[2]
        score= p[3]
        x_index = int(round(x/voxel_resolution +((voxel_array_size-1)/2)))
        y_index = int(round(y/voxel_resolution +((voxel_array_size-1)/2)))

        shift_view = int(round(voxel_array_size/3)) #Lidar schaut nicht nach hinten. Voxel array kann verschoben werden
        x_index = x_index - shift_view

        #print ("x: "+str(x) +"  y: "+str(y))
        #print ("x_index: "+str(x_index) +"  y_index: "+str(y_index))
        if (x_index<voxel_array_size) and (y_index<voxel_array_size) and (x_index>=0) and (y_index>=0):
            voxel[x_index,y_index] = 1        #TODO bisher nicht wirklich effizient
   
    return voxel

def load_model():
    global model
    json_file = open('/home/marvin/code/ar-tu-do/ros_ws/model06-03-2020_19:10.json', 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    model = models.model_from_json(loaded_model_json)
    # load weights into new model
    model.load_weights("/home/marvin/code/ar-tu-do/ros_ws/model06-03-2020_19:10.h5")
    print("Loaded model from disk")


load_model()

rospy.init_node('drive_nn', anonymous=True)

rospy.Subscriber(TOPIC_VOXEL, PointCloud2, voxel_callback)
rospy.Subscriber(TOPIC_STATE_TELEMETRY, gazebo_state_telemetry, state_telemetry_callback)
drive_parameters_publisher = rospy.Publisher(TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()