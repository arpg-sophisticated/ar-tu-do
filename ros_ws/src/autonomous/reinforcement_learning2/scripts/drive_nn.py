#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
import sys
import numpy
import rospy
from drive_msgs.msg import drive_param
from tensorflow.keras import models
import sensor_msgs.point_cloud2 as pc2
import numpy as np


TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_VOXEL = "/scan/voxels"
voxel_resolution = 0.2


def voxel_callback(voxel_message):    
    voxel_arry = createDBVoxelArray(voxel_message)
    voxel_arry = np.expand_dims(voxel_arry ,-1)
    voxel_arry = np.expand_dims(voxel_arry ,0)
    prediction = model.predict(voxel_arry)
    message = drive_param()
    message.velocity = prediction[0,0,0] 
    message.angle = prediction[0,0,1]

    drive_parameters_publisher.publish(message)

def createDBVoxelArray(voxel_message):

    voxel_array_size = 21
    voxel = numpy.zeros((voxel_array_size, voxel_array_size), dtype=bool)

    p_gen = pc2.read_points(voxel_message, field_names = ("x", "y", "z", "score"), skip_nans=True)

    for p in p_gen:
        x= p[0]
        y= p[1]
        z= p[2]
        score= p[3]
        x_index = x/voxel_resolution +((voxel_array_size-1)/2)
        y_index = y/voxel_resolution +((voxel_array_size-1)/2)
        if (abs(x_index)<voxel_array_size and abs(y_index)<voxel_array_size):
            voxel[int(x_index),int(y_index)] = 1        #TODO bisher nicht wirklich effizient

    return voxel

def load_model():
    global model
    json_file = open('/home/marvin/code/ar-tu-do/ros_ws/model26-02-2020_08:28.json', 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    model = models.model_from_json(loaded_model_json)
    # load weights into new model
    model.load_weights("/home/marvin/code/ar-tu-do/ros_ws/model26-02-2020_08:28.h5")
    print("Loaded model from disk")


load_model()

rospy.init_node('drive_nn', anonymous=True)

rospy.Subscriber(TOPIC_VOXEL, PointCloud2, voxel_callback)
drive_parameters_publisher = rospy.Publisher(TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()