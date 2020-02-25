
from sensor_msgs.msg import PointCloud2
import sys
import numpy
import rospy
from drive_msgs.msg import drive_param


TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_VOXEL = "/scan/voxels"

def voxel_callback(voxel_message):    
    prediction = model.predict(voxel_message)
    message = drive_param()
    message.angle = prediction[0] 
    message.velocity = prediction[1] 
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
    json_file = open('model.json', 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    global model = model_from_json(loaded_model_json)
    # load weights into new model
    model.load_weights("model.h5")
    print("Loaded model from disk")


rospy.Subscriber(TOPIC_VOXEL, PointCloud2, voxel_callback)
drive_parameters_publisher = rospy.Publisher(TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

load_model()