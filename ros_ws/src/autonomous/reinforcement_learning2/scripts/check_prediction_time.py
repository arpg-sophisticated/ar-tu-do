#!/usr/bin/env python

import sys
import numpy
import datetime
from tensorflow.keras import models, backend
import numpy as np
import os
from random import randint
import glob
import cv2

numpy.set_printoptions(threshold=sys.maxsize)


TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_STATE_TELEMETRY = "/gazebo/state_telemetry"
TOPIC_VOXEL = "/scan/voxels"
voxel_resolution = 0.1

last_state_telemetry_message = None

def loop_some_predictions():
    training_data_from_png = get_training_data_from_png()
    training_data_from_png=np.expand_dims(training_data_from_png ,-1)
    print (training_data_from_png.shape)
    voxel_arry = []
    #voxel_arry = np.expand_dims(voxel_arry ,-1)
    #voxel_arry = np.expand_dims(voxel_arry ,0)

    for i in range(100):
        random = randint(0,30000)
        test=training_data_from_png[random,:,:,:]
        voxel_arry.append(test)
    voxel_arry = np.asarray(voxel_arry)
    print (voxel_arry.shape)
    for i in range(100):
        print (str(i)+": "+str(datetime.datetime.now()))   
        sample = voxel_arry[i,:,:,:]
        print( sample.shape)   
        sample = np.expand_dims(sample ,0)
        print( sample.shape)   

        speed = np.array([5])

        prediction = model.predict([sample,speed],use_multiprocessing=True)


def get_training_data_from_png():
    print("reading pictures")
    np.set_printoptions(threshold=sys.maxsize)
    data_list = []
    for im_path in sorted(glob.glob("/home/Marvin/training_data/ar-tu-do_71x71/ar-tu-do_71x71/17-03-2020*.png"),key = lambda date: datetime.datetime.strptime(date[-30:-4], '%d-%m-%Y %H:%M:%S.%f')):
        data_list.append(cv2.imread(im_path)[:,:,0])
        #print(im_path)
        #print(plt.imread(im_path)[:,:,0])
        #print(im_path)
    
    np_array = np.asarray(data_list, dtype=np.float32)
    #np.save("/home/marvin/pictures.npy",np_array)
    return np_array


def load_model():
    print("Start Loading model from disk")
    global model
    model_folder = '/home/Marvin/model'
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




load_model()

loop_some_predictions()
