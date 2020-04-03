#!/usr/bin/env python

import numpy as np
import sensor_msgs.point_cloud2 as pc2


voxel_resolution = 0.1

def createDBVoxelArray(voxel_message):

    voxel_array_size = 71
    voxel = np.zeros((voxel_array_size, voxel_array_size), dtype=bool)

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