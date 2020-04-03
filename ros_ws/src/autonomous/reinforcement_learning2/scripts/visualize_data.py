import tensorflow as tf
import matplotlib.pyplot as plt
import psycopg2
import numpy as np
import datetime
import sys
import glob
import os
import keras.backend as K
from tensorflow.keras import datasets, layers, models, Input, Model, optimizers
import cv2
import matplotlib.pyplot as plt 


picture_size = 71
batch_size=32
epochs=100


def connect_to_database():
    try:
        global connection
        connection = psycopg2.connect(user = "postgres",
                                    password = "R5wo4ZWA",
                                    host = "52.157.178.55",
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

def get_training_data_from_db():
    print("executing db-query")
    query = "select speed,calc_velocity,calc_angle from training_data order by ts asc"
    #query = "select calc_velocity,calc_angle from training_data"

    cursor.execute(query)
    print("Selecting rows from training_data table")
    records = np.asarray(cursor.fetchall(), dtype=np.float32)

    #normalization
    stdev =np.asarray([1.547,1.446,0.262])
    avg =np.asarray([4.853,4.603,-0.094])
    records = (records - avg) /stdev
    #np.save("/home/marvin/db_data.npy",records)
    return records

def get_training_data_from_png():
    print("reading pictures")
    np.set_printoptions(threshold=sys.maxsize)
    data_list = []
    for im_path in sorted(glob.glob("/home/Marvin/training_data/ar-tu-do_71x71/ar-tu-do_71x71/*.png"),key = lambda date: datetime.datetime.strptime(date[-30:-4], '%d-%m-%Y %H:%M:%S.%f')):
        data_list.append(cv2.imread(im_path)[:,:,0])
        #print(im_path)
        #print(plt.imread(im_path)[:,:,0])
        #print(im_path)
    
    np_array = np.asarray(data_list, dtype=np.float32)
    #np.save("/home/marvin/pictures.npy",np_array)
    return np_array

training_data_from_db = get_training_data_from_db()
