import tensorflow as tf
import matplotlib.pyplot as plt
import psycopg2
import numpy as np
import datetime
import sys
import glob
import keras.backend as K
from tensorflow.keras import datasets, layers, models

def connect_to_database():
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

def diff_pred(y_true, y_pred):
    return K.mean(abs(y_true-y_pred))


def build_model():
    model = models.Sequential()
    model.add(layers.Conv2D(17, (7, 7), activation='relu', input_shape=(21, 21, 1)))
    model.add(layers.MaxPooling2D(pool_size=(3, 3),strides=(1,1)))
    model.add(layers.Conv2D(17, (13, 13), activation='relu'))
    model.add(layers.Reshape((1,17)))
    model.add(layers.Dense(17, input_shape=(1,17)))
    model.add(layers.Dense(1, input_shape=(1,17),activation='linear'))

    model.build()
    print(model.summary())

    model.compile(optimizer='adam',loss='mean_squared_error',
              metrics=[diff_pred])
    return model

def train_model_from_database():   

    model = build_model()

    connect_to_database()

    training_data_from_db = get_training_data_from_db()
    training_data_from_png = get_training_data_from_png()

    numpy_array = np.array([])

    trainX=np.expand_dims(training_data_from_png ,-1)
    trainY=np.expand_dims(training_data_from_db,-2)
    print("Shape of Training-Data-x:")
    print(trainX.shape)
    print("Shape of Training-Data-y:")
    print(trainY.shape)
    print("train model")
    model.fit(trainX, trainY, batch_size=32, epochs=50)
    save_model(model)
    
def save_model(model):
    # serialize model to JSON
    model_json = model.to_json()

    current_time = datetime.datetime.now()
    current_time_formated = current_time.strftime("%d-%m-%Y_%H:%M")

    with open("model"+current_time_formated+".json", "w") as json_file:
        json_file.write(model_json)
    # serialize weights to HDF5
    model.save_weights("model"+current_time_formated+".h5")
    print("Saved model to disk")

def get_training_data_from_db():
    print("executing db-query")
    query = "select calc_angle from training_data"
    #query = "select calc_velocity,calc_angle from training_data"

    cursor.execute(query)
    print("Selecting rows from training_data table")
    records = np.asarray(cursor.fetchall(), dtype=np.float32)
    return records

def get_training_data_from_png():
    print("reading pictures")

    data_list = []
    for im_path in glob.glob("/home/marvin/Pictures/*.png"):
        data_list.append(plt.imread(im_path)[:,:,0])
    
    np_array = np.asarray(data_list, dtype=np.float32)
    return np_array
train_model_from_database()