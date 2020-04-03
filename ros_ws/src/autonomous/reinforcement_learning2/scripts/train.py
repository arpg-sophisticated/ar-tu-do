import tensorflow as tf
import matplotlib.pyplot as plt
import psycopg2
import numpy as np
import datetime
import sys
import glob
import os
import keras.backend as K
import autokeras as ak
from tensorflow.keras import datasets, layers, models, Input, Model, optimizers
import cv2

picture_size = 71
batch_size=32
epochs=100


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
def diff_velocity(y_true, y_pred):
    return K.mean(abs((y_true[0]) -(y_pred[0])))
def diff_angle(y_true, y_pred):
    return K.mean(abs((y_true[1])-(y_pred[1])))

def custom_loss(y_true, y_pred):
    y_true = tf.matmul(y_true,tf.transpose(weights))
    y_pred = tf.matmul(y_pred,tf.transpose(weights))
    return K.square(y_true - y_pred)

def build_model_gut():
    inputPNG = Input(shape=(picture_size, picture_size, 1))
    inputNumeric = Input(shape=(1,))

    png_branch = layers.Conv2D(7, (25, 25), activation='relu')(inputPNG)
    png_branch = layers.Flatten()(png_branch)
    png_branch = layers.Dense(20, activation='relu')(png_branch)
    png_branch = Model(inputs=inputPNG,outputs=png_branch)

    numeric_branch = layers.Dense(5)(inputNumeric)
    numeric_branch = Model(inputs=inputNumeric,outputs=numeric_branch)

    combined_branch = layers.concatenate([png_branch.output,numeric_branch.output])
    combined_branch = layers.Dense(25, activation='relu')(combined_branch)
    combined_branch = layers.Dense(2)(combined_branch)

    model = Model(inputs=[png_branch.input,numeric_branch.input],outputs=combined_branch)

    print(model.summary())

    model.compile(optimizer='adam',loss='mean_squared_error', metrics=[diff_pred,diff_velocity,diff_angle])
    return model

def build_model_new():
    inputPNG = Input(shape=(picture_size, picture_size, 1))
    inputNumeric = Input(shape=(1,))

    png_branch = layers.Conv2D(16, (32, 32), activation='relu')(inputPNG)
    png_branch = layers.MaxPooling2D(pool_size=( 4,4),strides=(1,1))(png_branch)

    png_branch = layers.Conv2D(32, (8, 8), activation='relu')(png_branch)
    png_branch = layers.MaxPooling2D(pool_size=(4,4),strides=(1,1))(png_branch)
    
    png_branch = layers.Conv2D(64, (4, 4), activation='relu')(png_branch)
    png_branch = layers.MaxPooling2D(pool_size=(4,4),strides=(1,1))(png_branch)

    png_branch = layers.Flatten()(png_branch)
    png_branch = layers.Dense(256, activation='relu')(png_branch)
    png_branch = Model(inputs=inputPNG,outputs=png_branch)

    numeric_branch = layers.Dense(8)(inputNumeric)
    numeric_branch = Model(inputs=inputNumeric,outputs=numeric_branch)

    combined_branch = layers.concatenate([png_branch.output,numeric_branch.output])
    combined_branch = layers.Dense(128, activation='relu')(combined_branch)
    combined_branch = layers.Dense(2)(combined_branch)

    model = Model(inputs=[png_branch.input,numeric_branch.input],outputs=combined_branch)

    print(model.summary())

    #adam=optimizers.Adam(learning_rate=0.01, beta_1=0.9, beta_2=0.999, epsilon=1e-07, amsgrad=False, name='Adam')

    model.compile(optimizer='adam',loss='mean_squared_error', metrics=[diff_pred,diff_velocity,diff_angle])
    return model

def get_training_data():
    connect_to_database()

    training_data_from_db = get_training_data_from_db()

    training_data_from_png = get_training_data_from_png()

    training_data_numeric = training_data_from_db[:,0]
    training_data_label = training_data_from_db[:,1:]

    #boost difficult curve 265 - 375
    print ('png'+str(training_data_from_png.shape))
    print ('numeric'+str(training_data_numeric.shape))
    print ('label'+str(training_data_label.shape))
    for i in range(20):
        training_data_from_png = np.concatenate((training_data_from_png,training_data_from_png[265:375]))
        training_data_numeric = np.concatenate((training_data_numeric,training_data_numeric[265:375]))
        training_data_label = np.concatenate((training_data_label,training_data_label[265:375]))

    #prep for model
    trainXPNG = np.expand_dims(training_data_from_png ,-1)
    trainXNumeric = np.expand_dims(training_data_numeric ,-1)

    # shuffle
    p = np.random.permutation(len(training_data_label))

    training_data_label = training_data_label[p]
    trainXPNG = trainXPNG[p]
    trainXNumeric = trainXNumeric[p]

    return (training_data_label,trainXPNG,trainXNumeric)

def train_model():   

    model = build_model_gut()

    training_data = get_training_data()

    training_data_label = training_data[0]
    trainXPNG = training_data[1]
    trainXNumeric = training_data[2]

    print("Shape of Training-Data-x-png:")
    print(trainXPNG.shape)
    print("Shape of Training-Data-x-numeric:")
    print(trainXNumeric.shape)
    print("Shape of Training-Data-y:")
    print(training_data_label.shape)
    print("train model")
    model.fit([trainXPNG,trainXNumeric], training_data_label, batch_size=batch_size, epochs=epochs)
    save_model(model)

def train_model_automl():
    training_data = get_training_data()

    training_data_label = training_data[0]
    trainXPNG = training_data[1]
    trainXNumeric = training_data[2]

    trainX = [trainXPNG,trainXNumeric]

    am = ak.AutoModel(
    inputs=[ak.ImageInput(), ak.StructuredDataInput()],
    outputs=[
        ak.RegressionHead(metrics=['mse']),
        ak.RegressionHead(metrics=['mse'])
    ],
    max_trials=10)

    am.fit(trainX, [training_data_label[:,0],training_data_label[:,1]], epochs=10)
    save_model(am.export_model())
    
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
    for im_path in sorted(glob.glob("/home/marvin/training_data/ar-tu-do_71x71/*.png"),key = lambda date: datetime.datetime.strptime(date[-30:-4], '%d-%m-%Y %H:%M:%S.%f')):
        data_list.append(cv2.imread(im_path)[:,:,0])
        #print(im_path)
        #print(plt.imread(im_path)[:,:,0])
        #print(im_path)
    
    np_array = np.asarray(data_list, dtype=np.float32)
    #np.save("/home/marvin/pictures.npy",np_array)
    return np_array

train_model_automl()
#TODO: 
# 
# nn optimieren
#   angle und velocity normalisieren
# mehr daten aufzeichnen
# zuordnung von drive-param zu voxel bei store