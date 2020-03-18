import tensorflow as tf
import matplotlib.pyplot as plt
import psycopg2
import numpy as np
import datetime
import sys
import glob
import os
import keras.backend as K
from tensorflow.keras import datasets, layers, models, Input, Model

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

weights = K.variable(value=np.array([[10, 100]]))

def custom_loss(y_true, y_pred):
    y_true = tf.matmul(y_true,tf.transpose(weights))
    y_pred = tf.matmul(y_pred,tf.transpose(weights))
    return K.square(y_true - y_pred)

def build_model():
    inputPNG = Input(shape=(21, 21, 1))
    inputNumeric = Input(shape=(1,))

    png_branch = layers.Conv2D(17, (7, 7), activation='relu')(inputPNG)
    png_branch = layers.MaxPooling2D(pool_size=(3, 3),strides=(1,1))(png_branch)
    png_branch = layers.Conv2D(17, (13, 13), activation='relu')(png_branch)
    png_branch = layers.Reshape((17,))(png_branch)
    png_branch = Model(inputs=inputPNG,outputs=png_branch)

    numeric_branch = layers.Dense(3)(inputNumeric)
    numeric_branch = Model(inputs=inputNumeric,outputs=numeric_branch)

    combined_branch = layers.concatenate([png_branch.output,numeric_branch.output])
    combined_branch = layers.Dense(20)(combined_branch)
    combined_branch = layers.Dense(2)(combined_branch)

    model = Model(inputs=[png_branch.input,numeric_branch.input],outputs=combined_branch)

    print(model.summary())

    model.compile(optimizer='adam',loss='mean_squared_error', metrics=[diff_pred,diff_velocity,diff_angle])
    return model

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

def build_model3():
    inputPNG = Input(shape=(picture_size, picture_size, 1))
    inputNumeric = Input(shape=(1,))

    png_branch = layers.Conv2D(11, (25, 25), activation='relu')(inputPNG)
    png_branch = layers.MaxPooling2D(pool_size=(5, 5),strides=(2,2))(png_branch)
    png_branch = layers.Conv2D(3, (15, 15), activation='relu')(png_branch)
    png_branch = layers.MaxPooling2D(pool_size=(3, 3),strides=(1,1))(png_branch)
    png_branch = layers.Flatten()(png_branch)
    png_branch = layers.Dense(20, activation='relu')(png_branch)
    png_branch = Model(inputs=inputPNG,outputs=png_branch)

    png_branch2 = layers.Conv2D(17, (7, 7), activation='relu')(inputPNG)
    png_branch2 = layers.MaxPooling2D(pool_size=(3, 3),strides=(1,1))(png_branch2)
    png_branch2 = layers.Conv2D(17, (13, 13), activation='relu')(png_branch2)
    png_branch2 = layers.Flatten()(png_branch2)
    png_branch2 = layers.Dense(20)(png_branch2)
    png_branch2 = Model(inputs=inputPNG,outputs=png_branch2)

    numeric_branch = layers.Dense(5)(inputNumeric)
    numeric_branch = Model(inputs=inputNumeric,outputs=numeric_branch)

    combined_branch = layers.concatenate([png_branch.output,png_branch2.output,numeric_branch.output])
    combined_branch = layers.Dense(25, activation='relu')(combined_branch)
    combined_branch = layers.Dense(25, activation='relu')(combined_branch)
    combined_branch = layers.Dense(2)(combined_branch)

    model = Model(inputs=[png_branch.input,numeric_branch.input],outputs=combined_branch)

    print(model.summary())

    model.compile(optimizer='adam',loss='mean_squared_error', metrics=[diff_pred,diff_velocity,diff_angle])
    return model

def build_model4():
    inputPNG = Input(shape=(picture_size, picture_size, 1))
    inputNumeric = Input(shape=(1,))

    png_branch = layers.Conv2D(7, (30, 30), activation='relu')(inputPNG)
    png_branch = layers.MaxPooling2D(pool_size=( 3,3),strides=(2,2))(png_branch)
    png_branch = layers.Flatten()(png_branch)
    png_branch = layers.Dense(20, activation='relu')(png_branch)
    png_branch = Model(inputs=inputPNG,outputs=png_branch)

    png_branch2 = layers.Conv2D(7, (6, 6), activation='relu')(inputPNG)
    png_branch2 = layers.MaxPooling2D(pool_size=(3,3),strides=(2,2))(png_branch2)
    png_branch2 = layers.Conv2D(4, (3, 3), activation='relu')(png_branch2)
    png_branch2 = layers.MaxPooling2D(pool_size=(2,2),strides=(1,1))(png_branch2)
    png_branch2 = layers.Flatten()(png_branch2)
    png_branch2 = layers.Dense(20)(png_branch2)
    png_branch2 = Model(inputs=inputPNG,outputs=png_branch2)

    numeric_branch = layers.Dense(5)(inputNumeric)
    numeric_branch = Model(inputs=inputNumeric,outputs=numeric_branch)

    combined_branch = layers.concatenate([png_branch.output,png_branch2.output,numeric_branch.output])
    combined_branch = layers.Dense(25, activation='relu')(combined_branch)
    combined_branch = layers.Dense(2)(combined_branch)

    model = Model(inputs=[png_branch.input,numeric_branch.input],outputs=combined_branch)

    print(model.summary())

    model.compile(optimizer='adam',loss='mean_squared_error', metrics=[diff_pred,diff_velocity,diff_angle])
    return model

def build_model5():
    inputPNG = Input(shape=(picture_size, picture_size, 1))
    inputNumeric = Input(shape=(1,))

    png_branch = layers.Conv2D(7, (30, 30), activation='relu')(inputPNG)
    png_branch = layers.MaxPooling2D(pool_size=( 3,3),strides=(2,2))(png_branch)

    png_branch = layers.Conv2D(1, (5, 5), activation='relu')(png_branch)
    png_branch = layers.MaxPooling2D(pool_size=(3,3),strides=(2,2))(png_branch)

    png_branch = layers.Flatten()(png_branch)
    png_branch = layers.Dense(20, activation='relu')(png_branch)
    png_branch = Model(inputs=inputPNG,outputs=png_branch)

    numeric_branch = layers.Dense(5)(inputNumeric)
    numeric_branch = Model(inputs=inputNumeric,outputs=numeric_branch)

    combined_branch = layers.concatenate([png_branch.output,numeric_branch.output])
    combined_branch = layers.Dense(10, activation='relu')(combined_branch)
    combined_branch = layers.Dense(2)(combined_branch)

    model = Model(inputs=[png_branch.input,numeric_branch.input],outputs=combined_branch)

    print(model.summary())

    model.compile(optimizer='adam',loss='mean_squared_error', metrics=[diff_pred,diff_velocity,diff_angle])
    return model


def train_model():   

    model = build_model_gut()

    connect_to_database()

    training_data_from_db = get_training_data_from_db()

    training_data_from_png = get_training_data_from_png()

    training_data_numeric = training_data_from_db[:,0]
    training_data_label = training_data_from_db[:,1:]


    trainXPNG = np.expand_dims(training_data_from_png ,-1)
    trainXNumeric = np.expand_dims(training_data_numeric ,-1)

    # shuffle
    p = np.random.permutation(len(training_data_label))

    training_data_label = training_data_label[p]
    trainXPNG = trainXPNG[p]
    trainXNumeric = trainXNumeric[p]

    print("Shape of Training-Data-x-png:")
    print(trainXPNG.shape)
    print("Shape of Training-Data-x-numeric:")
    print(trainXNumeric.shape)
    print("Shape of Training-Data-y:")
    print(training_data_label.shape)
    print("train model")
    model.fit([trainXPNG,trainXNumeric], training_data_label, batch_size=batch_size, epochs=epochs)
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
    query = "select speed,calc_velocity,calc_angle from training_data order by ts asc"
    #query = "select calc_velocity,calc_angle from training_data"

    cursor.execute(query)
    print("Selecting rows from training_data table")
    records = np.asarray(cursor.fetchall(), dtype=np.float32)

    #normalization
    stdev =np.asarray([1.547,1.446,0.262])
    avg =np.asarray([4.853,4.603,-0.094])
    records = (records - avg) /stdev

    return records

def get_training_data_from_png():
    print("reading pictures")
    np.set_printoptions(threshold=sys.maxsize)
    data_list = []
    for im_path in sorted(glob.glob("/home/marvin/Pictures/*.png"),key=os.path.getmtime):
        data_list.append(plt.imread(im_path)[:,:,0])
        #print(plt.imread(im_path)[:,:,0])
        #print(im_path)
    
    np_array = np.asarray(data_list, dtype=np.float32)
    
    return np_array

train_model()
#TODO: 
# 
# nn optimieren
#   angle und velocity normalisieren
# mehr daten aufzeichnen
# zuordnung von drive-param zu voxel bei store