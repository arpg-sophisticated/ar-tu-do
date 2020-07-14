import tensorflow as tf
import matplotlib.pyplot as plt
import psycopg2
import numpy as np
import datetime
import sys
import glob
import os
import tensorflow.keras.backend as K
from tensorflow.keras import datasets, layers, models, Input, Model, optimizers
from scipy import misc

picture_size = 71
batch_size=32
epochs=100

voxel = False

laser_sample_count = 128

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

def build_model_old():
    inputPNG = Input(shape=(picture_size, picture_size, 1))
    inputNumeric = Input(shape=(1,))

    png_branch = layers.Conv2D(7, (25, 25), activation='tanh')(inputPNG)
    png_branch = layers.Flatten()(png_branch)
    png_branch = layers.Dense(20, activation='tanh')(png_branch)
    png_branch = Model(inputs=inputPNG,outputs=png_branch)

    numeric_branch = layers.Dense(5)(inputNumeric)
    numeric_branch = Model(inputs=inputNumeric,outputs=numeric_branch)

    combined_branch = layers.concatenate([png_branch.output,numeric_branch.output])
    combined_branch = layers.Dense(25, activation='tanh')(combined_branch)
    combined_branch = layers.Dense(1)(combined_branch)

    model = Model(inputs=[png_branch.input,numeric_branch.input],outputs=combined_branch)

    print(model.summary())

    #model.compile(optimizer='adam',loss='mean_squared_error', metrics=[diff_pred,diff_velocity,diff_angle])

    model.compile(optimizer='adam',loss='mean_squared_error')
    
    return model

def build_model_voxel():
    inputPNG = Input(shape=(picture_size, picture_size, 1), dtype=np.float16)
    inputNumeric = Input(shape=(1,))

    png_branch = layers.Conv2D(16, (32, 32), activation='relu')(inputPNG)
    png_branch = layers.MaxPooling2D(pool_size=( 4,4),strides=(1,1))(png_branch)

    png_branch = layers.Conv2D(32, (8, 8), activation='relu')(png_branch)
    png_branch = layers.MaxPooling2D(pool_size=(4,4),strides=(1,1))(png_branch)
    
    png_branch = layers.Conv2D(64, (4, 4), activation='relu')(png_branch)
    png_branch = layers.MaxPooling2D(pool_size=(4,4),strides=(1,1))(png_branch)

    png_branch = layers.Flatten()(png_branch)
    png_branch = layers.Dense(64, activation='relu')(png_branch)
    png_branch = Model(inputs=inputPNG,outputs=png_branch)

    numeric_branch = layers.Dense(8, activation='relu')(inputNumeric)
    numeric_branch = Model(inputs=inputNumeric,outputs=numeric_branch)

    combined_branch = layers.concatenate([png_branch.output,numeric_branch.output])
    combined_branch = layers.Dense(32, activation='relu')(combined_branch)
    combined_branch = layers.Dense(2, activation='tanh')(combined_branch)

    model = Model(inputs=[png_branch.input,numeric_branch.input],outputs=combined_branch)

    print(model.summary())

    adam=optimizers.Adam(learning_rate=0.001)

    #adam=optimizers.Adam(learning_rate=0.01, beta_1=0.9, beta_2=0.999, epsilon=1e-07, amsgrad=False, name='Adam')

    model.compile(optimizer=adam,loss='mean_squared_error')
    return model


def build_model_laser_scan():
    #inputNumeric = Input(shape=(laser_sample_count+1,))
    inputNumeric = Input(shape=(laser_sample_count,))

    numeric_branch = layers.Dense(64, activation='tanh')(inputNumeric)
    numeric_branch = layers.Dense(32, activation='tanh')(numeric_branch)
    #numeric_branch = layers.Dense(2, activation='tanh')(numeric_branch)
    numeric_branch = layers.Dense(1, activation='tanh')(numeric_branch)

    model = Model(inputs=inputNumeric,outputs=numeric_branch)

    print(model.summary())

    #adam=optimizers.Adam(learning_rate=0.01, beta_1=0.9, beta_2=0.999, epsilon=1e-07, amsgrad=False, name='Adam')
    adam=optimizers.Adam(learning_rate=0.001)

    #model.compile(optimizer='adam',loss='mean_squared_error', metrics=[diff_pred,diff_velocity,diff_angle])
    model.compile(optimizer=adam,loss='mean_squared_error')
    return model


def get_training_data_voxel():
    connect_to_database()

    training_data_from_db = get_training_data_from_db_voxel()

    training_data_from_png = get_training_data_from_png()

    training_data_numeric = training_data_from_db[:,0]
    training_data_label = training_data_from_db[:,1:]

    print ('png'+str(training_data_from_png.shape))
    print ('numeric'+str(training_data_numeric.shape))
    print ('label'+str(training_data_label.shape))

    #prep for model
    print('data preperation ...')
    trainXPNG = np.expand_dims(training_data_from_png ,-1)
    trainXNumeric = np.expand_dims(training_data_numeric ,-1)

    # shuffle
    print('data shuffling ...')
    p = np.random.permutation(len(training_data_label))
    training_data_label = training_data_label[p]
    trainXPNG = trainXPNG[p]
    trainXNumeric = trainXNumeric[p]

    print('training data loaded succesfully')

    return (training_data_label,trainXPNG,trainXNumeric)

def train_model():      

    if(voxel):

        model = build_model_voxel()

        training_data = get_training_data_voxel()

        training_data_label = training_data[0]
        trainXPNG = training_data[1]
        trainXNumeric = training_data[2]

        print("Shape of Training-Data-x-png:")
        print(trainXPNG.shape)
        print("Shape of Training-Data-x-numeric:")
        print(trainXNumeric.shape)
        print("Shape of Training-Data-y:")
        print(training_data_label.shape)

        print("first entry:")
        print(trainXPNG[0])
        print(trainXNumeric[0])
        print(training_data_label[0])

        print("train model")    
        model.fit([trainXPNG,trainXNumeric], training_data_label, batch_size=batch_size, epochs=epochs)
    else:
        model = build_model_laser_scan()
        training_data = get_training_data_laser()
        training_data_label = get_training_data_label()
        print("Shape of Training-Data:")
        print(training_data.shape)
        print("Shape of Training-Data-y:")
        print(training_data_label.shape)
        model.fit(training_data, training_data_label, batch_size=batch_size, epochs=epochs)
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

def get_training_data_laser():
    connect_to_database()
    print("executing db-query")
    #query = '''select td.speed || lv.values as values from training_data as td 
    query = '''select lv.values as values from training_data as td 
                join (
                    select ts, array_agg(value order by value_id asc) as values
                        from (
                        select ts, value, value_id
                        from training_data_laser_values
                        ) t
                        group by ts
                        order by ts) as lv 
                on td.ts = lv.ts 
                order by td.ts desc'''

    cursor.execute(query)
    records = np.asarray(cursor.fetchall(), dtype=np.float32) 

    records = np.squeeze(records, axis=1)
    
    return records

def get_training_data_label():
    connect_to_database()
    print("executing db-query")
    #query = '''select ARRAY[calc_velocity/12 , calc_angle]  from training_data      
    query = '''select ARRAY[calc_angle]  from training_data           
                order by ts desc'''

    cursor.execute(query)
    records = np.asarray(cursor.fetchall(), dtype=np.float32)

    records = np.squeeze(records, axis=1)

    #normalization
    #stdev =np.asarray([1.547,1.446,0.262])
    #avg =np.asarray([4.853,4.603,-0.094])
    #records = (records - avg) /stdev
    #np.save("/home/marvin/db_data.npy",records)
    return records

def get_training_data_from_db_voxel():
    print("executing db-query")
    #query = "select speed,calc_angle from training_data_new order by ts asc"
    query = "select speed,calc_velocity,calc_angle from training_data_new order by ts asc"
    #query = "select calc_velocity,calc_angle from training_data"

    cursor.execute(query)

    records = np.asarray(cursor.fetchall(), dtype=np.float32)

    #normalization 
    # SELECT avg(speed) as avg_speed,avg(calc_velocity) as avg_velocity, avg(calc_angle) as avg_angle,  stddev(speed) as stddev_speed, stddev(calc_velocity) as stddev_velocity,stddev(calc_angle) as stddev_angle
	# FROM public.training_data_new; 
    stdev =np.asarray([1.773,1.552,0.681])
    avg =np.asarray([4.330,4.136,-0.342])
    records = (records - avg) /stdev
    #np.save("/home/marvin/db_data.npy",records)
    return records

def get_training_data_from_png():
    print("reading pictures")
    np.set_printoptions(threshold=sys.maxsize)
    list_of_np_array =[]
    i=0

    for im_path in sorted(glob.glob("/home/marvin/training_data/ar-tu-do_71x71_new/*.png"),key = lambda date: datetime.datetime.strptime(date[-30:-4], '%d-%m-%Y %H:%M:%S.%f')):
        i = i+1
        np_read = np.array(misc.imread(im_path)[:,:,0],dtype=bool)
        list_of_np_array.append(np_read)
        if(i%1000==0):
            print(i)         
        #print(im_path)
        #print(plt.imread(im_path)[:,:,0])
        #print(im_path)
    np_array= np.asarray(list_of_np_array, dtype=bool)
    print(np_array.shape)
    #np.save("/home/marvin/pictures.npy",np_array)
    return np_array

train_model()