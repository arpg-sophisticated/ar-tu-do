#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
import sys
import rospy
import datetime
from drive_msgs.msg import drive_param
from std_msgs.msg import Empty
from drive_msgs.msg import gazebo_state_telemetry
from tensorflow.keras import models, backend
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState, ModelStates
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import os, rospkg
from utilities import createDBVoxelArray
import simulation_tools.reset_car as reset_car
from simulation_tools.track import track
from simulation_tools.reset_car import Point
import queue
import random


rospack = rospkg.RosPack()

np.set_printoptions(threshold=sys.maxsize)

TOPIC_CRASH = "/crash"
TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_STATE_TELEMETRY = "/gazebo/state_telemetry"
TOPIC_VOXEL = "/scan/voxels"

voxel_resolution = 0.1
reset_amount =70

last_state_telemetry_message = None

def send_stop_message():
    message = drive_param()
    message.velocity = 0
    message.angle = 0
    drive_parameters_publisher.publish(message)

def voxel_callback(voxel_message):
    global wait_after_reset
    global in_reset_mode
    global on_training
    if(in_reset_mode>1):
        send_stop_message()
        in_reset_mode -=1
        print("."),
        return
    if(in_reset_mode==1):
        reset_car_to_reset_point()
        in_reset_mode -=1
    if(wait_after_reset >0):
        send_stop_message()
        wait_after_reset -=1
        return
    if(on_training):
        return

    check_for_training()
      
    #print (str(datetime.datetime.now())+" createVoxelArray")
    voxel_arry = createDBVoxelArray(voxel_message)
    voxel_arry = np.expand_dims(voxel_arry ,-1)
    voxel_arry = np.expand_dims(voxel_arry ,0)

    if(last_state_telemetry_message is not None):
        wheel_speed = last_state_telemetry_message.wheel_speed
    else:
        wheel_speed =0
    wheel_speed = np.array([(wheel_speed*1.547)+4.853])

    #print (str(datetime.datetime.now())+" predict")
    prediction = model.predict([voxel_arry,wheel_speed],use_multiprocessing=True)

    velocity_stdev=1.446
    angle_stdev=0.262
    velocity_avg=4.603
    angle_avg=-0.094

    message = drive_param()
    message.velocity = (prediction[0,0]*velocity_stdev)+velocity_avg
    message.angle = (prediction[0,1]*angle_stdev)+angle_avg

    add_state_to_queue(voxel_arry,wheel_speed,message)

    global callbacks_since_reset
    callbacks_since_reset +=1

    message.velocity = message.velocity*0.1 #drive more slowly because of prediction time
    

    #print("vel: "+ str(message.velocity) + ", angle: "+ str(message.angle))

    #print (str(datetime.datetime.now())+" publish")
    drive_parameters_publisher.publish(message)

def state_telemetry_callback(state_telemetry_message):
    global last_state_telemetry_message
    last_state_telemetry_message = state_telemetry_message


def load_model():
    print("Start Loading model from disk")
    global model
    model_folder =os.path.join(rospack.get_path("reinforcement_learning2"), "model")

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

def on_crash(crash_message):
    global in_reset_mode
    if(in_reset_mode ==0 and wait_after_reset ==0):
        in_reset_mode = 20
        print("Waiting for reset "),
    
def reset_car_to_reset_point(clear_queue=True, mutate=True,to_start=False):
    global last_reset_time
    global failing_part
    global callbacks_since_reset
    global farest_segment
    global reset_point
    global state_queue

    if(reset_point is None):
        reset_point = state_queue.get()[0]
        farest_segment = get_position_segment()
    failing_part = True
    callbacks_since_reset =0
    #avoid multiple reset
    if((datetime.datetime.now()-last_reset_time).total_seconds()>1):
        print("reset car")
        send_stop_message()
        if(to_start):
            reset_car.reset()
        else:
            reset_car.set_pose_from_get_model_state_response(reset_point)
        last_reset_time=datetime.datetime.now()
        if(clear_queue):
            with state_queue.mutex:
                state_queue.queue.clear()
        if(mutate):
            mutate_output_layer()
        global wait_after_reset
        wait_after_reset=50
        
def mutate_output_layer():
    print('mutate output layer')

    global output_layer_weights
    global output_layer_bias

    angle_manipulation = np.random.rand(output_layer_weights.shape[0]) #create random numpy array 0-1 (angle)
    angle_manipulation= (angle_manipulation - 0.5) / 2
    velocity_manipulation = np.zeros(output_layer_weights.shape[0]) # velocity

    manipulation = np.concatenate((velocity_manipulation,angle_manipulation),axis=0)
    manipulation = manipulation.reshape((output_layer_weights.shape[0],2))
    
    new_output_layer_weights = output_layer_weights + manipulation
    model.layers[-1].set_weights([new_output_layer_weights,output_layer_bias])


def check_for_training():
    global failing_part
    global farest_segment
    global output_layer_weights
    global output_layer_bias
    global reset_point
    global failing_part
   
    if(failing_part and callbacks_since_reset>=(reset_amount*2)):
        if(farest_segment < get_position_segment()):
            print("------passed fail part------")
            reset_car_to_reset_point(clear_queue=False,mutate=False,to_start=True)
            model.layers[-1].set_weights([output_layer_weights,output_layer_bias])
            train()
            output_layer_weights = model.layers[-1].get_weights()[0]
            output_layer_bias = model.layers[-1].get_weights()[1]
            failing_part = False
            reset_point = None
        else:
            print("-----too slow, reset!------")
            reset_car_to_reset_point()

def register_get_model_state():
    print("register get Model State")
    global get_model_state
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy(
        '/gazebo/get_model_state', GetModelState)

def train():
    print("--------------train from queue--------------")
    global on_training
    on_training =True
    global state_queue
    state_list = list(state_queue.queue)

    voxel_array_elements = [state[1] for state in state_list]
    wheel_speed_elements = [state[2] for state in state_list]
    message_elements = [[state[3].velocity,state[3].angle] for state in state_list]

    voxel_array_array = np.squeeze(np.asarray(voxel_array_elements),axis =1)
    wheel_speed_array = np.asarray(wheel_speed_elements)
    message_array = np.asarray(message_elements)


    model.compile(optimizer='adam',loss='mean_squared_error')
    model.fit([voxel_array_array,wheel_speed_array], message_array, batch_size=8, epochs=10)

    print("--------------training finished--------------")
    on_training =False
    with state_queue.mutex:
        state_queue.queue.clear()

def add_state_to_queue(voxel_arry,wheel_speed,message):
    global state_queue
    model_state_response = get_model_state('racer','')
    if(state_queue.full()):
        state_queue.get()
    state_queue.put((model_state_response,voxel_arry,wheel_speed,message))
    car_position =Point(model_state_response.pose.position.x,model_state_response.pose.position.y)

    print(track.localize(car_position))
    
def get_position_segment():
    model_state_response = get_model_state('racer','')
    return reset_car.localize(model_state_response.pose.position.x,model_state_response.pose.position.y).segment

def init_reset_car():
    global state_queue
    global reset_point_count
    global last_reset_time
    global reset_point
    reset_point = None
    last_reset_time=datetime.datetime.now()
    state_queue = queue.Queue(maxsize=reset_amount) 
    register_get_model_state()
    reset_car.register_service()

def init_rl():
    #global memory
    #global policy
    #memory = rl.memory.SequentialMemory
    #policy = rl.policy.LinearAnnealedPolicy(rl.policy.EpsGreedyQPolicy(), attr='eps', value_max=1., value_min=.1, value_test=.05, nb_steps=10000)
    #dqn = rl.agents.dqn.DQNAgent(model=model, nb_actions=num_actions, memory=memory, nb_steps_warmup=10, target_model_update=1e-2, policy=policy)
   global failing_part
   global callbacks_since_reset
   global farest_segment
   global wait_after_reset
   global in_reset_mode
   global output_layer_weights
   global output_layer_bias
   global on_training
   on_training = False
   layers = model.layers
   output_layer_weights = layers[-1].get_weights()[0]
   output_layer_bias = layers[-1].get_weights()[1]
   in_reset_mode =0
   wait_after_reset =0
   farest_segment = 0
   layers = model.layers
   output_layer_shape = layers[-1].get_weights()[0].shape
   print ("------------shape of output layer: "+str(output_layer_shape))

   callbacks_since_reset =0
   failing_part = False

load_model()

init_reset_car()

init_rl()

rospy.init_node('drive_nn', anonymous=True)

rospy.Subscriber(TOPIC_CRASH, Empty, on_crash)

rospy.Subscriber(TOPIC_VOXEL, PointCloud2, voxel_callback, queue_size=1)
rospy.Subscriber(TOPIC_STATE_TELEMETRY, gazebo_state_telemetry, state_telemetry_callback)
drive_parameters_publisher = rospy.Publisher(TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()