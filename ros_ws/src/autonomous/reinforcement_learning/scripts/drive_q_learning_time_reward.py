#!/usr/bin/env python

from reinforcement_learning_node import ReinforcementLearningNode, device
import os
import rospy
from parameters_q_learning_time_reward import *
import torch
from topics import TOPIC_DRIVE_PARAMETERS_WF2RL, TOPIC_GAZEBO_STATE_TELEMETRY
from drive_msgs.msg import drive_param
from drive_msgs.msg import gazebo_state_telemetry


class QLearningDrivingNode(ReinforcementLearningNode):
    ''' ROS node to drive the car using previously learned
    Q-Learning weights
    '''

    def __init__(self):
        self.policy = NeuralQEstimator().to(device)
        self.current_speed = 0
        self.last_laser_scan_message = None

        try:
            self.policy.load()
            self.policy.eval()
        except IOError:
            message = "Model parameters for the neural net not found. You need to train it first."
            rospy.logerr(message)
            rospy.signal_shutdown(message)
            exit(1)

        ReinforcementLearningNode.__init__(
            self, ACTIONS, LASER_SAMPLE_COUNT)

        rospy.Subscriber(
            TOPIC_DRIVE_PARAMETERS_WF2RL,
            drive_param,
            self.wallfollowing_drive_param_callback)
        rospy.Subscriber(
            TOPIC_GAZEBO_STATE_TELEMETRY,
            gazebo_state_telemetry,
            self.speed_callback)

     # override implemented function of
     # reinforcement_learning_node
    def on_receive_laser_scan(self, message):
        self.last_laser_scan_message = message
        return

    def wallfollowing_drive_param_callback(self, message):
        if self.policy is None:
            return
        self.lastWFmessage = message
        if(self.current_speed == 0):
            print("speed 0!!!")

        state = self.convert_laser_and_speed_message_to_tensor(
            self.last_laser_scan_message, self.current_speed)

        with torch.no_grad():
            action = self.policy(state).max(0)[1].item()
        self.perform_action(action, True)

    def speed_callback(self, speed_message):
        self.current_speed = speed_message.wheel_speed
        #print("Speed: "+ str(self.current_speed))

    # override implemented function of
    # ReinforcementLearningNode
    def perform_action(self, action_index, addWFMessage):
        if action_index < 0 or action_index >= len(
                self.actions):
            raise Exception(
                "Invalid action: " +
                str(action_index))

        angle, velocity = self.actions[action_index]
        print(str(angle) + ", " + str(velocity))
        message = drive_param()
        # add WFdrivemessages
        if(addWFMessage):
            message.angle = angle + self.lastWFmessage.angle
            message.velocity = (
                velocity * SAFETY_REDUCTION) + self.lastWFmessage.velocity
        else:
            message.angle = angle
            message.velocity = velocity
        self.drive_parameters_publisher.publish(message)


rospy.init_node('q_learning_driving', anonymous=True)
node = QLearningDrivingNode()
rospy.spin()
