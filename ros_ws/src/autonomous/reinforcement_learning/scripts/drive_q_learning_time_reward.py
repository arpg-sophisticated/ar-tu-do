#!/usr/bin/env python

from reinforcement_learning_node import ReinforcementLearningNode, device
import os
import rospy
from parameters_q_learning_time_reward import NeuralQEstimator, ACTIONS, LASER_SAMPLE_COUNT
import torch
from topics import TOPIC_DRIVE_PARAMETERS_WF
from drive_msgs.msg import drive_param


class QLearningDrivingNode(ReinforcementLearningNode):
    ''' ROS node to drive the car using previously learned
    Q-Learning weights
    '''

    def __init__(self):
        self.policy = NeuralQEstimator().to(device)

        try:
            self.policy.load()
            self.policy.eval()
        except IOError:
            message = "Model parameters for the neural net not found. You need to train it first."
            rospy.logerr(message)
            rospy.signal_shutdown(message)
            exit(1)

        ReinforcementLearningNode.__init__(self, ACTIONS, LASER_SAMPLE_COUNT)

        rospy.Subscriber(TOPIC_DRIVE_PARAMETERS_WF, drive_param, self.wallfollowing_drive_param_callback)

    def on_receive_laser_scan(self, message):
        self.lastLasermessage = message
        

    def wallfollowing_drive_param_callback(self,message):
        if self.policy is None:
            return
        self.lastWFmessage = message
        if(self.lastLasermessage is None):
            return

        state = self.convert_laser_message_to_tensor(self.lastLasermessage)

        with torch.no_grad():
            action = self.policy(state).max(0)[1].item()
        self.perform_action(action,True)

    # override implemented function of ReinforcementLearningNode
    def perform_action(self, action_index, addWFMessage):
        if action_index < 0 or action_index >= len(self.actions):
            raise Exception("Invalid action: " + str(action_index))

        angle, velocity = self.actions[action_index]
        print(str(angle)+", "+ str(velocity))
        message = drive_param()
        # add WFdrivemessages
        if(addWFMessage):
            message.angle = angle +self.lastWFmessage.angle
            message.velocity = velocity+ self.lastWFmessage.velocity
        else:
            message.angle = angle
            message.velocity = velocity
        self.drive_parameters_publisher.publish(message)

rospy.init_node('q_learning_driving', anonymous=True)
node = QLearningDrivingNode()
rospy.spin()
