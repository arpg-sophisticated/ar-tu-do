#!/usr/bin/env python
# coding: utf-8

from topics import TOPIC_DRIVE_PARAMETERS_WF
from training_node import TrainingNode, device
import random
import math
from collections import deque
from parameters_q_learning_time_reward import *
import rospy
from drive_msgs.msg import drive_param

from playsound import playsound
import torch

import simulation_tools.reset_car as reset_car
from simulation_tools.track import track
BATCH_INDICES = torch.arange(0, BATCH_SIZE, device=device, dtype=torch.long)


class QLearningTimeRewardTrainingNode(TrainingNode):
    ''' ROS node to train the Q-Learning model with a reward based on time spent to finish an episode
    '''

    def __init__(self):
        TrainingNode.__init__(
            self,
            NeuralQEstimator().to(device),
            ACTIONS,
            LASER_SAMPLE_COUNT,
            MAX_EPISODE_LENGTH,
            LEARNING_RATE)

        self.memory = deque(maxlen=MEMORY_SIZE)
        self.optimization_step_count = 0

        self.episode_memory = deque(maxlen=EPISODE_MEMORY_SIZE)
        self.episode_starttime= rospy.get_time()

        self.lastWFmessage = None
        self.target_segment = None

        self.sleep_after_crash =0
        self.sleep_after_reset =0
        self.sleep_after_reached_target=0
        
        self.reached_target = False

        self.reached_target_time = None

        self.car_crashed =False

        self.sector_times_set = False

        self.sector_times = [None] * len(START_POINTS)

        self.reset_car_to_segment(0)

        rospy.Subscriber(TOPIC_DRIVE_PARAMETERS_WF, drive_param, self.wallfollowing_drive_param_callback)

        if CONTINUE:
            self.policy.load()

    def wallfollowing_drive_param_callback(self,message):
        if(self.lastLasermessage is None):
            return
        self.lastWFmessage = message
        if(not self.check_sector_times_set()):
            if(self.sleeping()):
                self.perform_action(NULL_ACTION_INDEX,False)
                return
            self.set_sector_times()
            self.drive_parameters_publisher.publish(message) # do wallfollowing to get sector_times
            return

        if(self.sleeping()):
            self.perform_action(NULL_ACTION_INDEX,False)
            return
        if(self.lastWFmessage == None):
            return

        new_state = self.convert_laser_message_to_tensor(self.lastLasermessage)

        if self.is_terminal_step:
            print("-- TERMINAL STEP --")
            self.assign_rewards_and_to_memory()
            self.replay()
            self.episode_memory.clear()
            self.reset_car_to_segment(random.randint(0,4))
            return
        
        if self.state is not None:
            self.on_complete_step(self.state, self.action, new_state)
        if self.reached_target_segment():
            self.reached_target_time = rospy.get_time()
            playsound('/home/marvin/Downloads/woosh_trimmed.mp3')  
            print("-- REACHED TARGET --")    
            self.perform_action(NULL_ACTION_INDEX,False)
            self.sleep_after_reached_target=100
            self.reached_target = True
            self.is_terminal_step = True   
        else:
            self.state = new_state
            self.action = self.select_action(new_state)
            self.perform_action(self.action,True)
            self.episode_length += 1
            self.total_step_count += 1

    def check_sector_times_set(self):
        if self.sector_times_set:
            return True
        for sector_time in self.sector_times:
            if sector_time is None or sector_time ==0:
                #print(sector_time)
                return False
        #print("sector times set!")
        self.sector_times_set = True
        print(str(self.sector_times))
        return True

    def set_sector_times(self):
        still_driving = False
        for i in range(len(self.sector_times)):
            if self.sector_times[i] ==0:
                still_driving = True
        if self.reached_target_segment() and still_driving:
            print("Episode Starttime: "+str(self.episode_starttime))
            print("Episode Endtime: "+str(rospy.get_time()))

            episode_time = rospy.get_time()-self.episode_starttime
            print("episode time: " +str(episode_time))
            self.sector_times[self.current_segment_index]=episode_time
            self.sleep_after_reached_target =100
            return
        for i in range(len(self.sector_times)):
            if self.sector_times[i] ==0:
                return
            if self.sector_times[i] is None:
                self.reset_car_to_segment(i)
                self.sector_times[i]=0
                return  
        

    def sleeping(self):
        sleep = self.sleep_after_reached_target+self.sleep_after_crash+self.sleep_after_reset
        sleeping=sleep>0
        if(sleeping):
            if(self.sleep_after_reset==1):
                self.episode_starttime = rospy.get_time()
            if(self.sleep_after_crash>0):
                self.sleep_after_crash -=1
            if(self.sleep_after_reached_target>0):
                self.sleep_after_reached_target -=1
            if(self.sleep_after_reset>0):
                self.sleep_after_reset -=1
        return sleeping

    # override implemented function of traning_node
    def on_receive_laser_scan(self, message):
        self.lastLasermessage=message

    def reached_target_segment(self):
        #print("Position: "+str(self.car_position))
        #print("------ "+str(track.get_closest_segment(self.car_position))+" ------")
        if(self.target_segment is not None):
            tmp_target_segment = self.target_segment #target_segment at the beginning of the round could be smaller then current segment
            current_segment = track.get_closest_segment(self.car_position)
            if(self.target_segment<10 and current_segment>=10):
                tmp_target_segment=tmp_target_segment+track.size
            return current_segment >=tmp_target_segment
        return False

    def assign_rewards_and_to_memory(self):
        reward = self.get_reward()
        for (state, action, next_state, is_terminal_step) in self.episode_memory:
            self.memory.append((state,action,reward,next_state,is_terminal_step))
            self.cumulative_reward += reward
        self.episode_memory.clear()

    def reset_car_to_segment(self,segment_index):
        print("--RESET--")
        self.sleep_after_reset = 100
        self.drive_forward = random.random() > 0.5 #TODO: bisher nicht benutzt
        start_segment =  START_POINTS[segment_index]
        self.current_segment_index =segment_index
        self.target_segment = (start_segment+10)%(track.size)
        self.car_position=reset_car.reset_to_segment(start_segment)
        print("---- start_segment = "+str(start_segment)+ ",target_segment = "+str(self.target_segment)+" ----")
        self.is_terminal_step = False
        self.state = None
        if self.episode_length != 0:
            print("Episode Length: "+ str(self.episode_length))
            self.on_complete_episode()
        self.episode_starttime= rospy.get_time()
        self.reached_target = False

    def replay(self): #TODO: this was done every step, now we are doing it after an episode, raise stepsize of update?
        print("---- replay (memory size: "+ str(len(self.memory))+")----")
        if len(self.memory) < 500 or len(self.memory) < BATCH_SIZE:
            return

        if self.optimization_step_count == 0:
            rospy.loginfo("Model optimization started.")

        transitions = random.sample(self.memory, BATCH_SIZE)  # nopep8
        states, actions, rewards, next_states, is_terminal = tuple(zip(*transitions))  # nopep8

        states = torch.stack(states)
        actions = torch.tensor(actions, device=device, dtype=torch.long)
        rewards = torch.tensor(rewards, device=device, dtype=torch.float)
        next_states = torch.stack(next_states)
        is_terminal = torch.tensor(
            is_terminal, device=device, dtype=torch.uint8)

        next_state_values = self.policy.forward(next_states).max(1)[0].detach()
        q_updates = rewards + next_state_values * DISCOUNT_FACTOR
        q_updates[is_terminal] = rewards[is_terminal]

        self.optimizer.zero_grad()
        net_output = self.policy.forward(states)
        loss = F.smooth_l1_loss(net_output[BATCH_INDICES, actions], q_updates)
        loss.backward()
        for param in self.policy.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()
        self.optimization_step_count += 1

    def get_epsilon_greedy_threshold(self):
        return EPS_END + (EPS_START - EPS_END) * \
            math.exp(-1. * self.total_step_count / EPS_DECAY)

    def select_action(self, state):
        use_epsilon_greedy = self.episode_count % 2 == 0
        if use_epsilon_greedy and random.random() < self.get_epsilon_greedy_threshold():
            return random.randrange(ACTION_COUNT)

        with torch.no_grad():
            output = self.policy(state)
            if self.episode_length < 10:
                self.net_output_debug_string = ", ".join(
                    ["{0:.1f}".format(v).rjust(5) for v in output.tolist()])
            return output.max(0)[1].item()

    # override implemented function of ReinforcementLearningNode
    def perform_action(self, action_index, addWFMessage):
        if action_index < 0 or action_index >= len(self.actions):
            raise Exception("Invalid action: " + str(action_index))

        angle, velocity = self.actions[action_index]
        message = drive_param()
        # add WFdrivemessages
        if(addWFMessage):
            message.angle = angle +self.lastWFmessage.angle
            message.velocity = velocity+ self.lastWFmessage.velocity
        else:
            message.angle = angle
            message.velocity = velocity
        self.drive_parameters_publisher.publish(message)

    def get_reward(self):
        
        if(self.reached_target):
            wallfollowing_episode_time = self.sector_times[self.current_segment_index]
            print("wallfollowing episode time: " + str(wallfollowing_episode_time))
            episode_time = self.reached_target_time-self.episode_starttime
            print("episode time: " +str(episode_time))

            time_difference = wallfollowing_episode_time - episode_time



            reward =  0.2 +((1+time_difference)**2)-1

            print("reward "+str(reward))
            return reward
        else:
            print("reward -2")
            return -2

    def get_episode_summary(self):
        return TrainingNode.get_episode_summary(self) + ' ' \
            + ("memory: {0:d} / {1:d}, ".format(len(self.memory), MEMORY_SIZE) if len(self.memory) < MEMORY_SIZE else "") \
            + "ε-greedy: " + str(int(self.get_epsilon_greedy_threshold() * 100)) + "% random, " \
            + "replays: " + str(self.optimization_step_count) + ", " \
            + "q: [" + self.net_output_debug_string + "], "

    def on_complete_step(self, state, action, next_state):
        self.episode_memory.append((state, action, next_state, self.is_terminal_step))  # nopep8

    def on_crash(self,_):
        if(not self.is_terminal_step):
            print("--CRASH--")
            self.perform_action(NULL_ACTION_INDEX,False)
            self.is_terminal_step = True
            self.sleep_after_crash =100


rospy.init_node('q_learning_training', anonymous=True)
node = QLearningTimeRewardTrainingNode()
rospy.spin()
