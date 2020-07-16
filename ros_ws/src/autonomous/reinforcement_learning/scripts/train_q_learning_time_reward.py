#!/usr/bin/env python
# coding: utf-8

import random
import math
import rospy
import torch
import simulation_tools.reset_car as reset_car
from training_node import TrainingNode, device
from collections import deque
from parameters_q_learning_time_reward import *
from drive_msgs.msg import drive_param
from drive_msgs.msg import gazebo_state_telemetry
from topics import TOPIC_GAZEBO_STATE_TELEMETRY, TOPIC_DRIVE_PARAMETERS, TOPIC_DRIVE_PARAMETERS_WF2RL
from playsound import playsound
from simulation_tools.track import track


BATCH_INDICES = torch.arange(
    0,
    BATCH_SIZE,
    device=device,
    dtype=torch.long)


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

        self.episode_memory = deque(
            maxlen=EPISODE_MEMORY_SIZE)
        self.episode_starttime = rospy.get_time()

        self.last_WF_message = None
        self.last_laser_scan_message = None
        self.current_speed = 0

        self.target_point = None

        self.sleep_after_crash = 0
        self.sleep_after_reset = 0
        self.sleep_after_reached_target = 0

        self.reached_target = False

        self.current_closest_point = None
        self.training_part_completet_rounds = 0

        self.reached_target_time = None

        self.car_crashed = False
        self.crash_count = 0

        self.training_part_times_set = False

        self.training_part_times = [
            None] * len(TRAINING_PARTS)

        self.crash_training_part_index = None

        self.current_training_part_index = None

        self.reset_car_to_start_of_training_part(0)

        rospy.Subscriber(
            TOPIC_DRIVE_PARAMETERS_WF2RL,
            drive_param,
            self.wallfollowing_drive_param_callback)
        rospy.Subscriber(
            TOPIC_GAZEBO_STATE_TELEMETRY,
            gazebo_state_telemetry,
            self.speed_callback)

        if CONTINUE:
            self.policy.load()

    # called to subscribe TOPIC_GAZEBO_STATE_TELEMETRY
    def speed_callback(self, speed_message):
        self.current_speed = speed_message.wheel_speed

    # called to subscribe TOPIC_DRIVE_PARAMETERS_WF2RL
    def wallfollowing_drive_param_callback(self, message):
        self.last_WF_message = message
        self.set_current_point()

        if(not self.check_training_part_times_set()):
            if(self.sleeping()):
                self.perform_action(
                    NULL_ACTION_INDEX, False)
                return
            self.set_sector_times()
            drive_param_message = drive_param()
            drive_param_message.angle = message.angle
            drive_param_message.velocity = message.velocity
            # do wallfollowing to get TrainingPartsTimes
            self.drive_parameters_publisher.publish(
                drive_param_message)
            return

        if(self.sleeping()):
            self.perform_action(NULL_ACTION_INDEX, False)
            return
        if(self.last_WF_message is None):
            return

        new_state = self.convert_laser_and_speed_message_to_tensor(
            self.last_laser_scan_message, self.current_speed)

        if self.is_terminal_step:
            print("-- TERMINAL STEP --")
            self.assign_rewards_and_to_memory()
            self.replay()
            self.episode_memory.clear()
            if(self.crash_training_part_index is None):
                self.reset_car_to_start_of_training_part(
                    random.randint(0, len(TRAINING_PARTS) - 1))
            else:
                self.reset_car_to_start_of_training_part(
                    self.crash_training_part_index)
            return

        if self.state is not None:
            self.on_complete_step(
                self.state, self.action, new_state)
        if self.reached_target_point():
            self.reached_target_time = rospy.get_time()
            playsound('/home/marvin/Downloads/power-up.mp3')
            print("-- REACHED TARGET --")
            self.crash_training_part_index = None
            self.crash_count = 0
            self.perform_action(NULL_ACTION_INDEX, False)
            self.sleep_after_reached_target = 150
            self.reached_target = True
            self.is_terminal_step = True
        else:
            self.state = new_state
            self.action = self.select_action(new_state)
            self.perform_action(self.action, True)
            self.episode_length += 1
            self.total_step_count += 1

    # returns True, if training_part_times are completed. Also resets car on
    # completion
    def check_training_part_times_set(self):
        if self.training_part_times_set:
            return True
        for training_part_time in self.training_part_times:
            if training_part_time is None or training_part_time == 0:
                # print(sector_time)
                return False
        #print("sector times set!")
        self.training_part_times_set = True
        print(str(self.training_part_times))
        self.sleep_after_reached_target = 150
        self.reset_car_to_start_of_training_part(
            random.randint(0, len(TRAINING_PARTS) - 1))
        return True

    # drive with WF to get a TrainingPartsTime for each sector from
    # TRAINING_PARTS
    def set_sector_times(self):
        still_driving = False
        for i in range(len(self.training_part_times)):
            if self.training_part_times[i] == 0:
                still_driving = True
        if self.reached_target_point() and still_driving:
            print("Sector Time Summary:")
            print("Startpoint: " +
                  str(TRAINING_PARTS[self.current_training_part_index][0]))
            print("Endpoint: " +
                  str(TRAINING_PARTS[self.current_training_part_index][1]))
            print("Rounds: " +
                  str(TRAINING_PARTS[self.current_training_part_index][3]))

            print("Starttime: " +
                  str(self.episode_starttime))
            print("Endtime: " + str(rospy.get_time()))

            episode_time = rospy.get_time() - self.episode_starttime
            print("Time: " + str(episode_time))
            self.training_part_times[self.current_training_part_index] = episode_time
            self.sleep_after_reached_target = 150
            return
        for i in range(len(self.training_part_times)):
            if self.training_part_times[i] == 0:
                return
            if self.training_part_times[i] is None:
                self.reset_car_to_start_of_training_part(i)
                self.training_part_times[i] = 0
                return

    # returns true, if car should not do any action
    def sleeping(self):
        sleep = self.sleep_after_reached_target + \
            self.sleep_after_crash + self.sleep_after_reset
        sleeping = sleep > 0
        if(sleeping):
            if(self.sleep_after_reset == 1):
                self.episode_starttime = rospy.get_time()
            if(self.sleep_after_crash > 0):
                self.sleep_after_crash -= 1
            if(self.sleep_after_reached_target > 0):
                self.sleep_after_reached_target -= 1
            if(self.sleep_after_reset > 0):
                self.sleep_after_reset -= 1
        return sleeping

    # override implemented function of traning_node
    def on_receive_laser_scan(self, message):
        self.last_laser_scan_message = message
        return

    def set_current_point(self):
        if(self.drive_forward):
            if((self.current_closest_point == track.get_length() - 1) and track.get_closest_segment(self.car_position) == 0):
                self.training_part_completet_rounds += 1
                print(
                    "----------- ROUND COMPLETED ----------------")
        else:
            if((self.current_closest_point == 0) and track.get_closest_segment(self.car_position) == track.get_length() - 1):
                self.training_part_completet_rounds += 1
                print(
                    "----------- ROUND COMPLETED ----------------")
        self.current_closest_point = track.get_closest_segment(
            self.car_position)

    def reached_target_point(self):
        #print("Position: "+str(self.car_position))
        #print("------ "+str(track.get_closest_segment(self.car_position))+" ------")
        if(self.target_point is not None):
            self.target_point
            rounds_to_complete = TRAINING_PARTS[self.current_training_part_index][3]
            return self.current_closest_point == self.target_point and self.training_part_completet_rounds >= rounds_to_complete
        return False

    def assign_rewards_and_to_memory(self):
        print("Episode length to memory: " +
              str(len(self.episode_memory)))
        reward = self.get_reward()
        for (
            state,
            action,
            next_state,
                is_terminal_step) in self.episode_memory:
            self.memory.append(
                (state, action, reward, next_state, is_terminal_step))
            self.cumulative_reward += reward
        self.episode_memory.clear()

    def reset_car_to_start_of_training_part(
            self, part_index):
        print("--RESET--")
        self.sleep_after_reset = 100
        self.drive_forward = TRAINING_PARTS[part_index][2]
        # self.drive_forward = random.random() > 0.5 #TODO: bisher nicht
        # benutzt

        self.current_training_part_index = part_index
        if(self.drive_forward):
            start_point = TRAINING_PARTS[part_index][0]
            self.target_point = TRAINING_PARTS[part_index][1]
        else:
            start_point = TRAINING_PARTS[part_index][1]
            self.target_point = TRAINING_PARTS[part_index][0]

        self.car_position = reset_car.reset_to_segment(
            start_point, forward=self.drive_forward)
        print("---- start_point = " +
              str(start_point) +
              ",target_point = " +
              str(self.target_point) +
              " rounds: " +
              str(TRAINING_PARTS[part_index][3]) +
              " ----")
        self.is_terminal_step = False
        self.state = None
        if self.episode_length != 0:
            print("Episode Length: " +
                  str(self.episode_length))
            self.on_complete_episode()
        self.episode_starttime = rospy.get_time()
        self.reached_target = False
        self.training_part_completet_rounds = 0

    def replay(self):  # TODO: this was done every step, now we are doing it after an episode, raise stepsize of update?
        print("---- replay (memory size: " +
              str(len(self.memory)) + ")----")
        if len(
                self.memory) < 500 or len(
                self.memory) < BATCH_SIZE:
            return

        if self.optimization_step_count == 0:
            rospy.loginfo("Model optimization started.")

        transitions = random.sample(self.memory, BATCH_SIZE)  # nopep8
        states, actions, rewards, next_states, is_terminal = tuple(zip(*transitions))  # nopep8

        states = torch.stack(states)
        actions = torch.tensor(
            actions, device=device, dtype=torch.long)
        rewards = torch.tensor(
            rewards, device=device, dtype=torch.float)
        next_states = torch.stack(next_states)
        is_terminal = torch.tensor(
            is_terminal, device=device, dtype=torch.uint8)

        next_state_values = self.policy.forward(
            next_states).max(1)[0].detach()
        q_updates = rewards + next_state_values * DISCOUNT_FACTOR
        q_updates[is_terminal] = rewards[is_terminal]

        self.optimizer.zero_grad()
        net_output = self.policy.forward(states)
        loss = F.smooth_l1_loss(
            net_output[BATCH_INDICES, actions], q_updates)
        loss.backward()
        for param in self.policy.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()
        self.optimization_step_count += 1

    def get_epsilon_greedy_threshold(self):
        return EPS_END + (EPS_START - EPS_END) * \
            math.exp(-1. * self.total_step_count / EPS_DECAY)

    def select_action(self, state):
        if(self.crash_count > 100):
            return NULL_ACTION_INDEX
        use_epsilon_greedy = self.episode_count % 2 > 0
        use_epsilon_greedy_with_part_factor = use_epsilon_greedy and random.random(
        ) <= TRAINING_PARTS[self.current_training_part_index][5]
        if use_epsilon_greedy_with_part_factor and random.random(
        ) < self.get_epsilon_greedy_threshold():
            return random.randrange(ACTION_COUNT)

        with torch.no_grad():
            output = self.policy(state)
            # if self.episode_length < 10:
            #    self.net_output_debug_string = ", ".join(
            #        ["{0:.1f}".format(v).rjust(5) for v in output.tolist()])
            return output.max(0)[1].item()

    # override implemented function of
    # ReinforcementLearningNode
    def perform_action(self, action_index, addWFMessage):
        if action_index < 0 or action_index >= len(
                self.actions):
            raise Exception(
                "Invalid action: " +
                str(action_index))

        angle, velocity = self.actions[action_index]
        message = drive_param()
        # add WFdrivemessages
        if(addWFMessage):
            message.angle = angle + self.last_WF_message.angle
            message.velocity = velocity + self.last_WF_message.velocity
        else:
            message.angle = angle
            message.velocity = velocity
        #print(str(angle)+", "+ str(velocity))
        self.drive_parameters_publisher.publish(message)

    # returns reward for current_training_part_index
    def get_reward(self):
        if(self.reached_target):
            wallfollowing_training_part_time = self.training_part_times[
                self.current_training_part_index]
            print("wallfollowing training part time: " +
                  str(wallfollowing_training_part_time))
            episode_time = self.reached_target_time - self.episode_starttime
            print(
                "training part time: " +
                str(episode_time))

            time_difference = wallfollowing_training_part_time - episode_time
            print(
                "----------------------------DIFFERENCE " +
                str(time_difference) +
                " -----------------------")

            difference_multiplikator = TRAINING_PARTS[self.current_training_part_index][4]
            reward = (
                ((1 + (time_difference * difference_multiplikator) - 0.05)**3) - 1) / 5.0

            reward = reward + 0.3  # add no-crash reward
            if(reward <= 0):
                reward = 0
            print("reward " + str(reward))
            return reward
        else:
            print("reward 0")
            return 0

    # skip returning a summary for the episode
    def get_episode_summary(self):
        return

    # add state,action, next_state to memory (no reward
    # assigned yet)
    def on_complete_step(self, state, action, next_state):
        self.episode_memory.append((state, action, next_state, self.is_terminal_step))  # nopep8

    # called to subscribe TOPIC_CRASH
    def on_crash(self, _):
        if(not self.is_terminal_step):
            if(not self.reached_target_point()):
                print("--CRASH--")
                self.crash_count += 1
                self.perform_action(
                    NULL_ACTION_INDEX, False)
                self.is_terminal_step = True
                self.sleep_after_crash = 100
                self.crash_training_part_index = self.current_training_part_index
                playsound(
                    '/home/marvin/Downloads/bowser-falls.mp3')
                episode_memory_list = list(
                    self.episode_memory)[-300:]
                self.episode_memory.clear()
                self.episode_memory.extendleft(
                    episode_memory_list)


rospy.init_node('q_learning_training', anonymous=True)
node = QLearningTimeRewardTrainingNode()
rospy.spin()
