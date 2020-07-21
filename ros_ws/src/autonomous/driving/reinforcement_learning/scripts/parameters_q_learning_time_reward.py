import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable

import os
import rospy

from rospkg import RosPack

# General parameters
# ACTIONS = [(0.5, 1.0), (0.5, 0.0), (0, 0.0),
#           (0, 1.0), (-0.5, 0.0), (-0.5, 1.0)]
ACTIONS = [(0.15, 1.0), (0.15, 0.0), (0, 0.0),
           (0, 1.0), (-0.15, 0.0), (-0.15, 1.0)]

ACTION_COUNT = len(ACTIONS)
NULL_ACTION_INDEX = 2

# (startindex,endindex,forward,rounds,timedifference-multiplikator,greedy-factor)

#TRAINING_PARTS = [(0, 10, False, 0, 1, 1), (10, 20, False, 0, 1, 1), (20, 30, False, 0, 1, 1), (30, 40, False, 0, 1, 1), (40, 0, False, 1, 1, 1),  # nopep8
#                  (0, 10, True, 0, 1, 1), (10, 20, True, 0, 1, 1), (20, 30, True, 0, 1, 1), (30, 40, True, 0, 1, 1), (40, 0, True, 1, 1, 1)]  # nopep8
#TRAINING_PARTS = [(0, 10, False, 0, 1, 1), (10, 20, False, 0, 1, 1), (20, 30, False, 0, 1, 1), (30, 40, False, 0, 1, 1), (40, 0, False, 1, 1, 1), (40, 0, False, 2, 0.2, 0.5),  # nopep8
#                   (0, 10, True, 0, 1, 1), (10, 20, True, 0, 1, 1), (20, 30, True, 0, 1, 1), (30, 40, True, 0, 1, 1), (40, 0, True, 1, 1, 1), (1, 11, True, 1, 0.2, 0.5)]  # nopep8
TRAINING_PARTS = [(0, 10, False, 0, 1, 1), (10, 20, False, 0, 1, 1), (20, 30, False, 0, 1, 1), (30, 40, False, 0, 1, 1), (40, 0, False, 1, 1, 1), (40, 0, False, 2, 0.2, 0.5), (40, 0, False, 2, 0.12, 0.4),  # nopep8
                  (0, 10, True, 0, 1, 1), (10, 20, True, 0, 1, 1), (20, 30, True, 0, 1, 1), (30, 40, True, 0, 1, 1), (40, 0, True, 1, 1, 1), (1, 11, True, 1, 0.2, 0.5), (1, 11, True, 2, 0.12, 0.4)]  # nopep8


# Only use some of the LIDAR measurements
# When changing this value, also update laser_sample_count in
# q_learning_time_reward*.launch
LASER_SAMPLE_COUNT = 8

# safety velocity reduction for driving node
SAFETY_REDUCTION = 1

MODEL_FILENAME = os.path.join(RosPack().get_path(
    "reinforcement_learning"), "q_learning_time_reward.to")

# Training parameters

# Start by loading previously trained parameters.
# If this is False, training will start from scratch
CONTINUE = True

DISCOUNT_FACTOR = 0.99  # aka gamma

MAX_EPISODE_LENGTH = 500
# Sample neural net update batch from the replay memory.
# It contains this many steps.
MEMORY_SIZE = 20000

# We need a memory for the current episode
# to asign the reward afterwards
EPISODE_MEMORY_SIZE = 8000

MAX_EPISODE_TIME_IN_SECONDS = 10

BATCH_SIZE = 1024
LEARNING_RATE = 0.0001

# Probability to select a random episode starts at EPS_START
# and reaches EPS_END once EPS_DECAY episodes are completed.
EPS_START = 1.0
EPS_END = 0.3
EPS_DECAY = 100000


class NeuralQEstimator(nn.Module):
    def __init__(self):
        super(NeuralQEstimator, self).__init__()
        self.fc1 = nn.Linear(LASER_SAMPLE_COUNT + 1, 64)
        self.fc2 = nn.Linear(64, 32)
        self.fc3 = nn.Linear(32, ACTION_COUNT)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)

    def load(self):
        self.load_state_dict(torch.load(MODEL_FILENAME))
        rospy.loginfo("Model parameters loaded.")

    def save(self):
        torch.save(self.state_dict(), MODEL_FILENAME)
