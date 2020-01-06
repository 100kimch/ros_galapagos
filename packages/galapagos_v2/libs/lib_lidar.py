#! /usr/bin/env python3

import rospy
from constants import LIDAR_DIRECTIONS

# * Variables
# LIDAR_VALUES = {
#     'front': 0.00,
#     'right': 0.00,
#     'back': 0.00,
#     'left': 0.00
# }

DISTANCES = {}
# CRASHABLE = {
#     'left': False,
#     'right': False
# }
NUM_TO_GET_AVERAGE = 3


def set_lidar_values(lidar_data):
    """ set lidar value to LIDAR_VALUES """
    global DISTANCES
    global CRASHABLE
    for index, key in enumerate(LIDAR_DIRECTIONS):
        distance = 0
        length = 5
        for j in [-2, -1, 0, 1, 2]:
            value = lidar_data.ranges[(LIDAR_DIRECTIONS[key] + j) % 360]
            if value == 0:
                length -= 1
                continue
            else:
                distance += value

        if length > 0:
            distance /= length
        # print(distance)
        # DISTANCES[key] = lidar_data.ranges[LIDAR_DIRECTIONS[key]]
        DISTANCES[key] = distance

    # print(lidar_data.ranges[LIDAR_DIRECTIONS['front']:LIDAR_DIRECTIONS['left']])
    # left_pane = \
    #     min(lidar_data.ranges[LIDAR_DIRECTIONS['front']:LIDAR_DIRECTIONS['left']])
    # right_pane = \
    #     min(lidar_data.ranges[LIDAR_DIRECTIONS['right']:LIDAR_DIRECTIONS['front']])

    # if (left_pane < 0.2) and (left_pane > 0):
    #     CRASHABLE.left = True
    # if (right_pane < 0.2) and (right_pane > 0):
    #     CRASHABLE.right = True


def is_left_crashable():
    return CRASHABLE.left


def is_right_crashable():
    return CRASHABLE.right


def get_object_distance(direction):
    """ get lidar value by direction """
    return float('%.2f' % DISTANCES[direction])


def is_in_tunnel(lidar_data):
    """ check if the turtlebot is in tunnel """
    return False / True
