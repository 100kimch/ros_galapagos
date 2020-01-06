#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, LaserScan
import processor
from constants import PATH_RASPICAM, PATH_USBCAM, PATH_LIDAR, SELECTED_STATE, IS_DEBUG_MODE
from turtlebot import TURTLE

IS_DEBUG_MODE = True
SELECTED_STATE = 'tunnel'

# TURTLE.disable()
TURTLE.enable_fish = False
# rospy.Subscriber(PATH_USBCAM, CompressedImage, processor.process_frontcam, queue_size=1)
rospy.Subscriber(PATH_LIDAR, LaserScan, processor.process_tunnel, queue_size=1)
