#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, LaserScan
import processor
from turtlebot import TURTLE
from constants import PATH_USBCAM, PATH_RASPICAM, PATH_LIDAR, IS_DEBUG_MODE

# TODO: Fix this: (variable of constants.py will never be changed)
IS_DEBUG_MODE = True

# TURTLE.disable()

# rospy.Subscriber(PATH_USBCAM, CompressedImage, processor.process_eye, queue_size=1)
rospy.Subscriber(PATH_USBCAM, CompressedImage, processor.process_subcam, queue_size=1)
# rospy.Subscriber(PATH_RASPICAM, CompressedImage, processor.process_fishcam, queue_size=1)
# rospy.Subscriber(PATH_LIDAR, LaserScan, processor.process_blocking, queue_size=1)
