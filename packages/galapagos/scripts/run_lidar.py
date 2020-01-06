#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, LaserScan
import processor
from turtlebot import TURTLE
from constants import PATH_RASPICAM, PATH_USBCAM, PATH_LIDAR, IS_DEBUG_MODE

# TODO: Fix this: (variable of constants.py will never be changed)
IS_DEBUG_MODE = True

# TURTLE.LINE_BASE = 2
TURTLE.enable_fish = False
# TURTLE.set_speed("fast")
# TURTLE.disable()  
# TURTLE.turn("right", 2.03, True)
# rospy.Subscriber(PATH_LIDAR, LaserScan, processor.process_lidar, queue_size=1)
rospy.Subscriber(PATH_LIDAR, LaserScan, processor.process_tunnel, queue_size=1)
rospy.Subscriber(PATH_RASPICAM, CompressedImage, processor.process_fishcam, queue_size=1)

# while not rospy.is_shutdown():
#     TURTLE.move()
#     rospy.Rate(60).sleep()
