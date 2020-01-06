#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, LaserScan
import processor
from constants import PATH_RASPICAM, PATH_USBCAM, PATH_LIDAR, IS_DEBUG_MODE
from turtlebot import TURTLE

IS_DEBUG_MODE = False

# TURTLE.disable()

# TURTLE.enable()
# TURTLE.set_speed("normal")
# TURTLE.change_line("left", 1)
# TURTLE.go_forward(2)
rospy.Subscriber(PATH_RASPICAM, CompressedImage,
                 _processor.process_fishcam, queue_size=1)
rospy.Subscriber(PATH_USBCAM, CompressedImage,
                 _processor.process_frontcam, queue_size=1)
# rospy.Subscriber(PATH_LIDAR, LaserScan, _processor.process_lidar, queue_size=1)
