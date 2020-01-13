#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, LaserScan

import processor
from constants import PATH_RASPICAM, PATH_USBCAM, PATH_LIDAR
from turtlebot import TURTLE
from scheduler import SCHEDULER 

TURTLE.set_speed("normal")
TURTLE.set_speed_smooth("normal")

SCHEDULER.set_state("parking")
# SCHEDULER.set_state("construction")


rospy.Subscriber(PATH_USBCAM, CompressedImage,
                 processor.process_subcam, queue_size=1)
rospy.Subscriber(PATH_LIDAR, LaserScan,
                 processor.process_lidar, queue_size=1)
rospy.Subscriber(PATH_RASPICAM, CompressedImage,
                 processor.process_frontcam, queue_size=1)
