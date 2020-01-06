#!/usr/bin/env python3

import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String
from time import sleep

import processor
from constants import PATH_RASPICAM, PATH_USBCAM, PATH_LIDAR, PATH_GALAPAGOS_STATE
from turtlebot import TURTLE
from scheduler import SCHEDULER  # ! not used in contest

# TURTLE.disable()
# TURTLE.set_speed("little")
TURTLE.set_speed("normal")
# TURTLE.set_speed("slower")
TURTLE.set_speed_smooth("normal")

# SCHEDULER.set_state("traffic_light")
SCHEDULER.set_state("to_intersection")
# SCHEDULER.set_state("zigzag")
# SCHEDULER.set_state("to_construction")
# SCHEDULER.set_state("construction")
# SCHEDULER.set_state("parking")


rospy.Subscriber(PATH_USBCAM, CompressedImage,
                 processor.process_subcam, queue_size=1)
rospy.Subscriber(PATH_LIDAR, LaserScan,
                 processor.process_lidar, queue_size=1)
rospy.Subscriber(PATH_RASPICAM, CompressedImage,
                 processor.process_frontcam, queue_size=1)

# i = 0
# pub = rospy.Publisher('test', String, queue_size=1)
# while not rospy.is_shutdown():
#     MSG = String()
#     MSG.data = str(i)
#     pub.publish(MSG)
#     i = i + 1
#     sleep(1)
