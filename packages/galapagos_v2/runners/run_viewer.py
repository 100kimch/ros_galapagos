#!/usr/bin/env python3

import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
import viewer
import processor
from turtlebot import TURTLE
from scheduler import SCHEDULER  # ! not used in contest

from constants import PATH_RASPICAM, PATH_USBCAM, PATH_LIDAR, PATH_GALAPAGOS_STATE

TURTLE.disable()
SCHEDULER.set_state("to_construction")
# SCHEDULER.set_state("parking")
# SCHEDULER.set_state("to_intersection")
# SCHEDULER.set_state("traffic_light")

rospy.Subscriber(PATH_USBCAM, CompressedImage,
                 processor.process_subcam, queue_size=1)
rospy.Subscriber(PATH_RASPICAM, CompressedImage,
                 processor.process_frontcam, queue_size=1)
rospy.Subscriber(PATH_LIDAR, LaserScan,
                 processor.process_lidar, queue_size=1)
# rospy.Subscriber('/cmd_vel', Twist, viewer.view_speed, queue_size=1)
