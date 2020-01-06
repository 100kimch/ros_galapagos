#!/usr/bin/env python3

import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan
import processor
from constants import PATH_RASPICAM, PATH_USBCAM, PATH_LIDAR, PATH_GALAPAGOS_STATE
from turtlebot import TURTLE
from scheduler import SCHEDULER  # ! not used in contest

TURTLE.set_speed("normal")
# TURTLE.disable()
# TURTLE.set_speed("fast")
# TURTLE.set_speed_smooth("normal")

# SCHEDULER.set_state("traffic_light")
SCHEDULER.set_state("default")
# SCHEDULER.set_state("to_intersection")
# SCHEDULER.set_state("construction")
# SCHEDULER.set_state("parking")

rospy.Subscriber(PATH_USBCAM, CompressedImage,
                 processor.process_subcam, queue_size=1)
# rospy.Subscriber(PATH_LIDAR, LaserScan,
#  processor.test_block_escaping, queue_size=1)
# rospy.Subscriber(PATH_LIDAR, LaserScan,
#  processor.process_lidar, queue_size=1)
# rospy.Subscriber(PATH_RASPICAM, CompressedImage,
#                  processor.process_frontcam, queue_size=1)
