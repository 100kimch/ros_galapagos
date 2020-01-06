#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan
import processor
from constants import PATH_RASPICAM, PATH_USBCAM, PATH_LIDAR, PATH_GALAPAGOS_STATE, IS_DEBUG_MODE

# TODO: Fix this: (variable of constants.py will never be changed)
IS_DEBUG_MODE = False

rospy.Subscriber(PATH_GALAPAGOS_STATE, String, processor.set_package_state, queue_size=1)
rospy.Subscriber(PATH_RASPICAM, CompressedImage, processor.view_fishcam, queue_size=1)
# rospy.Subscriber(PATH_USBCAM, CompressedImage, processor.view_frontcam, queue_size=1)
rospy.Subscriber(PATH_USBCAM, CompressedImage, processor.view_subcam, queue_size=1)
# rospy.Subscriber(PATH_LIDAR, LaserScan, processor.process_lidar, queue_size=1)

# while not rospy.is_shutdown():
#     TURTLE.move()
#     rospy.Rate(60).sleep()