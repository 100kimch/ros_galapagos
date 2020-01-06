#!/usr/bin/env python3

import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan
import viewer
from constants import PATH_RASPICAM, PATH_USBCAM, PATH_LIDAR, PATH_GALAPAGOS_STATE

rospy.Subscriber(PATH_USBCAM, CompressedImage,
                 viewer.calibrate_subcam, queue_size=1)
# rospy.Subscriber(PATH_RASPICAM, CompressedImage,
#  viewer.view_frontcam, queue_size=1)
