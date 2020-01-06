#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import processor
from constants import PATH_USBCAM

IS_DEBUG_MODE = True
CURRENT_STATE = 'traffic_light'

rospy.Subscriber(PATH_USBCAM, CompressedImage, processor.process_front_image, queue_size=1)
