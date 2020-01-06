#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import processor
from constants import PATH_RASPICAM, IS_DEBUG_MODE

IS_DEBUG_MODE = False

rospy.Subscriber(PATH_RASPICAM, CompressedImage, processor.process_fishcam, queue_size=1)
