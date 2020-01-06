#!/usr/bin/env python3
#! Deprecated

import rospy
from sensor_msgs.msg import CompressedImage
import processor
from constants import PATH_RASPICAM, PATH_USBCAM

rospy.Subscriber('/usbcam/image_raw/compressed',
                 CompressedImage, processor.process_frontcam, queue_size=1)
