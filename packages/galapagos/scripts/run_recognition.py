#!/usr/bin/env python3
#! Deprecated

import rospy
from sensor_msgs.msg import CompressedImage
from constants import *
from lib_signal_recognition import *
from processor import *

def recognition(image):
    cv2.imshow("current",image)
    if is_intersection(image):
        print("construction detected!!")
    else:
        print("nothing detected!!")

rospy.Subscriber(PATH_USBCAM, CompressedImage, recognition, queue_size=1)
