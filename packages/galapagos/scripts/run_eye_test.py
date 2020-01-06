#!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import CompressedImage
# import processor
# from turtlebot import TURTLE
# from constants import PATH_USBCAM, IS_DEBUG_MODE
from lib_eye import EYE

# TODO: Fix this: (variable of constants.py will never be changed)
IS_DEBUG_MODE = True

i = 0
while i <= 50:
    image_path = "../../../assets/images_usb/image" + "{:02d}".format(i) + ".png"
    key = EYE.see(image_path=image_path)
    i = i + key
    if i < 0 or i > 50:
        i = 0
# rospy.Subscriber(PATH_USBCAM, CompressedImage, processor.process_eye, queue_size=1)

# while not rospy.is_shutdown():
# TURTLE.move()
# rospy.Rate(120).sleep()
