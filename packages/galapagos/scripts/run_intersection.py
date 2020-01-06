rospy.Subscriber(PATH_LIDAR, LaserScan, processor.process_lidar, queue_size=1)
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import processor
from constants import PATH_RASPICAM, PATH_USBCAM, SELECTED_STATE, IS_DEBUG_MODE

IS_DEBUG_MODE = True
SELECTED_STATE = 'intersection'

rospy.Subscriber(PATH_RASPICAM, CompressedImage, processor.process_fishcam, queue_size=1)
rospy.Subscriber(PATH_USBCAM, CompressedImage, processor.process_frontcam, queue_size=1)
