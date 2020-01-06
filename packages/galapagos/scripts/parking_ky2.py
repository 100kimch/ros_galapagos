#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Int8MultiArray
import processor
import lib_line_tracing
from constants import *
from turtlebot import *

IS_DEBUG_MODE = True
SELECTED_STATE = 'parking'

# parking informations 
left_distance = Int8MultiArray()
right_distance = Int8MultiArray()

LEFT = 1
RIGHT = 2

parking_space = 0               # 0 : cannot parking yet / 1 : LEFT spot / 2 : RIGHT spot
parking_enable = False          # True : arrive at parking spot / False : not yet

def parking_move():
    global LEFT; global RIGHT; global parking_enable; global parking_space

	if parking_space == LEFT:
    	TURTLE.set_angular(-0.11)
    	TURTLE.set_speed_test(0.7)
    	TURTLE.move()
    	rospy.sleep(rospy.Duration(2))

    	TURTLE.set_angular(-0.1)
    	TURTLE.set_speed_test(0)
    	TURTLE.move()
    	rospy.sleep(rospy.Duration(1.7))

    	TURTLE.set_angular(0)
    	TURTLE.set_speed_test(0)
    	TURTLE.move()
    	rospy.sleep(wait_time)

    	TURTLE.set_angular(0.1)
    	TURTLE.set_speed_test(0)
    	TURTLE.move()
    	rospy.sleep(rospy.Duration(1.7))

    	TURTLE.set_angular(0.11)
    	TURTLE.set_speed_test(-0.7)
    	TURTLE.move()
    	rospy.sleep(rospy.Duration(2))
	
    	TURTLE.set_angular(0)
    	TURTLE.set_speed_test(0)
    	TURTLE.move()

	else:
		TURTLE.set_angular(-0.11)
    	TURTLE.set_speed_test(0.7)
    	TURTLE.move()
    	rospy.sleep(rospy.Duration(2))

    	TURTLE.set_angular(-0.1)
    	TURTLE.set_speed_test(0)
    	TURTLE.move()
    	rospy.sleep(rospy.Duration(1.7))

    	TURTLE.set_angular(0)
    	TURTLE.set_speed_test(0)
    	TURTLE.move()
    	rospy.sleep(wait_time)

    	TURTLE.set_angular(0.1)
    	TURTLE.set_speed_test(0)
    	TURTLE.move()
    	rospy.sleep(rospy.Duration(1.7))

    	TURTLE.set_angular(0.11)
    	TURTLE.set_speed_test(-0.7)
    	TURTLE.move()
    	rospy.sleep(rospy.Duration(2))
	
    	TURTLE.set_angular(0)
    	TURTLE.set_speed_test(0)
    	TURTLE.move()
	parking_enable = False


def parking_possibility():
    global left_distance; global right_distance; global parking_enable; global parking_space
    global LEFT; global RIGHT

	right_distance = _lidar.ranges[270:330]
	left_distance = _lidar.ranges[120:180]

    # select parking space : left or right 
    for i in left_distance:
        if i < 0.12:
            parking_enable = True
			parking_space = LEFT
	for i in right_distance:
        if i < 0.12:
            parking_enable = True
            parking_space = RIGHT
    
    print(parking_enable,' ',parking_space)

def parking(parking_start):
	global parking_enable; global parking_space; global LINE_BASE

    rospy.init_node('parking', anonymous=True)

	if parking_enable == True:
		LINE_BASE = 0
		parking_move()
		LINE_BASE = 1

	elif parking_start == 1:
		LINE_BASE = 1
        rospy.Subscriber(PATH_LIDAR, LaserScan, parking_possibility, queue_size=1)
        rospy.spin()




