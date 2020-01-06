#! Deprecated

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8MultiArray
from turtlebot import *
from constants import *
from lib_lidar import *


# * Variables
parking_LEFT = 1
parking_RIGHT = 2
parking_enable = False          # True : arrive at parking spot / False : not yet
tmp = 1


# * parking function!
def make_delay():
    TURTLE.set_angular(0)
    TURTLE.set_speed_by_percentage(0)
    TURTLE.move()
    for i in range(0,10,1):
        rospy.sleep(rospy.Duration(0.1))


def parking_move(parking_direction):    
    parking_angles = [-0.4, 0, 0, 0, -1, 0]
    parking_speeds = [0.8, 1, 0, -1, 0, 1]
    parking_times = [4, 0.4, 1.1, 2.2, 1.6, 3]

    make_delay()
    if parking_direction == parking_RIGHT:
        TURTLE.set_angular(-0.4)
        TURTLE.set_speed_by_percentage(0.8)
    else:
        TURTLE.set_angular(0.4)
        TURTLE.set_speed_by_percentage(0.8)
    TURTLE.move()
    rospy.sleep(rospy.Duration(4))
    for i in range(0, 5, 1):
        make_delay()
        TURTLE.set_angular(parking_angles[i])
        TURTLE.set_speed_by_percentage(parking_speeds[i])
        TURTLE.move()
        rospy.sleep(rospy.Duration(parking_times[i]))

    make_delay()

    # go right parking spot, stop, go back, turn right, stop



# * Thread function!
def parking_control(lidar):
    global parking_enable; global parking_space; global LEFT; global RIGHT
    set_lidar_values(lidar)

    #print("right : ",right_distance)
    #print("left : ",left_distance)

    # select parking space : left or right 
    print("============================")
    for i in range(30,40,1):
        distance = lidar.ranges[i]
        if distance > 0.7:
            distance = 0
        elif distance < 0.5:
            parking_enable = True
            parking_move(parking_RIGHT)
            
            if IS_DEBUG_MODE == True:
                print("right parking start!")
                print(str(i) + " " + str(distance))


    for i in range(320,330,1):
        distance = lidar.ranges[i]
        if distance > 0.7:
            distance = 0
        elif distance < 0.5:
            parking_enable = True
            parking_move(parking_LEFT)

            if IS_DEBUG_MODE == True:
                print("left parking start!")
                print(str(i) + " " + str(distance))

    print("============================")
    