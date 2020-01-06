#! /usr/bin/env python3
#! Deprecated

import numpy as np
import copy
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import LaserScan
NAME = 'Test Code'

######################################## functions ########################################


def get_average(_data):
    average = sum(_data)/(len(_data)-_data.count(0)+0.01)
    return average


############################ main function of detecting blocking bar ############################
def detecting_blocking_bar(_ladar_data):
    ''' detecting blocking bar function main function of this py file
        * Input
            _ladar_data : ladar data from LaserScan directory that is published at ladar module
        * Output
            bool : True is there is blocking_bar in front of our turtlebot
                   False is there is no blocking_bar
    '''
    front_distance = _ladar_data[0:360]
    rospy.loginfo(front_distance)

    print(front_distance)

    result = 1
    if result == 1:
        return True
    else:
        return False


#! code for test start
def ladar_test(_ladar_data):
    # print(_ladar_data)
    front_distance = _ladar_data.ranges[0:360]
    arr = [0, 90, 180, 270]
    for i in arr:
        print('deg %d: %f' % (i, _ladar_data.ranges[i]))
    # rospy.loginfo(front_distance)
#! code for test end


rospy.init_node('ladar_sensor', anonymous=True)


def scan_ladar():
        # rospy.Subscriber('/scan', LaserScan, detecting_blocking_bar)
    rospy.Subscriber('/scan', LaserScan, ladar_test,
                     queue_size=1)  # ! code for test
    rospy.spin()


if __name__ == '__main__':
    try:
        scan_ladar()
    except rospy.ROSInterruptException:
        pass
