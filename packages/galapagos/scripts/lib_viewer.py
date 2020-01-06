#!/usr/bin/env python3

import sys
from time import sleep
import rospy
from turtlebot import TURTLE


# * Variables
NAME = 'lib_viewer'

print("----- GALAPAGOS TURTLE -----")

# * Main Codes
if __name__ == '__main__':
    try:
        RUN_TYPE = 'run_viewer'
        rospy.loginfo("executing: %s" % RUN_TYPE)
        __import__('%s' % (RUN_TYPE), fromlist=[RUN_TYPE])
        rospy.loginfo("GALAPAGOS VIEWER started.")
        
        rospy.spin() # ! Deprecated
    # NOTE: The codes below will never be worked
    # except rospy.ROSInterruptException:
    #     rospy.loginfo('exception occurred')
    #     rospy.signal_shutdown("bye")
    #     pass
    except KeyboardInterrupt:
        print("\r", end='')

    TURTLE.disable()
    # TODO: change time function to "await"
    for i in [1]:
        print("Turning off in %d s...\r" % i, end='')
        sleep(1)
    print("See you.                        ")