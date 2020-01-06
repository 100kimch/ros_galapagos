#!/usr/bin/env python3

import sys
from time import sleep
import rospy
from turtlebot import TURTLE
from std_msgs.msg import String
from constants import PATH_GALAPAGOS_STATE

# * Variables
NAME = 'lib_runner'

print("----- GALAPAGOS TURTLE -----")
# * Main Codes
if __name__ == '__main__':
    try:
        RUN_TYPE = 'run_' + sys.argv[1]

        # NOTE: publisher to stop viewers
        PUBLISHER_PACKAGE = rospy.Publisher(PATH_GALAPAGOS_STATE, String, queue_size=5)
        MSG = String()
        MSG.data = RUN_TYPE
        PUBLISHER_PACKAGE.publish(MSG)

        rospy.loginfo("executing: %s" % RUN_TYPE)
        __import__('%s' % (RUN_TYPE), fromlist=[RUN_TYPE])
        rospy.loginfo("Galapagos package started.")
        
        while not rospy.is_shutdown():
            TURTLE.move()
            rospy.Rate(5).sleep()
        
        #rospy.spin() # ! Deprecated
    # NOTE: The codes below will never be worked
    # except rospy.ROSInterruptException:
    #     rospy.loginfo('exception occurred')
    #     rospy.signal_shutdown("bye")
    #     pass
    except KeyboardInterrupt:
        print("\r", end='')

    TURTLE.disable()
    MSG.data = "view"
    print(str(PUBLISHER_PACKAGE.publish(MSG)))
    
    # TODO: change time function to "await"
    for i in [1]:
        print("Turning off in %d s...\r" % i, end='')
        sleep(1)
    print("See you.                        ")
