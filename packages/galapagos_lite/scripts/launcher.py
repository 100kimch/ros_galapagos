#!/usr/bin/env python3
""" a module to launch modules """

import rospy
import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    sys.path.append('/opt/ros/kinetic/lib/python3.5/dist-packages')

print()
print("========= GALAPAGOS TURTLE =========")
print("  Ver 2.0.0")
print("  Konkuk University")
print("  Electronics Engineering")
print("  CLUB Boot4Dim x XDeca")
print("  Author: Kim Ji Hyeong")
print("          (100kimch@naver.com)")
print("          Song Min Soo")
print("          (tjqansthd@konkuk.ac.kr)")
print("          Ji Kunyoung")
print("          (93jkm@naver.com)")
print("====================================")
print()

from scheduler import SCHEDULER

# * init codes
SCHEDULER.import_modules()

if sys.argv[2] == "True":
    rospy.init_node('GA_' + sys.argv[1],
                    anonymous=False, log_level=rospy.DEBUG)
    SCHEDULER.debug_mode = True
    print()
    print("====================================")
    print("  DEBUG MODE Enabled.")
else:
    rospy.init_node('GA_' + sys.argv[1], anonymous=False)
    print()
    print("====================================")
    print("  Node initialized")

if sys.argv[3] == "True":
    SCHEDULER.debug_option["show_timer"] = True
    print("  TIMER Enabled.")
# else:
    # SCHEDULER.set_timer(False)

print("====================================")
print()

from turtlebot import TURTLE
from constants import PATH_GALAPAGOS_STATE
from time import sleep  # * Variables

RUN_TYPE = 'run_' + sys.argv[1]

# * Main Codes
if __name__ == '__main__':
    try:
        rospy.loginfo("[LAUNCH] executing: %s" % RUN_TYPE)
        SCHEDULER.load_module(RUN_TYPE)
        rospy.loginfo("[LAUNCH] Galapagos package started.")

        if sys.argv[1] == "viewer":
            # while not rospy.is_shutdown():
            #     pass
            # TURTLE.disable()
            rospy.spin()
        else:
            while not rospy.is_shutdown():
                TURTLE.move()
                rospy.Rate(30).sleep()

    except KeyboardInterrupt:
        print("\r", end='')

    TURTLE.disable()

    for i in [1]:
        print("Turning off in %d s...\r" % i, end='')
        sleep(1)

    rospy.loginfo("[LAUNCH] Package ended.")
    # print("See you.                                ")
