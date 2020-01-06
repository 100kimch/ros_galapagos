#!/usr/bin/env python3

# NOTE: using rospy library unrecommended in processor.py
import rospy
from geometry_msgs.msg import Twist
from lib_eye import EYE
from lib_lidar import *
from scheduler import SCHEDULER
import math

# * Variables

IS_RUNNING = True
BUF_ANGULAR = [0, 0]


# * Methods


def initialize():
    """ initialize processing """
    EYE.calibrate()


def view_subcam(image):
    """ process the subcam image """
    global DIRECTION
    global BUF_ANGULAR

    if not SCHEDULER.is_enable["subcam"]:
        return
    if SCHEDULER.is_subcam_occupied():
        return

    if SCHEDULER.debug_option["show_timer"]:
        SCHEDULER.check_time("subcam", min=0.3)

    info = EYE.see_sub(image)
    # print(EYE.get_front_state() + "  " + str(EYE.get_front_state()))

    if info is None:
        rospy.logwarn("[PROC] No Information!")
        return
    elif False:
        # elif EYE.get_front_state() is "turning":
        # rospy.logdebug("[PROC] turning...")
        center = info["center"]
        slope = info["slope"]

        # left: + right: -
        # if slope > 0:
        #     weight_slope = pow(abs(slope) / 1.8, 0.9) * 2.6
        # else:
        #     weight_slope = - pow(abs(slope) / 1.8, 0.9) * 2.6
        #     # weight_slope = - pow(abs(slope) / 1.8, 0.5) * 2.5

        if slope > 0:
            value = pow(abs(slope) / 1.8, 1.2) * 3.2
        else:
            value = - pow(abs(slope) / 1.8, 1.2) * 3.2

        # if slope > 0:
        #     weight_center = pow(abs(center) / 250, 0.9) * 5.5
        # elif slope < -0:
        #     weight_center = - pow(abs(center) / 250, 0.9) * 5.5
        # else:
        #     weight_center = 0
        #     # weight_center = slope * 1

        if value > 2.6:
            value = 2.6
        elif value < -2.6:
            value = -2.6

        degree = value
        # BUF_ANGULAR.append(value)

        # past_val = BUF_ANGULAR.pop(0) * 0.7
        # if info["has_line"]:
        #     if (value * past_val >= 0) and (abs(value) > abs(past_val)):
        #         degree = value
        #     else:
        #         degree = 0
        #         BUF_ANGULAR = [0] * BUF_SIZE
        # else:
        #     if value > 0:
        #         degree = 2.7
        #     elif value < 0:
        #         degree = -2.7
        #     else:
        #         degree = 0

        # if not info["has_line"]:
        #     if (slope < 0):
        #         degree = 4.12
        #     else:
        #         degree = -4.12

        if SCHEDULER.debug_option["show_center_slope"]:
            rospy.logdebug(
                "[PROC] slope: {:.2f}  w_slope: {:.2f}  degree: {:.2f}  {}".format(
                    slope, value, degree, info["has_line"])
            )
    elif EYE.get_front_state() is "straight":
        # rospy.logdebug("[PROC] going straight...")
        center = info["center"]
        if center < 0:
            value = pow(abs(center) / 150, 0.9) * 2
        elif center > 0:
            value = - pow(abs(center) / 150, 0.9) * 2
        else:
            value = 0

        if value > 1.5:
            value = 1.5
        elif value < -1.5:
            value = -1.5

        degree = value
        if SCHEDULER.debug_option["show_center_slope"]:
            rospy.logdebug(
                "[PROC] center: {:.2f}  w_center: {:.2f}  {}".format(
                    center, degree, info["has_line"])
            )

    rospy.Timer(
        rospy.Duration(0.14), SCHEDULER.release_subcam_occupied, oneshot=True
    )
    # TURTLE.turn("", 0.13, degree)
    SCHEDULER.release_subcam_occupied()


def view_frontcam(image):
    """ process the frontcam """
    # rospy.loginfo("[VIEW] frontcam image received.")
    if not SCHEDULER.is_frontcam_occupied():
        if SCHEDULER.debug_option["show_center_slope"]:
            SCHEDULER.check_time("frontcam", min=0.4)
        info = EYE.see_front(image)
        if info is None:
            return
        # rospy.logdebug("info: {:s}".format(str(info)))
        if info["center"] is "-1000" or info["center"] is "1000" or info["center"] is "0":
            pass
        else:
            EYE.reset_state()
        rospy.Timer(rospy.Duration(0.1),
                    SCHEDULER.release_frontcam_occupied, oneshot=True)


def view_lidar(lidar_data):
    """ process the lidar data """
    set_lidar_values(lidar_data)
    rospy.logdebug("left: {:.2f}  right: {:.2f}  front: {:.2f}".format(
        get_object_distance("left"), get_object_distance("right"), get_object_distance("front")))


def view_speed(twist):
    global IS_RUNNING
    if twist.linear.x == 0 and twist.angular.x == 0:
        IS_RUNNING = False
    else:
        # rospy.logdebug("[VIEW] speed: {:.2f}".format(twist.linear.x))
        IS_RUNNING = True


def calibrate_subcam(image):
    """ calibrate the subcam """
    info = EYE.see_sub(image)
    rospy.loginfo("info: {:s}".format(str(info)))
    rospy.signal_shutdown("[VIEW] ended calibration")
