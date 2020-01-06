#!/usr/bin/env python3

# NOTE: should only use "log functions" of rospy in processor.py
import rospy
from lib_subcam import EYE
# from lib_lidar import *
from scheduler import SCHEDULER
from turtlebot import TURTLE

import timeit
import math
import requests

# * Variables
LANE_TO = "left"
STEP = 0
BUF_SIZE = 3
BUF_ANGULAR = [0] * BUF_SIZE
TEST_STATE = False
NUM_OBSTRUCTION = 0


def initialize():
    """ initialize processing """
    EYE.calibrate()
    # init_ref_images()


def reset_buffer():
    BUF_ANGULAR = [0] * BUF_SIZE


def reverse_lane():
    global LANE_TO
    if LANE_TO == "right":
        LANE_TO = "left"
    else:
        LANE_TO = "right"


def test_frontcam(image):
    """ process the frontcam image """
    # print("image is subscribed")
    if not SCHEDULER.is_frontcam_enable():
        return

    if not EYE.is_front_occupied():

        STATE = SCHEDULER.get_state()

        if STATE == "traffic_light":
            signal = is_light_green(image)
            rospy.logdebug("[PROC] is_light_green: {}"
                           .format(signal))
            if signal:
                SCHEDULER.set_state("to_intersection")
                TURTLE.enable()
                TURTLE.set_speed("fast")
                TURTLE.set_speed_smooth("normal")

        elif STATE == "to_intersection":
            signal = check_left_right_sign(image)
            rospy.logdebug("[PROC] left or right: {}"
                           .format(signal))
            if signal == "right":
                SCHEDULER.set_state("intersection_right")
            elif signal == "left":
                SCHEDULER.set_state("intersection_left")
        if SCHEDULER.debug_option["show_center_slope"]:
            SCHEDULER.check_time("frontcam", min=0.5)
        info = EYE.see_front(image)
        if info is None:
            return

        # rospy.logdebug("[PROC] test_frontcam: state {}  {}".format(
        #     info["state"], info["horizon_position"]))
        # if (info["horizon_position"] < 150):
        #     TURTLE.set_speed("fast")
        #     TURTLE.set_speed_smooth("normal")

        if (info["state"] == "turning") and (info["turning_to"] is not "None"):
            rospy.logdebug(
                "[PROC] turn off for 1.5s")
            rospy.Timer(rospy.Duration(1.5),
                        EYE.release_front_occupied, oneshot=True)
        else:
            rospy.Timer(rospy.Duration(0.1),
                        EYE.release_front_occupied, oneshot=True)


def process_frontcam(image):
    """ process the frontcam image """
    if not SCHEDULER.is_frontcam_enable():
        return

    STATE = SCHEDULER.get_state()
    info = EYE.see_front(image)

    if SCHEDULER.debug_option["show_front_info"]:
        rospy.logdebug(info)

    if STATE == "default":
        # if EYE.is_boostable(image):
        process_acceleration(info)
        # signal = is_construction(image)
        # rospy.logdebug(signal)

    if STATE == "traffic_light":
        if is_light_green(image):
            TURTLE.enable()
            SCHEDULER.set_state("to_intersection")
        return

    if STATE == "to_intersection":
        signal = check_left_right_sign(image)
        if signal == "right":
            SCHEDULER.set_state("intersection_right")
        elif signal == "left":
            SCHEDULER.set_state("intersection_left")
        return

    if STATE == "intersection_right":
        # TODO: make algorithms for right
        if EYE.is_boostable(image):
            TURTLE.boost()
            SCHEDULER.set_state("to_construction")
        return

    if STATE == "intersection_left":
        if EYE.is_boostable(image):
            TURTLE.boost()
            SCHEDULER.set_state("to_construction")
        return

    if STATE == "to_construction":
        if EYE.is_boostable(image):
            TURTLE.boost()

        if is_construction(image):
            SCHEDULER.set_state("construction_searching")

    if STATE == "construction_searching":
        pass


def process_acceleration(info):
    if not info:
        return

    horizon = info['horizon_position']
    center = abs(info['center'])

    if EYE.get_front_state() == "straight":
        if horizon < 200 and center < 50:
            TURTLE.boost()
            if SCHEDULER.debug_option["show_front_info"]:
                rospy.logwarn("[PROC] Boosting")
        elif horizon < 260 and center < 50:
            TURTLE.set_speed_smooth('little')
    if horizon > 320 or info['state'] == 'turning':
        TURTLE.set_speed('normal')
    elif horizon > 260:
        TURTLE.set_speed_smooth('normal')


def process_subcam(image):
    """ process the subcam image """

    if not SCHEDULER.is_subcam_enable():
        return
    if EYE.is_sub_occupied() or TURTLE.is_occupied():
        return

    if SCHEDULER.debug_option["show_timer"]:
        SCHEDULER.check_time("subcam", min=0.25)

    info = EYE.see_sub(image)

    if info is None:
        # rospy.logwarn("[PROC] No Information!")
        return

    center = info["center"]
    slope = info["slope"]

    if slope < -0.5:
        # limit = 1.6
        limit = 1.5  # For slow speed
        amplitude = 1.6
    # elif slope > 0.5:
    #     limit = 1.6
    #     amplitude = 1.2
    else:
        # limit = 1.2
        # amplitude = 1.2
        limit = 1.2  # For slow speed
        amplitude = 1.2  # For slow speed

    if EYE.get_front_state() == "straight":
        if (abs(center) < 30) and slope < -0.4:
            degree = pow(abs(slope) / 1.8, 1.1) * amplitude
        elif center < 0:
            degree = pow(abs(center) / 100, 2.0) * amplitude
        elif center > 0:
            degree = - pow(abs(center) / 100, 2.0) * amplitude
        else:
            degree = 0
    else:
        if (abs(center) < 30) and slope < -0.4:
            degree = pow(abs(slope) / 1.8, 0.9) * amplitude
        elif center < 0:
            degree = pow(abs(center) / 100, 1.9) * amplitude
        elif center > 0:
            degree = - pow(abs(center) / 100, 1.9) * amplitude
        else:
            degree = 0

    buf_sum = sum(BUF_ANGULAR)
    if EYE.get_front_state() == "straight":
        adjust_angular = BUF_ANGULAR.pop(0) * 0.9
        BUF_ANGULAR.append(degree)
        degree -= adjust_angular
        # if abs(buf_sum) > 1:
    else:
        reset_buffer()
        adjust_angular = 0

    degree = max(min(degree, limit), -limit)

    if not info["has_line"]:
        if center < -55:
            # degree = 1.6
            degree = 1.4  # For slow speed
        elif center > 50:
            degree = -1.2
            # degree = -1.2  # For slow speed
        elif center > 10:  # editing...
            degree = -0.9  # editing...
        else:
            degree = 1.4

    # if SCHEDULER.debug_option["show_center_slope"]:
    #     rospy.logdebug(
    #         "[PROC] center: {:.2f}  slope: {:.2f}  degree: {:.2f}  adj: {:.2f}  buf_sum: {:.2f}  {}  {}".format(
    #             center, slope, degree, adjust_angular, buf_sum, EYE.get_front_state(), info["has_line"])
    #     )

    rospy.Timer(
        rospy.Duration(0.15), EYE.release_sub_occupied, oneshot=True
    )
    TURTLE.turn(0.13, degree)


def process_lidar(lidar_data):
    """ process the lidar image """
    if not SCHEDULER.is_lidar_enable():
        return

    set_lidar_values(lidar_data)

    state = SCHEDULER.get_state()
    if state is "construction":
        process_construction()
    elif state is "parking":
        process_parking()
    elif state is "tunnel":
        process_tunnel()


def process_tunnel():
    """ process tunnel state """

    if TURTLE.is_occupied():
        return

    if STEP == 30:
        # TODO: turn to 45 degrees
        pass
    elif STEP == 31:
        TURTLE.set_speed("normal")
        # TURTLE.go_turn("right", 1, angular=2, speed=0.15)
        front = get_object_distance("front")
        left = get_object_distance("left")
        right = get_object_distance("right")
        rospy.logdebug(
            "front: {}  left: {}  right: {}".format(front, left, right))
        if left < 0.20:
            # TODO: turn to variable degrees
            TURTLE.turn(0.15, -6)
            STEP = 34
            return
        elif right < 0.20:
            # TODO: turn to variable degrees
            TURTLE.turn(-0.15, -6)
            STEP = 35
            return
        elif front == 0 or front > 0.40:
            return
        else:
            TURTLE.turn(0.15, -6)
            TURTLE.turn(0.15, -6)
    elif STEP == 32:
        TURTLE.set_speed("normal")
        frontleft = get_object_distance("frontleft")
        rospy.logdebug("frontleft: {}".format(frontleft))
        if frontleft == 0 or frontleft < 0.40:
            return
        else:
            rospy.sleep(rospy.Duration(1.8))
            TURTLE.turn(0.13, 6)
    elif STEP == 33:
        TURTLE.set_speed("normal")
        frontleft = get_object_distance("frontleft")
        rospy.logdebug("frontleft: {}".format(frontleft))
        if frontleft == 0 or frontleft < 0.40:
            return
        else:
            STEP = 31
            return
    elif STEP == 34:
        front = get_object_distance("front")
        if front == 0 or front > 0.20:
            return
        # TODO : turn to left
        TURTLE.turn(-0.3, -6)
    elif STEP == 35:
        # NOTE: ending step
        TURTLE.set_speed("normal")
    else:
        return
    STEP += 1


def process_parking():
    """ process parking state """
    global STEP

    # if TURTLE.is_occupied():
    #     return

    if STEP == 10:
        frontleft = get_object_distance("frontleft")
        frontright = get_object_distance("frontright")

        if SCHEDULER.debug_option["show_parking_lidar"]:
            rospy.logdebug("front: {:.2f}  frontleft: {:.2f}".format(
                get_object_distance("front"), get_object_distance("frontleft")
            ))

        if (frontleft > 0) and (frontleft < 0.5):
            STEP = 11
            rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        elif (frontright > 0) and (frontright < 0.5):
            STEP = 12
            rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        # NOTE: return is needed to prevent executing STEP += 1
        return

    elif STEP == 11:
        SCHEDULER.disable_cams()
        TURTLE.set_speed("normal")
        TURTLE.go_turn("right", 2)
        STEP = 13
        rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        return

    elif STEP == 12:
        SCHEDULER.disable_cams()
        TURTLE.set_speed("normal")
        TURTLE.go_turn("left", 2)
        STEP = 13
        rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        return

    elif STEP == 13:
        TURTLE.go_forward(0.7)
        # for stopping for a while
        rospy.sleep(rospy.Duration(0.5))

    elif STEP == 14:
        TURTLE.go_backward(1.0)
    elif STEP == 15:
        TURTLE.go_turn_backward(1.1)
    elif STEP == 16:
        TURTLE.set_speed("normal")
        TURTLE.go_forward(1)
    elif STEP == 17:
        # TURTLE.set_speed("fast")
        # TURTLE.set_speed_smooth("normal")
        TURTLE.set_speed("normal")
        SCHEDULER.set_state("zigzag")
    else:
        return
    STEP += 1


def process_construction():
    """ process construction state """
    global STEP
    global NUM_OBSTRUCTION
    global LANE_TO

    if TURTLE.is_occupied():
        return

    if STEP == 0:
        leftside = get_object_distance("leftside")
        left = get_object_distance("left")
        if leftside > 0:
            rospy.logdebug("[PROC] LIDAR LEFTSIDE: {}".format(
                leftside))
        if (leftside > 0) and (leftside < 0.40):
            STEP = 1
            rospy.logdebug("[PROC] STEP changed to {}".format(STEP))

            SCHEDULER.disable_cams()
            rospy.loginfo("[PROC] construction state started.")
            TURTLE.go_forward(3.5)
            rospy.sleep(rospy.Duration(0.5))
            return
        else:
            return
        # elif (left > 0) and (left < 1.5):
        #     STEP = 2
        #     rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        #     return

        # rospy.sleep(rospy.Duration(2))
    elif STEP == 1:
        TURTLE.set_speed("normal")
        TURTLE.set_speed_smooth("slow")
        left = get_object_distance("left")
        if left > 0:
            rospy.logdebug("[PROC] LIDAR LEFT: {}".format(
                left))
        if (left < 0.50) or (left > 1.5):
            return
        else:
            STEP = 3
            rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
            return
    elif STEP == 2:
        # TODO: write code for first left lane
        pass
    elif STEP == 3:
        TURTLE.go_turn("left", speed=0.11)
        LANE_TO = "left"
    elif STEP == 4:
        TURTLE.set_speed("normal")
        reverse_lane()
        biased = get_object_distance(LANE_TO + "_biased")
        if biased > 0:
            rospy.logdebug("[PROC] LIDAR {:s}_BIASED: {}"
                           .format(LANE_TO, biased))
        reverse_lane()

        if (biased == 0) or (biased > 0.30):
            return

        TURTLE.go_turn(LANE_TO, duration=0.5, angular=4.2)
        TURTLE.set_speed("normal")
        # TURTLE.set_speed("fast")
        # if LANE_TO is "left":
        #     TURTLE.turn(0.13, 1.3, consuming_time=1.5)
        # else:
        #     TURTLE.turn(0.13, -1.3, consuming_time=1.5)
        rospy.sleep(rospy.Duration(2.2))
        reverse_lane()

    elif STEP == 5:
        TURTLE.go_turn(LANE_TO, duration=0.7, angular=4.2)
        TURTLE.set_speed("normal")
        # TURTLE.set_speed("fast")
        # if LANE_TO is "left":
        #     TURTLE.turn(0.13, 1.3, consuming_time=1.5)
        # else:
        #     TURTLE.turn(0.13, -1.3, consuming_time=1.5)

        NUM_OBSTRUCTION += 1
        if NUM_OBSTRUCTION < 2:
            STEP = 4
            return

    elif STEP == 6:
        TURTLE.go_forward(1)
        rospy.sleep(rospy.Duration(0.6))
    elif STEP == 7:
        TURTLE.go_turn("left", duration=0.8, angular=3)
    elif STEP == 8:
        TURTLE.set_speed("fast")
        TURTLE.set_speed_smooth("normal")
        TURTLE.go_forward(5)
        rospy.sleep(rospy.Duration(0.5))
    elif STEP == 9:
        # NOTE: turn to parking step
        STEP = 10
        SCHEDULER.set_state("parking")
    else:
        return
    STEP += 1
    rospy.logdebug("[PROC] STEP changed to {}".format(STEP))


# def test_frontcam(image):
#     """ process the frontcam """
#     # rospy.loginfo("[VIEW] frontcam image received.")
#     if not EYE.is_front_occupied():
#         if SCHEDULER.debug_option["show_center_slope"]:
#             SCHEDULER.check_time("frontcam", min=0.5)
#         info = EYE.see_front(image)
#         if info is None:
#             return

#         # rospy.logdebug("[PROC] test_frontcam: state {}  {}".format(
#         #     info["state"], info["horizon_position"]))
#         # if (info["horizon_position"] < 150):
#         #     TURTLE.set_speed("fast")
#         #     TURTLE.set_speed_smooth("normal")

#         if (info["state"] == "turning") and (info["turning_to"] is not "None"):
#             rospy.logdebug(
#                 "[PROC] turn off for 1.5s")
#             rospy.Timer(rospy.Duration(1.5),
#                         EYE.release_front_occupied, oneshot=True)
#         else:
#             rospy.Timer(rospy.Duration(0.1),
#                         EYE.release_front_occupied, oneshot=True)


def test_block_escaping(lidar_data):
    global STEP

    set_lidar_values(lidar_data)

    if not TURTLE.is_occupied():
        if STEP == 1:
            TURTLE.set_speed("normal")
            # TURTLE.go_turn("right", 1, angular=2, speed=0.15)
            front = get_object_distance("front")
            rospy.logdebug("front: {}".format(front))
            if front == 0 or front > 0.40:
                return
            else:
                TURTLE.turn(0.15, -6)
        elif STEP == 2:
            TURTLE.set_speed("normal")
            frontleft = get_object_distance("frontleft")
            rospy.logdebug("frontleft: {}".format(frontleft))
            if frontleft == 0 or frontleft < 0.40:
                return
            else:
                rospy.sleep(rospy.Duration(1.8))
                TURTLE.turn(0.13, 6)
        elif STEP == 3:
            TURTLE.set_speed("slow")
        else:
            return
        STEP += 1
    else:
        print("IS OCCUPIED!")


def test_parking(image):
    global STEP

    if not TURTLE.is_occupied():
        if STEP == 1:
            TURTLE.go_forward(0.5)
        elif STEP == 2:
            TURTLE.set_speed("normal")
            TURTLE.go_turn("right", 2)
        elif STEP == 3:
            TURTLE.go_forward(0.7)
            # for stopping for a while
            rospy.sleep(rospy.Duration(0.5))
        elif STEP == 4:
            TURTLE.go_backward(1)
        elif STEP == 5:
            TURTLE.go_turn_backward(1.2)
        elif STEP == 6:
            TURTLE.go_forward(2)
        elif STEP == 7:
            TURTLE.set_speed("fast")
            TURTLE.set_speed_smooth("normal")
            # TURTLE.set_speed("normal")
        else:
            return
        STEP += 1


initialize()
