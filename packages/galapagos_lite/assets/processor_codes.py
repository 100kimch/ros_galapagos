#!/usr/bin/env python3

# NOTE: should only use "log functions" of rospy in processor.py
import rospy
from lib_eye import EYE
from lib_frontcam import *
from lib_lidar import *
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
    init_ref_images()


def reset_buffer():
    global BUF_ANGULAR
    BUF_ANGULAR = [0] * BUF_SIZE


def reverse_lane():
    global LANE_TO
    if LANE_TO == "right":
        LANE_TO = "left"
    else:
        LANE_TO = "right"

def process_frontcam(image):
    """ process the frontcam image """
    if not SCHEDULER.is_frontcam_enable():
        return

    state = SCHEDULER.get_state()
    info = EYE.see_front(image)

    if SCHEDULER.debug_option["show_front_info"]:
        rospy.logdebug(info)

    if state == "default":
        if EYE.is_boostable(image):
            process_acceleration(info)
        # return
        # SCHEDULER.set_state("to_intersection")
        # signal = is_construction(image)
        # rospy.logdebug(signal)
        # if is_construction(image):
        #     TURTLE.boost()
        #     SCHEDULER.set_state("construction")
        return
    if state == "traffic_light":
        if is_light_green(image):
            TURTLE.enable()
            SCHEDULER.set_state("to_intersection")
        return

    # NOTE: temporary settings:
    if state == "to_intersection":
        # rospy.Timer(rospy.Duration(35), SCHEDULER.enable_lidar, oneshot=True)
        SCHEDULER.set_state("intersection_left")

    # if state == "to_intersection":
    #     signal = check_left_right_sign(image)
    #     if signal == "right":
    #         SCHEDULER.set_state("intersection_right")
    #     elif signal == "left":
    #         SCHEDULER.set_state("intersection_left")
    #     return

    if state == "intersection_right":
        # TODO: make algorithms for right
        if EYE.is_boostable(image):
            TURTLE.boost()
            SCHEDULER.set_state("to_construction")
        return

    # NOTE: temporary settings:
    if state == "intersection_left":
        # if EYE.is_boostable(image):
        #     TURTLE.boost()
        #     SCHEDULER.set_state("to_construction")
        return

    if state == "to_construction":
        # if EYE.is_boostable(image):
            # TURTLE.boost()
        EYE.check_yellow = True
        # if is_construction(image):
            # SCHEDULER.set_state("construction_searching")

    if state == "construction_searching":
        pass


def process_acceleration(info):
    if not info:
        return

    horizon = info['horizon_position']
    center = abs(info['center'])

    if horizon < 150:
        return
    if EYE.get_front_state() == "straight":
        if horizon < 280 and center < 50:
            # TURTLE.boost()
            TURTLE.set_speed_smooth('fast')
            if SCHEDULER.debug_option["show_front_info"]:
                rospy.logwarn("[PROC] Boosting")
        elif horizon < 300 and center < 150:
            TURTLE.set_speed_smooth('normal')
    if horizon > 280 or info['state'] == 'turning':
        TURTLE.set_speed_smooth('normal')
    elif horizon > 310:
        TURTLE.set_speed('normal')


def process_subcam(image):
    """ process the subcam image """

    if not SCHEDULER.is_subcam_enable():
        return
    if SCHEDULER.is_subcam_occupied() or TURTLE.is_occupied():
        return

    info = EYE.see_sub(image)

    if SCHEDULER.debug_option["show_timer"]:
        # Check delay only if has line
        # SCHEDULER.check_time("subcam", min=0.28, stop_when_delay=info["has_line"])
        SCHEDULER.check_time("subcam", min=0.28, stop_when_delay=False)

    if info is None:
        rospy.logwarn("[PROC] No Information!")
        return

    center = info["center"]
    slope = info["slope"]

    if slope < -0.5:
        limit = 1.6
        amplitude = 1.0
    else:
        limit = 1.2
        amplitude = 0.8

    limit /= 1.9
    # amplitude /= 2

    state = SCHEDULER.get_state()
    if (EYE.get_front_state() == "straight") and (state is not "zigzag"):
        if (abs(center) < 30) and slope < -0.4:
            degree = pow(abs(slope) / 1.8, 1.1) * amplitude
        elif center < 0:
            degree = pow(abs(center) / 100, 2.0) * amplitude / 2
        elif center > 0:
            degree = - pow(abs(center) / 100, 2.0) * amplitude
        else:
            degree = 0
    else:
        if slope < 0:
            degree = - pow(abs(slope) / 1.8, 2.9) * amplitude * 28.0
        else:
            degree = pow(abs(slope) / 1.0, 1.2) * amplitude * 4.0

        # degree = pow(abs(slope) / 1.8, 0.9) * amplitude
        # elif center < 0:
        #     degree = pow(abs(center) / 100, 1.9) * amplitude
        # elif center > 0:
        #     degree = - pow(abs(center) / 100, 1.9) * amplitude
        # else:
        #     degree = 0

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
        if center > 200:
            # degree = -1.2
            degree = -1.1  # For enhancing frequency
        # elif EYE.get_front_state() == "straight":
        #     degree = 0.6
        else:
            # degree = 1.4
            degree = 1.3  # For enhancing frequency
    # if not info["has_line"]:
    #     if center < -55:
    #         # degree = 1.6
    #         degree = 1.4  # For slow speed
    #     elif center > 50:
    #         degree = -1.2
    #         # degree = -1.2  # For slow speed
    #     # elif center > 19:
    #     #     degree = -1.2
    #     else:
    #         degree = 1.4

    if SCHEDULER.debug_option["show_center_slope"]:
        rospy.logdebug(
            "[PROC] center: {:.2f}  slope: {:.2f}  degree: {:.2f}  adj: {:.2f}  buf_sum: {:.2f}  {}  {}".format(
                center, slope, degree, adjust_angular, buf_sum, EYE.get_front_state(), info["has_line"])
        )

    rospy.Timer(
        rospy.Duration(0.15), SCHEDULER.release_subcam_occupied, oneshot=True
    )
    TURTLE.turn(0.13, degree)


def process_lidar(lidar_data):
    """ process the lidar image """
    if not SCHEDULER.is_lidar_enable():
        return
    if SCHEDULER.is_lidar_occupied() or TURTLE.is_occupied():
        return

    state = SCHEDULER.get_state()
    if SCHEDULER.debug_option["show_timer"] and (state != "construction"):
        SCHEDULER.check_time("lidar", min=0.4, stop_when_delay=False)

    set_lidar_values(lidar_data)
    state = SCHEDULER.get_state()

    front = get_object_distance("front")

    if (front < 0.15) and (front > 0):
        TURTLE.stop()
        return

    if state is "default":
        leftside = get_object_distance("leftside")
        print(leftside)
        if (leftside < 0.35) and (leftside > 0):
            rospy.Timer(
                rospy.Duration(5), SCHEDULER.release_lidar_occupied, oneshot=True
            )
            SCHEDULER.set_state("to_construction")
            return
    elif state is "to_construction":
        leftside = get_object_distance("leftside")
        print("to_construction: " + str(leftside))
        if (leftside < 0.35) and (leftside > 0):
            SCHEDULER.set_state("construction")
            return
        # rospy.Timer(
        #     rospy.Duration(0.15), SCHEDULER.release_lidar_occupied, oneshot=True
        # )
        # process_construction()
    elif state is "construction":
        process_construction()
    elif state is "parking":
        process_parking()

def process_parking():
    """ process parking state """
    global STEP

    # if TURTLE.is_occupied():
    #     return

    if STEP == 10:
        frontleft = get_object_distance("frontleft")
        frontright = get_object_distance("frontright")
        left = get_object_distance("left")
        right = get_object_distance("right")

        if SCHEDULER.debug_option["show_parking_lidar"]:
            rospy.logdebug("frontright: {:.2f}  frontleft: {:.2f}".format(
                frontright, frontleft
            ))

        if (frontleft > 0) and (frontleft < 1.0):
            STEP = 11
            rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        elif (frontright > 0) and (frontright < 1.0):
            STEP = 12
            rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        if (left > 0) and (left < 0.5):
            STEP = 13
            rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        elif (right > 0) and (right < 0.5):
            STEP = 14
            rospy.logdebug("[PROC] STEP changed to {}".format(STEP))

        # NOTE: return is needed to prevent executing STEP += 1
        return

    elif STEP == 11:
        SCHEDULER.disable_cams()
        TURTLE.set_speed("normal")
        TURTLE.go_forward(2.5)
        TURTLE.go_turn("right", 2)
        STEP = 15
        rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        return

    elif STEP == 12:
        SCHEDULER.disable_cams()
        TURTLE.set_speed("normal")
        TURTLE.go_forward(2.5)
        TURTLE.go_turn("left", 2)
        STEP = 15
        rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        return

    elif STEP == 13:
        SCHEDULER.disable_cams()
        TURTLE.set_speed("normal")
        TURTLE.go_turn("right", angular=1.8, duration=1.2)
        STEP = 15
        rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        return

    elif STEP == 14:
        SCHEDULER.disable_cams()
        TURTLE.set_speed("normal")
        TURTLE.go_turn("left", angular=1.8, duration=1.2)
        STEP = 15
        rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
        return

    elif STEP == 15:
        TURTLE.go_forward(2.0)
        # for stopping for a while
        rospy.sleep(rospy.Duration(0.5))

    elif STEP == 16:
        TURTLE.go_backward(1.5)

    elif STEP == 17:
        TURTLE.go_turn_backward(1.2)

    elif STEP == 18:
        TURTLE.set_speed("normal")
        TURTLE.go_forward(3)
        # rospy.sleep(rospy.Duration(3))
    elif STEP == 19:
        TURTLE.set_speed("fast")
        SCHEDULER.set_state("zigzag")
    else:
        return
    STEP += 1


def process_construction():
    """ process construction state """
    global STEP
    global NUM_OBSTRUCTION
    global LANE_TO
    global BUF_ANGULAR
    global BUF_SIZE

    if TURTLE.is_occupied():
        return

    if STEP == 0:
        # TURTLE.set_speed("normal")
        leftside = get_object_distance("leftside")
        left = get_object_distance("left")

        if leftside > 0:
            rospy.logdebug("[PROC] LIDAR LEFTSIDE: {}".format(
                leftside))

        if (leftside > 0) and (leftside < 0.50) and (left > 1.00):
            # EYE.check_yellow = False
            SCHEDULER.set_state("construction")
            rospy.loginfo("[PROC] construction state started.")

            STEP = 1
            rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
            TURTLE.go_forward(3.5)
            return
        else:
            return

    elif STEP == 1:
        TURTLE.set_speed("normal")
        TURTLE.set_speed_smooth("slow")
        TURTLE.turn(0.13, 0)

        left = get_object_distance("left")
        leftback = get_object_distance("leftback")
        rospy.logdebug("[PROC] LIDAR LEFT: {:.2f} LEFTBACK: {:.2f}}".format(left, leftback))
        if (left > 0) and (left < 0.50):
            return
        else:
            TURTLE.set_speed("slow")
            if (leftback > 0.5):
                # TURTLE.go_forward(2.5)
                STEP = 3
                rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
            return

    elif STEP == 2:
        # TODO: write code for first left lane
        pass

    elif STEP == 3:
        TURTLE.set_speed("normal")
        TURTLE.set_speed_smooth("stop")
        front = get_object_distance("front")
        right_biased = get_object_distance("right_biased")

        if (front > 1.0) and (right_biased < 1.0) and (right_biased > 0.0):
            print("passed", front, right_biased)
            print("BUF_SIZE: ", BUF_SIZE)
            BUF_SIZE = 15
            reset_buffer()
            TURTLE.set_speed_smooth("slow")
            pass
        elif front * right_biased == 0:
            return
        else:
            TURTLE.set_speed("stop")
            print("turning...", front, right_biased)
            TURTLE.turn(0.13, 1.0)
            # rospy.sleep(rospy.Duration(5.0))
            return
        # TURTLE.go_turn("left")

    elif STEP == 4:
        right_biased = get_object_distance("right_biased")
        left_biased = get_object_distance("left_biased")
        front = get_object_distance("front")

        if right_biased == 0.0:
            right_biased = 3.0
        if left_biased == 0.0:
            left_biased = 3.0
        if front == 0.0:
            front = 3.0
        elif front < 0.2:
            TURTLE.set_speed_smooth("stop")
        else:
            TURTLE.set_speed_smooth("slow")

        min_distance = min(right_biased, left_biased)

        degree = 0
        if (front < 1.0):
            degree += max(pow(1.0 - front, 2), 0)
        else:
            degree += max(0.5 - min_distance, 0) * 3
        # if min_distance < 0.5:
            # degree += max((0.5 - min_distance), 0) * 1.5
        # elif (min_distance > 1.0) and (min_distance < 3.0):
            # degree = 0.2
            
    
        if (left_biased == min_distance) and (min_distance < 0.5):
            degree *= -1

        # max_distance = max(right_biased, left_biased)
        # if (left_biased == max_distance):
        #     degree *= -1

        # degree = 0
        # if min_distance > 0 and min_distance < 0.5:
        #     if right_biased > left_biased:
        #         degree = (0.5 - min_distance) * (-7)
        #         LANE_TO = "right"
        #     elif right_biased < left_biased:
        #         degree = (0.5 - min_distance) * (7)
        #         LANE_TO = "left"

        # if is_left_crashable():
        #     degree = -1.7
        # elif is_right_crashable():
        #     degree = 1.7

        degree *= 3
        degree = max(min(degree, 2.0), -2.0)
        BUF_ANGULAR.append(degree)
        degree -= BUF_ANGULAR.pop(0)
        print("BUF_ANGULAR:", BUF_ANGULAR)
        # if degree != 0:
        #     BUF_ANGULAR.append(degree)

        # elif len(BUF_ANGULAR) > 9:
        #     STEP = 5

        if SCHEDULER.debug_option["show_construction_lidar"]:
            rospy.logdebug("[PROC] r_based: {:.2f}  l_based: {:.2f}  min: {:.2f}  front: {:.2f}  deg: {:.2f}"
                           .format(right_biased, left_biased, min_distance, front, degree))

        TURTLE.turn(0.13, degree)
        return

    elif STEP == 5:
        print("[StEP 5]")
        if len(BUF_ANGULAR) > 0:
            TURTLE.turn(0.13, -BUF_ANGULAR.pop(0))
            return
        else:
            front = get_object_distance("front")
            print(front)
            # if (front > 0) and (front < 1.0):
            #     if LANE_TO == "right":
            #         TURTLE.turn(0.13, 0.8)
            #     else:
            #         TURTLE.turn(0.13, -0.8)
            #     return
            # else:
            if NUM_OBSTRUCTION < 1:
                NUM_OBSTRUCTION += 1
                STEP = 4
                return

    elif STEP == 6:
        TURTLE.go_forward(1)
        TURTLE.set_speed("normal")
        TURTLE.go_turn("left")
        TURTLE.set_speed("normal")
        TURTLE.set_speed("fast")
        TURTLE.set_speed_smooth("normal")
        TURTLE.go_forward(5)

        STEP = 10
        SCHEDULER.set_state("parking")
        BUF_SIZE = 3
        reset_buffer()
        return

    STEP += 1
    rospy.logdebug("[PROC] STEP changed to {}".format(STEP))


# def process_construction():
#     """ process construction state """
#     global STEP
#     global NUM_OBSTRUCTION
#     global LANE_TO

#     if TURTLE.is_occupied():
#         return

#     if STEP == 0:
#         leftside = get_object_distance("leftside")
#         left = get_object_distance("left")
#         # front = get_object_distance("front")
#         print(left)
#         if leftside > 0:
#             rospy.logdebug("[PROC] LIDAR LEFTSIDE: {}".format(
#                 leftside))
#         if (leftside > 0) and (leftside < 0.40) and (left > 1.00):
#             STEP = 1
#             rospy.logdebug("[PROC] STEP changed to {}".format(STEP))

#             # SCHEDULER.disable_cams()
#             SCHEDULER.set_state("construction")
#             rospy.loginfo("[PROC] construction state started.")
#             TURTLE.go_forward(3.0)
#             # rospy.sleep(rospy.Duration(0.5))
#             return
#         else:
#             return
#         # elif (left > 0) and (left < 1.5):
#         #     STEP = 2
#         #     rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
#         #     return

#         # rospy.sleep(rospy.Duration(2))
#     elif STEP == 1:
#         TURTLE.set_speed("normal")
#         TURTLE.set_speed_smooth("slow")
#         left = get_object_distance("left")
#         if left > 0:
#             rospy.logdebug("[PROC] LIDAR LEFT: {}".format(
#                 left))
#         if (left < 0.50) or (left > 1.5):
#             return
#         else:
#             STEP = 3
#             rospy.logdebug("[PROC] STEP changed to {}".format(STEP))
#             return
#     elif STEP == 2:
#         # TODO: write code for first left lane
#         pass
#     elif STEP == 3:
#         TURTLE.go_turn("left")
#         LANE_TO = "left"
#     elif STEP == 4:
#         TURTLE.set_speed("normal")
#         reverse_lane()
#         biased = get_object_distance(LANE_TO + "_biased")
#         if biased > 0:
#             rospy.logdebug("[PROC] LIDAR {:s}_BIASED: {}"
#                            .format(LANE_TO, biased))
#         reverse_lane()

#         if (biased == 0) or (biased > 0.35):
#             return

#         TURTLE.go_turn(LANE_TO, duration=0.5, angular=1.8)
#         TURTLE.set_speed("normal")
#         # TURTLE.set_speed("fast"
#         # if LANE_TO is "left":
#         #     TURTLE.turn(0.13, 1.3, consuming_time=1.5)
#         # else:
#         #     TURTLE.turn(0.13, -1.3, consuming_time=1.5)
#         rospy.sleep(rospy.Duration(1.0))
#         reverse_lane()

#     elif STEP == 5:
#         TURTLE.go_turn(LANE_TO, duration=0.5, angular=1.8)
#         TURTLE.set_speed("normal")
#         # TURTLE.set_speed("fast")
#         # if LANE_TO is "left":
#         #     TURTLE.turn(0.13, 1.3, consuming_time=1.5)
#         # else:
#         #     TURTLE.turn(0.13, -1.3, consuming_time=1.5)

#         NUM_OBSTRUCTION += 1
#         if NUM_OBSTRUCTION < 2:
#             STEP = 4
#             return

#     elif STEP == 6:
#         TURTLE.go_forward(2.5)
#         rospy.sleep(rospy.Duration(0.6))
#     elif STEP == 7:
#         TURTLE.go_turn("left")
#     elif STEP == 8:
#         TURTLE.set_speed("fast")
#         TURTLE.set_speed_smooth("normal")
#         TURTLE.go_forward(5)
#         rospy.sleep(rospy.Duration(0.5))
#     elif STEP == 9:
#         # NOTE: turn to parking step
#         STEP = 10
#         SCHEDULER.set_state("parking")
#     else:
#         return
#     STEP += 1
#     rospy.logdebug("[PROC] STEP changed to {}".format(STEP))


initialize()
