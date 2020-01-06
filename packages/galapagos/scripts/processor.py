#!/usr/bin/env python3

# from lib_line_tracing # ! Deprecated
# from lib_signal_recognition # ! Deprecated

# NOTE: using rospy library unrecommended in processor.py
import rospy
# NOTE: python 3.5^ needed to use asyncio 
import asyncio
from lib_frontcam import *
from lib_fishcam import *
from lib_lidar import *
from lib_eye import *
from lib_parking import *
from turtlebot import TURTLE
from constants import IS_DEBUG_MODE, SELECTED_STATE
# NOTE: to check time:
import timeit
import math
# * Variables

################ Variables by minsoo #######################
CURRENT_STATE = 'inter_curving'
#CURRENT_STATE = 'intersection'
MOVING_POSITION = False
SEEN_PARKING_SIGN = False
SEEN_TUNNEL_SIGN = False
IS_IN_TUNNEL = False
# 
# HAS_OBJECT_IN_50 = False
# HAS_OBJECT_IN_20 = False

straight_cnt = 0
curving_cnt = 0

LIDAR_FLAG = False
############################################################
GALAPAGOS_STATE = "view"
'''
if SELECTED_STATE == '':
    CURRENT_STATE = 'traffic_light'
else:
    CURRENT_STATE = SELECTED_STATE
'''
LINE_BASE = 'both'
MOVING_POSITION = False  # ! Deprecated
DIRECTION = 'left'
SEEN_LEFT_SIGN = False
SEEN_PARKING_SIGN = False  # ! Deprecated
SEEN_TUNNEL_SIGN = False  # ! Deprecated
IS_IN_TUNNEL = False
SEEN_STOPPING_SIGN = False
SIGN_CORNER = None
# DISTANCE_FRONT = 0.00  # ! Deprecated
# HAS_OBJECT_IN_50 = False
# HAS_OBJECT_IN_20 = False
HAS_BOTH_LINES = False
IS_TURNING = False

TRACKING = "fish"
TURNING_TO = False

TEST_ANGULAR = 0
TEST_ONCE = True

# STATE_CONSTRUCTION = "start"
STATE_CONSTRUCTION = "searching"
STATE_TUNNEL = "inside"

TURN_COUNT = 0
# * Methods

def initialize():
    EYE.calibrate()
    TURTLE.set_speed('normal')


def reset_front_image_flags():
    global LINE_BASE
    global MOVING_POSITION
    global SEEN_PARKING_SIGN
    global SEEN_TUNNEL_SIGN
    global SEEN_STOPPING_SIGN
    LINE_BASE = 'both'
    MOVING_POSITION = False
    SEEN_PARKING_SIGN = False
    SEEN_TUNNEL_SIGN = False
    SEEN_STOPPING_SIGN = False


def reverse_direction():
    global DIRECTION
    if DIRECTION == 'right':
        DIRECTION = 'left'
    else:
        DIRECTION = 'right'
    rospy.loginfo('\n[PROC] direction changed to ' + DIRECTION)

def track_front(event=None):
    global TRACKING
    
    EYE.reset_state()
    TRACKING = "front"
    # rospy.loginfo("\n[PROC] tracking changed to " + TRACKING)
    return

def track_fish(event=None):
    global TRACKING

    TRACKING = "fish"
    # rospy.loginfo("\n[PROC] tracking changed to " + TRACKING)
    return

def process_frontcam(image):
    """ process the image of raspicam """
    global CURRENT_STATE
    global MOVING_POSITION
    global SEEN_PARKING_SIGN
    global SEEN_TUNNEL_SIGN
    global LIDAR_FLAG
    
    raw_data = np.fromstring(image.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)

    #### ROI SETTING ######
    blob_ROI = cv_img[100:, :]
    #######################
    if CURRENT_STATE == 'traffic_light':
        if is_light_green(cv_img):
            TURTLE.enable()
            #TURTLE.set_speed('fast')
            print("detected green")
            CURRENT_STATE = 'intersection'
            #TURTLE.set_weight(0.8)
            return
        else:
            print("no green")
            TURTLE.enable()
            #TURTLE.set_speed('fast')
            print("detected green")
            CURRENT_STATE = 'intersection'
            #TURTLE.set_weight(0.8)
            return

    if CURRENT_STATE == 'intersection':
        cv2.imshow("blob_ROI",blob_ROI)
        # cv2.waitKey(1)
        print("intersection state")
        if is_intersection(cv_img):
            TURTLE.set_weight(0.8)
            CURRENT_STATE = 'left_or_right'
            print("intersection detected!!")
            return
        else:
            return


    if CURRENT_STATE == 'left_or_right':
        print("left or right state")
        cv2.imshow("blob_ROI",blob_ROI)
        # cv2.waitKey(1)
        tmp_state = check_left_right_sign(blob_ROI)
        print("tmp state: ",tmp_state)
        if tmp_state == 'right':
            #print("11tmp state: ",tmp_state, ", right cnt: ",inter_right_cnt)
            TURTLE.LINE_BASE = 2
            #print("11tmp state: ",tmp_state, ", right cnt: ",inter_right_cnt)
            CURRENT_STATE = 'inter_curving'
            TURTLE.set_weight(1.0)
        elif tmp_state == 'left':
            #print("11tmp state: ",tmp_state, ", left cnt: ",inter_left_cnt)
            TURTLE.LINE_BASE = 1
            #print("22tmp state: ",tmp_state, ", left cnt: ",inter_left_cnt)
            CURRENT_STATE = 'inter_curving'
            TURTLE.set_weight(1.0)
        elif tmp_state == 'none':
            return
    
    if CURRENT_STATE == 'inter_curving':
        print("#################################################")
        print("########### inter_curving state #################")
        print("#################################################")
        global straight_cnt
        if abs(TURTLE.weight*TURTLE._angular) < 0.1:
            straight_cnt += 1
            print("straight counting : ",straight_cnt," is counted")
            if straight_cnt > 5:
                straight_cnt = 0
                TURTLE.LINE_BASE = 2
                CURRENT_STATE = 'construction'
                return
            else:
                return
        else:
            straight_cnt = 0
            return

    if CURRENT_STATE == 'construct_recog':
        tmp_state = is_construction(blob_ROI)
        print(tmp_state)
        if tmp_state is True:
            TURTLE.LINE_BASE = 2
            CURRENT_STATE = 'construction'
            #LIDAR_FLAG = True
        else:
            return

    if CURRENT_STATE == 'construction':
        return
        
    # if CURRENT_STATE == 'construction':
    #     '''
    #     task for Ji-hyung
    #     '''

    #     TURTLE.LINE_BASE = 1
    #     CURRENT_STATE = 'parking'
    #     pass
    '''
    if CURRENT_STATE == 'stop_sign':
        if stop_sign_flag == False:
            sign_stop = is_stopping_sign(image)
            if sign_stop == True:
                stop_sign_flag = True
                return
            else:
                return
        else:
            sign_stop = is_stopping_sign(image)
            if sign_stop == False:
                stop_false_cnt = stop_false_cnt + 1
                if stop_false_cnt > 6 :
                    CURRENT_STATE = 'construction'
                    return
                else:
                    return
            else:
                stop_false_cnt = 0
                return
                    

            
    # if CURRENT_STATE == 'construction':
        # TURTLE.set_speed("slow")
        # if HAS_OBJECT_IN_20:
            TURTLE.turn(MOVING_POSITION, 15, 3)
            if MOVING_POSITION == 'left':
                MOVING_POSITION = 'right'
            else:
                MOVING_POSITION = 'left'
            return
        else:
            if has_crossing_line(image):
                if MOVING_POSITION == 'left':
                    moving_to = 'right'
                else:
                    moving_to = 'left'
                TURTLE.turn(moving_to, 15, 3)
            else:
                # TODO: combine trace_line() + trace_blocking()
                if get_num_of_lines(image) == 2:
                    CURRENT_STATE = 'normal'
                else:
                    return

    if CURRENT_STATE == 'parking':
        # TODO: finish code of parking state
        TURTLE.set_speed('normal')
        return

    if CURRENT_STATE == 'blocking_bar':
        # TODO: finish code of blocking_bar state
        TURTLE.set_speed('normal')
        return

    if CURRENT_STATE == 'tunnel':
        # TODO: finish code of tunnel state
        TURTLE.set_speed('normal')
        return

    # ending the normal state:
    if CURRENT_STATE == 'normal':
        reset_front_image_flags()
        TURTLE.set_speed('fast')
        return
    '''


'''
def process_frontcam(image):
    """ process the image of raspicam """
    global CURRENT_STATE
    global MOVING_POSITION
    global SEEN_PARKING_SIGN
    global SEEN_TUNNEL_SIGN
    global LINE_BASE
    global SEEN_STOPPING_SIGN

    if CURRENT_STATE == 'traffic_light':
        if is_light_green(image):
            if IS_DEBUG_MODE == True:
                TURTLE.set_speed('fast')
                rospy.loginfo('\n[PROC] Current state: normal')
                CURRENT_STATE = 'normal'
            else:
                rospy.logdebug('Debug mode finished')
                raise rospy.ROSInterruptException
        else:
            return

    if CURRENT_STATE == 'normal':
        LINE_BASE = 'both'
        sign = check_sign(image)

        if sign == 'intersection':
            TURTLE.set_speed('normal')
            rospy.loginfo('\n[PROC] Current state: intersection')
            CURRENT_STATE = 'intersection'

        elif sign == 'construction':
            TURTLE.set_speed('slow')
            rospy.loginfo('\n[PROC] Current state: construction')
            DIRECTION = 'right'
            LINE_BASE = 'left'
            CURRENT_STATE = 'construction'

        elif sign == 'parking':
            # SEEN_PARKING_SIGN = True # ! Deprecated
            TURTLE.set_speed('normal')
            rospy.loginfo('\n[PROC] Current state: parking')
            CURRENT_STATE = 'parking'
            # return

        elif HAS_OBJECT_IN_50:
            TURTLE.set_speed('slow')
            rospy.loginfo('\n[PROC] Current state: blocking_bar')
            CURRENT_STATE = 'blocking_bar'

        elif sign == 'tunnel':
            # SEEN_TUNNEL_SIGN = True # ! Deprecated
            TURTLE.set_speed('normal')
            rospy.loginfo('\n[PROC] Current state: tunnel')
            TURTLE.turn_by_degree(45, 0)  # TODO: support integer deg value
            # return

        elif is_straight_in(10, image):
            if is_straight_in(50, image):
                TURTLE.increase_speed()
            else:
                TURTLE.decrease_speed()
            return
        else:
            TURTLE.set_speed('normal')
            return

    if CURRENT_STATE == 'intersection':
        sign_corner = check_sign(image)
        if sign_corner == None:
            if SEEN_STOPPING_SIGN:
                TURTLE

        if sign_corner == None:
            LINE_BASE = 'both'
        else:
            if sign_corner == 'left':
                LINE_BASE = 'left'
            elif sign_corner == 'right':
                LINE_BASE = 'right'

            if is_stopping_sign(image):
                SEEN_STOPPING_SIGN = True
            else:
                if SEEN_STOPPING_SIGN:
                    CURRENT_STATE = 'normal'
                else:
                    return

    # if CURRENT_STATE == 'construction':
        # TURTLE.set_speed('slow')
        # if HAS_OBJECT_IN_20:
            TURTLE.turn(MOVING_POSITION, 15, 3)
            if MOVING_POSITION == 'left':
                MOVING_POSITION = 'right'
            else:
                MOVING_POSITION = 'left'
            return
        else:
            if has_crossing_line(image):
                if MOVING_POSITION == 'left':
                    moving_to = 'right'
                else:
                    moving_to = 'left'
                TURTLE.turn(moving_to, 15, 3)
            else:
                # TODO: combine trace_line() + trace_blocking()
                if get_num_of_lines(image) == 2:
                    CURRENT_STATE = 'normal'
                else:
                    return

    if CURRENT_STATE == 'parking':
        # TODO: finish code of parking state
        TURTLE.set_speed('normal')
        return

    if CURRENT_STATE == 'blocking_bar':
        # TODO: finish code of blocking_bar state
        TURTLE.set_speed('normal')

        return

    if CURRENT_STATE == 'tunnel':
        # TODO: finish code of tunnel state
        TURTLE.set_speed('normal')
        return

    # ending the normal state:
    if CURRENT_STATE == 'normal':
        if IS_DEBUG_MODE == True:
            rospy.logdebug('Debug mode finished')
            raise rospy.ROSInterruptException
        else:
            rospy.loginfo('\n[PROC] Current state: normal')
            reset_front_image_flags()
            TURTLE.set_speed('fast')
        return
'''

def idle(event=None):
    """ idle process """
    global TRACKING

    rospy.loginfo("\n[PROC] idle executed")
    TRACKING = "idle"
    # TURTLE.disable()
    rospy.spin()
    # while True:
    #     rospy.loginfo('tracking:' + TRACKING)
    return

def process_fishcam(image):
    """ trace side lines by base
    if base is 'left: trace left line
    if base is 'both': trace both lines
    if base is 'right': trace right lne
    return value is not needed.
    """
    if TURTLE._enable_running is False:
        return
    if TURTLE.enable_fish is False:
        return

    if TURTLE.LINE_BASE == 3:
        trace_line(image)
    elif TURTLE.LINE_BASE == 1 or TURTLE.LINE_BASE == 2:
        # rospy.loginfo("[LINE] trace_one_line(" + str(LINE_BASE) + ")")
        trace_one_line(image, TURTLE.LINE_BASE)
    # use TURTLE.set_
    # use TURTLE.set_angular(angular)
    return
    

'''
def process_fishcam(image):
    """ process the fisheye lens image """
    start = timeit.default_timer()

    global LINE_BASE
    global TRACKING
    global TURNING_TO
    global TEST_ANGULAR
    global TEST_ONCE

    if TRACKING is not "fish":
        return

    if not EYE.is_fish_occupied():
    # if True:
        info = EYE.see_bottom(image)
        # rospy.Timer(rospy.Duration(0.04), EYE.release_fish_occupied, oneshot=True, reset=True)

        if info is None:
            print("NO INFO!")
            # TURTLE.set_angular_smooth(0.12)
            # pass
        else:
            # rospy.loginfo("\n[PROC] info: " + str(info))
            # TURTLE.set_speed('slow')
            # if info["slope"]:
            #     TURTLE.set_speed_by_percentage(-abs(info["slope"] / 6))
            # else:
            TURTLE.set_speed('normal')

            if info["right"] < 640:
                rospy.loginfo("\n[PROC] info: " + str(info))
                TURTLE.set_angular(0.75 + abs(info["slope"]) * 1.8)
                TURTLE.set_angular_smooth(-0.1)
            else:
                if TEST_ANGULAR is not 1:
                    if info["left"] is 0:
                        if TEST_ONCE:
                            TURTLE.set_angular(0.12)
                            TURTLE.set_angular_smooth(0.05)
                        else:
                            TURTLE.set_angular(0.12)
                            TURTLE.set_angular_smooth(0.05)
                            TEST_ONCE = False
                    # elif info["left"] < 7 and info["left"] > 0:
                    #     TURTLE.set_angular_smooth(0.1)
                    TEST_ANGULAR = 1

                if info["left"] > 0 and info["left"] <= 10:
                    TURTLE.set_angular(0)
                    TEST_ANGULAR = 0

                # if TEST_ANGULAR is not -1:
                if info["left"] > 10:
                    TURTLE.set_angular(-0.75 + -abs(info["slope"]) * 1.8)
                    TURTLE.set_angular_smooth(-0.1)
                    TEST_ANGULAR = -1


        rospy.Timer(rospy.Duration(0.05), EYE.release_fish_occupied, oneshot=True, reset=True)
        # EYE.release_fish_occupied()
        end = timeit.default_timer()
        print("l: {:d}".format(info["left"]) + "   s: {:.01f}".format(info["slope"])
            + "  time: {:.02f}".format(end - start))
    # print(end - start)
    # print("turning to: " + str(TURNING_TO))
    # if TURNING_TO:
    #     TURTLE.turn(TURNING_TO, 2.3)
    #     TURNING_TO = None
    # rospy.Timer(rospy.Duration(2.3), track_front, oneshot=True, reset=True)


    # else:
        # track_front()
    # print("awaiting finished")
    # idle()
    # track_front()

    # rospy.Timer(rospy.Duration(5), idle)

    # rospy.loginfo('\n[PROC] fish image received')
    # trace_line_by_base(image, 2)
    # trace_line(image)
    # TURTLE.set_angular(trace_line_by_base(LINE_BASE))
    # trace_line(image) # ! Deprecated
    # print(TURTLE.get_info())
    # TURTLE.move()
    return
'''

def process_subcam(image):
    """ process the subcam image """
    start = timeit.default_timer()

    global LINE_BASE
    global TRACKING
    global TURNING_TO
    global TEST_ANGULAR
    global TEST_ONCE

    if TRACKING is not "fish":
        return

    if not EYE.is_sub_occupied():
    # if True:
        rospy.Timer(rospy.Duration(0.04), EYE.release_sub_occupied, oneshot=True)
        info = EYE.see_sub(image)
        # rospy.Timer(rospy.Duration(0.04), EYE.release_sub_occupied, oneshot=True, reset=True)

        if info is None:
            print("NO INFO!")
            # TURTLE.set_angular_smooth(0.12)
            # pass
        else:
            # TURTLE.set_speed('slow')
            # if info["slope"]:
            #     TURTLE.set_speed_by_percentage(-abs(info["slope"] / 6))
            # else:
            TURTLE.set_speed('normal')

            center_x, center_y, point_x, point_y = (value for value in info["line_center"])
            gap_x = abs(point_x - center_x)
            rospy.loginfo("\n[PROC] info: \n" + str(info))
            rospy.loginfo("\nGAP: " + str(gap_x))

            if info["slope"] > 0:
                slope = math.sqrt(0.5 * info["slope"])
            else:
                slope = - math.sqrt(-0.5 * info["slope"])
            
            if not info["has_line"]:
                if gap_x > 100:
                    if TEST_ONCE:
                        TURTLE.set_angular(-1.75)
                        TURTLE.set_angular_smooth(-0.1)
                    else:
                        TURTLE.set_angular_smooth(-0.1)
                elif gap_x < 0:
                    if TEST_ONCE:
                        TURTLE.set_angular(1.75)
                        TURTLE.set_angular_smooth(0.2)
                    else:
                        TURTLE.set_angular_smooth(0.2)
                TEST_ONCE = False
            # elif gap_x > 100:
            #     TURTLE.set_angular(-0.15 * info["slope"] * 4)
            #     TURTLE.set_angular_smooth(-0.1)
            else:
                TEST_ONCE = True
                if gap_x > 100:
                    TURTLE.set_angular(-0.15 + slope)
                    TURTLE.set_angular_smooth(-0.1)
                elif gap_x > 30:
                    TURTLE.set_angular(0)
                else:
                    # if slope < 0:
                    #     TURTLE.set_angular(-0.25 + slope * 1.5)
                    #     TURTLE.set_angular_smooth(-0.1)
                    # else:
                    rospy.loginfo("LOST")
                    TURTLE.set_angular(0.45 + slope)
                    TURTLE.set_angular_smooth(0.1)

        # EYE.release_sub_occupied()
        end = timeit.default_timer()
        # print("sub l: {:d}".format(info["left"]) + "   s: {:.01f}".format(slope)
        #     + "  time: {:.02f}".format(end - start))
        print("\nTIME: {:.02f}".format(end - start))

def process_tunnel(lidar_data):
    global LIDAR_FLAG
    if LIDAR_FLAG is False:
        return
    global DIRECTION
    TURTLE.enable_fish = False
    set_lidar_values(lidar_data)
    TURTLE.set_speed_smooth("fast")
    directions = ["front", "leftside", "rightside"]
    min_distance = 1000
    for direction in directions:
        distance = get_object_distance(direction)
        if distance > 0 and distance < min_distance:
            min_distance = distance
    # distance = min(get_object_distance("front"), get_object_distance("leftside"), get_object_distance("rightside"))
    # distance = get_object_distance("front")
    rospy.loginfo(str(min_distance) + str(distance))
    if min_distance < 0.50 and min_distance > 0:
        TURTLE.turn(DIRECTION, 2.03, True, True)
        if DIRECTION == "left":
            DIRECTION = "right"
        elif DIRECTION == "right":
            DIRECTION = "left"

def process_tunnel2(lidar_data):
    global LIDAR_FLAG
    if LIDAR_FLAG is False:
        return
    global DIRECTION
    global IS_TURNING

    TURTLE.enable_fish = False
    set_lidar_values(lidar_data)
    TURTLE.set_speed_smooth("fast")
    directions = ["front", "leftside", "rightside"]
    distances = {}
    for direction in directions:
        distances[direction] = get_object_distance(direction)


    if not IS_TURNING:
        print(distances["leftside"], distances["rightside"])
        if distances["leftside"] < 0.45 and distances["leftside"] > 0:
            TURTLE.turn("right", 1.5)
            DIRECTION = "left"
            IS_TURNING = True
        elif distances["rightside"] < 0.45 and distances ["rightside"] > 0:
            TURTLE.turn("left", 1.5)
            DIRECTION = "right"
            IS_TURNING = True
    else:
        print(get_object_distance("front"))
        if get_object_distance("front") == 0:
            rospy.signal_shutdown("[TURTLE] shutting down...")
        else:
            TURTLE.turn(DIRECTION, 1.5)
        # TURTLE.turn(DIRECTION, 1.5)
        IS_TURNING = False

def process_eye(image):
    """ process the Eye System """
    global TRACKING
    global TURNING_TO

    # if TRACKING is not "front":
    #     return
    # rospy.loginfo('\n[PROC] frontcam image received')
    # rospy.loginfo(EYE.is_occupied())

    if not EYE.is_front_occupied():
        info = EYE.see(image)
        # rospy.loginfo("\n[PROC] info: " + str(info))
        rospy.Timer(rospy.Duration(0.1), EYE.release_occupied, oneshot=True)
        
        if TRACKING is not "front":
            return
        if info is None:
            pass
        else:
            center = info["center"]
            TURTLE.set_speed('normal')

            if info["state"] is "straight":
                # TURTLE.set_speed('normal')
                pass
            elif info["state"] is "lost_track":
                rospy.loginfo("\n[PROC] state: lost_track")
                TURNING_TO = info["turning_to"]
                TURTLE.go_forward(1)
                rospy.Timer(rospy.Duration(1), track_fish, oneshot=True, reset=True)
            # elif info["state"] is "turning":
            #     TURNING_TO = info["turning_to"]
            #     track_fish()
                # rospy.Timer(rospy.Duration(5.5), track_fish, oneshot=True, reset=True)

            if center > 500:
                TURTLE.set_angular(0.27)
                rospy.loginfo("\n[PROC] [!] turning to right at " + str(center))
                return
            if center < -500:
                TURTLE.set_angular(-0.27)
                rospy.loginfo("\n[PROC] [!] turning to left at " + str(center))
                return
            if center > 60:
                TURTLE.set_angular(0.12)
                # TURTLE.set_angular(0.08)
                rospy.loginfo("\n[PROC] turning to right at " + str(center))
                return
            elif center < -60:
                TURTLE.set_angular(-0.12)
                # TURTLE.set_angular(-0.08)
                rospy.loginfo("\n[PROC] turning to left at " + str(center))
                return
        TURTLE.set_angular_smooth(0)
    return

def process_blocking(lidar_data):
    global DISTANCE_FRONT

    set_lidar_values(lidar_data)

    distance_1 = get_object_distance('front')
    distance_2 = get_object_distance('leftside')
    distance_3 = get_object_distance('rightside')
    rospy.loginfo("\n[PROC] front: " + str(distance_1) + "  " + str(distance_2) + "  " + str(distance_3))

def process_lidar(lidar_data):
    global CURRENT_STATE
    if CURRENT_STATE is not "construction":
        return
    """ process the lidar data """
    # global IS_IN_TUNNEL
    # global DISTANCE_FRONT
    # global HAS_OBJECT_IN_20
    global IS_TURNING
    # global SEEN_PARKING_SIGN
    # global SEEN_LEFT_SIGN
    global STATE_CONSTRUCTION
    global DIRECTION
    global TURN_COUNT

    if TURN_COUNT == 3:
        CURRENT_STATE = "parking"
        rospy.signal_shutdown("\n[PROC] Shutting down...")
    set_lidar_values(lidar_data)

    if not TURTLE.is_settable():
        return

    rospy.loginfo(str(STATE_CONSTRUCTION) + "    " + str(get_object_distance('left')) + "   " + str(get_object_distance('rightside')))

    if STATE_CONSTRUCTION == "searching":
        distance = get_object_distance("left")
        if distance == 0:
            return
        elif distance < 0.35:
            STATE_CONSTRUCTION = "ready"
        elif distance > 0.5 and distance < 0.9:
            STATE_CONSTRUCTION = "start"
            DIRECTION = "right"
            TURTLE.enable_fish = False
            TURTLE.turn("left", 2.03, True)
        return

    elif STATE_CONSTRUCTION == "ready":
        distance = get_object_distance("front")
        if distance == 0:
            return
        elif distance < 1:
            STATE_CONSTRUCTION = "fitting"
            DIRECTION = "left"
            TURTLE.enable_fish = False
            # TURTLE.set_angular(0)
            TURTLE.turn("right", 0.5, True)
        return

    elif STATE_CONSTRUCTION == "fitting":
        TURTLE.turn("right", 0.5, True)
        STATE_CONSTRUCTION = "start"

        
    elif STATE_CONSTRUCTION == "start":
        if not IS_TURNING:
            if DIRECTION == 'right':
                distance = get_object_distance('leftside')
            else:
                distance = get_object_distance('rightside')

            rospy.loginfo('\n[PROC] distance on ' + DIRECTION + ' : ' + str(distance))
            if distance == 0:
                return
            elif distance < 0.35:
                TURTLE.turn(DIRECTION, 1.2)
                IS_TURNING = True
            else:
                TURTLE.set_speed('normal')
        else:
            if DIRECTION == 'right':
                distance = get_object_distance('frontleft')
            else:
                distance = get_object_distance('frontright')

            # rospy.loginfo('\n[PROC] turning distance on ' +
                        # DIRECTION + ' : ' + str(distance))
            if distance > 0.34 or distance == 0:
                reverse_direction()
                TURTLE.turn(DIRECTION, 1.2)
                TURN_COUNT = TURN_COUNT + 1
                IS_TURNING = False

    # """ parking control using lidar_data """
    # if SEEN_PARKING_SIGN:
    #     parking_control(lidar_data)
    #     #TODO : one line tracer start

    #     if SEEN_LEFT_SIGN:
    #         SEEN_PARKING_SIGN = False
    #         rospy.sleep(rospy.Duration(10))
    #         SEEN_LEFT_SIGN = False
    #         #TODO : two line tracer start

def set_package_state(msg):
    """ callback function to stop viewers """
    global GALAPAGOS_STATE

    rospy.loginfo("msg received: ", msg.data)
    GALAPAGOS_STATE = msg.data

def view_frontcam(image):
    """ view the Eye System """
    if GALAPAGOS_STATE is not "view":
        return
    # rospy.loginfo("\n[VIEWER] viewing frontcam...")
    if not EYE.is_front_occupied():
        EYE.see(image)
        # rospy.loginfo("\n[PROC] info: " + str(info))
        rospy.Timer(rospy.Duration(0.1), EYE.release_occupied, oneshot=True)

def view_fishcam(image):
    """ view the fisheye lens image """
    if GALAPAGOS_STATE is not "view":
        return
    # rospy.loginfo("\n[VIEWER] viewing fishcam...")
    if not EYE.is_fish_occupied():
        info = EYE.see_bottom(image)
        rospy.Timer(rospy.Duration(0.1), EYE.release_fish_occupied, oneshot=True, reset=True)

def view_subcam(image):
    """ view the fisheye lens image """
    if GALAPAGOS_STATE is not "view":
        return
    # rospy.loginfo("\n[VIEWER] viewing fishcam...")
    if not EYE.is_front_occupied():
        info = EYE.see_sub(image)
        rospy.loginfo("\n[VIEW] info: \n" + str(info))
        rospy.Timer(rospy.Duration(0.1), EYE.release_sub_occupied, oneshot=True, reset=True)





#! Deprecated


# def test_line_tracing(image):
#     trace_one_line(image, 2)
#     #trace_line(image)
#     # gogo()

#! Deprecated


# def test_sudden_stop(image):
#     global DISTANCE_FRONT

#     if RUN_MODE == 'debug':
#         rospy.logdebug('fish_image received')

#     process_fishcam(image)

#     if DISTANCE_FRONT == 0:
#         TURTLE.enable()
#         rospy.loginfo('\n[PROC] turtlebot enabled.')
#     elif DISTANCE_FRONT < 0.20:
#         TURTLE.stop()
#         rospy.loginfo('\n[PROC] turtlebot stopped.')
#         return
#     elif DISTANCE_FRONT < 0.50:
#         # TODO: decrease speed
#         pass

#     # trace_line(image)
#     TURTLE.set_speed_by_percentage(1)
#     TURTLE.move()


# * Initialization
reset_front_image_flags()
initialize()
