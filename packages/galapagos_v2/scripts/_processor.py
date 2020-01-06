#!/usr/bin/env python3

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

CURRENT_STATE = 'traffic_light'
# CURRENT_STATE = 'inter_curving'
GALAPAGOS_STATE = "view"
COUNT_STRAIGHT = 0
ENABLE_LIDAR = False
LINE_BASE = 'both'
DIRECTION = 'right'
SEEN_LEFT_SIGN = False
SEEN_BLOCKING_BAR = False
SIGN_CORNER = None
HAS_BOTH_LINES = False
IS_TURNING = False
IS_INSIDE_TUNNEL = False
TRACKING = "fish"
TURNING_TO = False
TEST_ONCE = True
# STATE_CONSTRUCTION = "start"
STATE_CONSTRUCTION = "searching"
STATE_TUNNEL = "inside"
TURN_COUNT = 0
######################### add enable_frontcam flag ###############################
ENABLE_FRONT = True
##################################################################################

# * Methods

def initialize():
    """ initialize processing """
    EYE.calibrate()
    TURTLE.set_speed('normal')
    if CURRENT_STATE is "traffic_light":
        TURTLE.disable()
    else:
        TURTLE.enable()

def process_frontcam(image):
    global ENABLE_FRONT
    if ENABLE_FRONT is False:
        return
    """ process the image of raspicam """
    global CURRENT_STATE
    
    raw_data = np.fromstring(image.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)
    # NOTE: ROI Setting
    blob_ROI = cv_img[100:, :]

    if CURRENT_STATE == 'traffic_light':
        if is_light_green(cv_img):
            TURTLE.enable()
            #TURTLE.set_speed('fast')
            print("detected green")
            CURRENT_STATE = 'left_or_right'
            print_state()
            #TURTLE.set_weight(0.8)
            return
        else:
            print("no green")
            """"
            TURTLE.enable()
            #TURTLE.set_speed('fast')
            print("detected green")
            CURRENT_STATE = 'left_or_right'
            print_state()
            #TURTLE.set_weight(0.8)
            """
            return
    '''
    if CURRENT_STATE == 'intersection':
        cv2.imshow("blob_ROI",blob_ROI)
        # cv2.waitKey(1)
        print("intersection state")
        if is_intersection(cv_img):
            TURTLE.set_weight(0.8)
            CURRENT_STATE = 'left_or_right'
            print_state()
            return
        else:
            return
    '''

    if CURRENT_STATE == 'left_or_right':
        print("left or right state")
        cv2.imshow("blob_ROI",blob_ROI)
        # cv2.waitKey(1)
        tmp_state = check_left_right_sign(blob_ROI)
        print("tmp state: ",tmp_state)
        if tmp_state == 'right':
            #print("11tmp state: ",tmp_state, ", ri ght cnt: ",inter_right_cnt)
            TURTLE.LINE_BASE = 2
            #print("11tmp state: ",tmp_state, ", right cnt: ",inter_right_cnt)
            CURRENT_STATE = 'inter_curving'
            print_state()
            TURTLE.set_weight(1.0)
        elif tmp_state == 'left':
            #print("11tmp state: ",tmp_state, ", left cnt: ",inter_left_cnt)
            TURTLE.LINE_BASE = 1
            #print("22tmp state: ",tmp_state, ", left cnt: ",inter_left_cnt)
            CURRENT_STATE = 'inter_curving'
            print_state()
            TURTLE.set_weight(1.0)
        elif tmp_state == 'none':
            return
    
    if CURRENT_STATE == 'inter_curving':
        global COUNT_STRAIGHT
        global ENABLE_LIDAR
        if abs(TURTLE.weight*TURTLE._angular) < 0.1:
            COUNT_STRAIGHT += 1
            print("straight counting : ", COUNT_STRAIGHT, " is counted")
            if COUNT_STRAIGHT > 25:
                COUNT_STRAIGHT = 0
                TURTLE.LINE_BASE = 2
                ENABLE_LIDAR = True
                CURRENT_STATE = "construction"
                print_state()
                return
            else:
                return
        else:
            COUNT_STRAIGHT = 0
            return

    if CURRENT_STATE is "before_parking":
        TURTLE.LINE_BASE = 1
        rospy.Timer(rospy.Duration(10), set_state_to_parking, oneshot=True)
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

def process_lidar(lidar_data):
    """ process the lidar data """
    global CURRENT_STATE
    global IS_TURNING
    global STATE_CONSTRUCTION
    global DIRECTION
    global TURN_COUNT
    global ENABLE_LIDAR
    global ENABLE_FRONT

    if not ENABLE_LIDAR:
        return

    set_lidar_values(lidar_data)

    if CURRENT_STATE is "construction":
        if not TURTLE.is_settable():
            return

        rospy.loginfo(str(STATE_CONSTRUCTION) + "    " + str(get_object_distance('left')) + "   " + str(get_object_distance('front')))

        if STATE_CONSTRUCTION == "searching":
            distance = get_object_distance("left")
            if distance == 0:
                return
            elif distance < 0.35:
                STATE_CONSTRUCTION = "ready"
            elif distance > 0.35 and distance < 1:
                STATE_CONSTRUCTION = "start"
                DIRECTION = "right"
                TURTLE.enable_fish = False
                TURTLE.turn("left", 2.03, True)
            return

        elif STATE_CONSTRUCTION == "ready":
            distance = get_object_distance("front")
            if distance == 0:
                return
            elif distance < 0.5:
                # TURTLE.change_line("left", 0.5)
                DIRECTION = "left"
                TURTLE.enable_fish = False
                TURTLE.change_line("left", 1)
                # TODO: FIX change_line()
                STATE_CONSTRUCTION = "start"
            return

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
                    # TURTLE.change_line(DIRECTION, 1)
                    reverse_direction()
                    TURN_COUNT = TURN_COUNT + 1
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
                    TURN_COUNT = TURN_COUNT + 1
                    TURTLE.turn(DIRECTION, 1.2)
                    IS_TURNING = False

            if TURN_COUNT == 3:
                TURTLE.turn("left", 1.5, True)
                STATE_CONSTRUCTION = "ending"

        elif STATE_CONSTRUCTION == "ending":
                TURTLE.go_forward(1.0)
                CURRENT_STATE = "before_parking"
                print_state()
                # rospy.signal_shutdown("\n[PROC] Shutting down...")

    if CURRENT_STATE is "blocking":
        ENABLE_FRONT = False
        process_blocking()
    if CURRENT_STATE is "tunnel":
        process_tunnel2()

def process_tunnel():
    global DIRECTION

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

def process_tunnel2():
    global DIRECTION
    global IS_TURNING
    global IS_INSIDE_TUNNEL

    TURTLE.set_speed_smooth("fast")
    directions = ["front", "leftside", "rightside"]
    distances = {}
    for direction in directions:
        distances[direction] = get_object_distance(direction)

    if not IS_INSIDE_TUNNEL:
        distance = get_object_distance("right")
        if (distance < 0.3 and distance < 0):
            IS_INSIDE_TUNNEL = True
            TURTLE.enable_fish = False
    elif not IS_TURNING:
        rospy.loginfo("\n[PROC] tunnel: " + distances["leftside"] + str("   ") + distances["rightside"])
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

def go_anyway(event=None):
    print("excuted go_anyway function :(")
    TURTLE.enable_fish = True
    CURRENT_STATE = "tunnel"

def process_blocking():
    directions = ["front", "leftside", "rightside"]
    distances = {}
    rospy.loginfo("-------------")
    for direction in directions:
        distances[direction] = get_object_distance(direction)
        rospy.loginfo("\n[PROC] " + str(direction) + ": " + distances[direction])
    rospy.loginfo("-------------")

    if distances["rightside"] < 0.3 and distances["rightside"] > 0:
        TURTLE.enable_fish = False
        TURTLE.go_forward(2)
        SEEN_BLOCKING_BAR = True
    elif SEEN_BLOCKING_BAR:
        if distances["front"] > 0 or distances["leftside"] > 0 or distances["rightside"] > 0:
            TURTLE.enable_fish = True
            CURRENT_STATE = "tunnel"
        print_state()
    else:
        rospy.Timer(rospy.Duration(5.0), go_anyway, oneshot=True, reset=True)
    # rospy.loginfo("\n[PROC] front: " + str(distance_1) + "  " + str(distance_2) + "  " + str(distance_3))

def process_subcam(image):
    """ process the subcam image """
    start = timeit.default_timer()

    global LINE_BASE
    global TRACKING
    global TURNING_TO
    global TEST_ANGULAR

    if TRACKING is not "fish":
        return

    if not SCHEDULER.is_subcam_occupied():
    # if True:
        rospy.Timer(rospy.Duration(0.04), SCHEDULER.release_subcam_occupied, oneshot=True)
        info = EYE.see_sub(image)
        # rospy.Timer(rospy.Duration(0.04), SCHEDULER.release_subcam_occupied, oneshot=True, reset=True)

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

        # SCHEDULER.release_subcam_occupied()
        end = timeit.default_timer()
        # print("sub l: {:d}".format(info["left"]) + "   s: {:.01f}".format(slope)
        #     + "  time: {:.02f}".format(end - start))
        print("\nTIME: {:.02f}".format(end - start))

def process_eye(image):
    """ process the Eye System """
    global TRACKING
    global TURNING_TO

    # if TRACKING is not "front":
    #     return
    # rospy.loginfo('\n[PROC] frontcam image received')
    # rospy.loginfo(EYE.is_occupied())

    if not SCHEDULER.is_frontcam_occupied():
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

def print_state():
    print("#################################################")
    rospy.loginfo("State: " + str(CURRENT_STATE))
    print("#################################################")

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

def set_state_to_parking(event=None):
    global CURRENT_STATE
    CURRENT_STATE = "parking"
    print_state()

def set_package_state(msg):
    """ callback function to stop viewers """
    global GALAPAGOS_STATE

    rospy.loginfo("msg received: ", msg.data)
    GALAPAGOS_STATE = msg.data

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

def view_frontcam(image):
    """ view the Eye System """
    if GALAPAGOS_STATE is not "view":
        return
    # rospy.loginfo("\n[VIEWER] viewing frontcam...")
    if not SCHEDULER.is_frontcam_occupied():
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
    if not SCHEDULER.is_frontcam_occupied():
        info = EYE.see_sub(image)
        rospy.loginfo("\n[VIEW] info: \n" + str(info))
        rospy.Timer(rospy.Duration(0.1), SCHEDULER.release_subcam_occupied, oneshot=True, reset=True)

# * Initialization
initialize()
