#!/usr/bin/env python3
# ! Deprecated

from lib_line_tracing import *
from lib_signal_recognition import *
from lib_lidar import *
from turtlebot import TURTLE

# * Variables
CURRENT_STATE = 'traffic_light'
MOVING_POSITION = False
SEEN_PARKING_SIGN = False
SEEN_TUNNEL_SIGN = False
IS_IN_TUNNEL = False

HAS_OBJECT_IN_50 = False
HAS_OBJECT_IN_20 = False

straight_cnt = 0
curving_cnt = 0
# * Methods

def reset_front_image_flags():
    global MOVING_POSITION
    global SEEN_PARKING_SIGN
    global SEEN_TUNNEL_SIGN
    global HAS_OBJECT_IN_FRONT
    MOVING_POSITION = False
    SEEN_PARKING_SIGN = False
    SEEN_TUNNEL_SIGN = False
    SEEN_STOPPING_SIGN = False


def process_front_image(image):
    """ process the image of raspicam """
    global CURRENT_STATE
    global MOVING_POSITION
    global SEEN_PARKING_SIGN
    global SEEN_TUNNEL_SIGN

    raw_data = np.fromstring(image.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)

    #### ROI SETTING ######
    blob_ROI = cv_img[100:, :]
    #######################
    if CURRENT_STATE == 'traffic_light':
        if is_light_green(cv_img):
            TURTLE.enable()
            #TURTLE.set_speed('fast')
            CURRENT_STATE = 'intersection'
            return
        else:
            return

    if CURRENT_STATE == 'intersection':
        if is_intersection(cv_img):
            TURTLE.set_weight(0.7)
            CURRENT_STATE = 'left_or_right'
            return
        else:
            return


    if CURRENT_STATE == 'left_or_right':
        tmp_state = check_left_right_sign(blob_ROI)
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
        global straight_cnt
        if abs(TURTLE.weight*TURTLE._angular) < 0.1:
            straight_cnt += 1
            if straight_cnt > 3 :
                straight_cnt = 0
                TURTLE.LINE_BASE = 3
                CURRENT_STATE = 'construct_recog'
                return
            else:
                return
        else:
            return

    if CURRENT_STATE == 'construct_recog':
        tmp_state = is_construction(blob_ROI)
        if tmp_state is True:
            TURTLE.LINE_BASE = 2
            CURRENT_STATE = 'construction_ready'
        else:
            return

    if CURRENT_STATE == 'construction_ready':
        global curving_cnt
        if abs(TURTLE.weight*TURTLE._angular) > 0.25 :
            curving_cnt += 1
            if curving_cnt > 5 :
                curving_cnt = 0
                CURRENT_STATE = 'construction'
                TURTLE.enable_fish = False
            else:
                return
        else:
            return

    if CURRENT_STATE == 'construction':
        '''
        task for Ji-hyung
        '''
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
                    

            
    if CURRENT_STATE == 'construction':
        TURTLE.set_speed("slow")
        if HAS_OBJECT_IN_20:
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


def process_fish_image(image):
    """ process the fisheye lens image """

    trace_line(image)
    if CURRENT_STATE == 'intersection':
        if sign_intersection == 'left':
            TURTLE.set_speed_by_percentage(0.5)
            TURTLE.set_angular(TURTLE._angular + 0.2)
        elif sign_intersection == 'right':
            TURTLE.set_speed_by_percentage(0.5)
            TURTLE.set_angular(TURTLE._angular - 0.2)
    elif CURRENT_STATE == 'stop_sign':
        if left_detected > right_detected :
            TURTLE.set_speed_by_percentage(0.5)
            TURTLE.set_angular(TURTLE._angular + 0.2)
        elif right_detected < left_detected :
            TURTLE.set_speed_by_percentage(0.5)
            TURTLE.set_angular(TURTLE._angular - 0.2)
    TURTLE.move()
            

def process_usbcam_image(compressed_image):
    """ process the image of usb webcam """
    return


def process_lidar(lidar_data):
    """ process the lidar data """
    global HAS_OBJECT_IN_50
    global HAS_OBJECT_IN_20
    global IS_IN_TUNNEL

    HAS_OBJECT_IN_50 = has_object(lidar_data, 50)
    HAS_OBJECT_IN_20 = has_object(lidar_data, 20)
    IS_IN_TUNNEL = is_in_tunnel(lidar_data)


def test_line_tracing(image):
    trace_line(image)


# * Initialization
reset_front_image_flags()
