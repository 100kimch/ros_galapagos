#! Deprecated

import rospy
from constants import *
from lib_signal_recognition import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from turtlebot import TURTLE
from lib_line_tracing import *

state = 'intersection'
inter_left_cnt = 0
inter_right_cnt = 0
# * Variables

K = np.array(ARRAY_K)
D = np.array(ARRAY_D)



def start_tracing(front_compressed):
    global state
    global inter_left_cnt
    global inter_right_cnt
    raw_data = np.fromstring(front_compressed.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)

    #### ROI SETTING ######
    blob_ROI = cv_img[100:, :]
    #######################
    #cv2.imshow("blob",blob_ROI)
    #cv2.waitKey(1)
    if state == 'traffic_light':
        tmp_state = is_light_green(blob_ROI)
        print(tmp_state)
        if tmp_state == True:
            # TURTLE.set_speed_by_percentage(0)
            # TURTLE.set_angular(0.)
            state = 'intersection'
        else:
            pass
    elif state == 'intersection':
        tmp_state = check_left_right_sign(blob_ROI)
        if tmp_state == 'right':
            #print("11tmp state: ",tmp_state, ", right cnt: ",inter_right_cnt)
            TURTLE.LINE_BASE = 2
            #print("11tmp state: ",tmp_state, ", right cnt: ",inter_right_cnt)
            state = 'right'
        elif tmp_state == 'left':
            #print("11tmp state: ",tmp_state, ", left cnt: ",inter_left_cnt)
            TURTLE.LINE_BASE = 1
            #print("22tmp state: ",tmp_state, ", left cnt: ",inter_left_cnt)
            state = 'left'
        elif tmp_state == 'none':
            print("no detected")
            pass
    else:
        #TURTLE.disable()
        print("=====================")
        print("present state: ", state)
        print("=====================")
    #TURTLE.move()

rospy.Subscriber(PATH_RASPICAM,CompressedImage, trace_line_by_base,  queue_size = 1) ## Used for Detecting Sinho,Jucha
rospy.Subscriber(PATH_USBCAM,CompressedImage, start_tracing,  queue_size = 1) ## Used for Detecting Sinho,Jucha
