# ! 함수 이름 안바꿈, 변수 이름은 바꿈

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int8
from std_msgs.msg import Float32

from sensor_msgs.msg import CompressedImage

########## ORB Reference ############
Orb = cv2.ORB_create()
Bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

img_ref_color = cv2.imread('parking.jpg')
img_ref_gray = cv2.cvtColor(img_ref_color, cv2.COLOR_BGR2GRAY)

keypoint_img_ref = Orb.detect(img_ref_gray, None)
keypoint_img_ref, descriptor_img_ref = Orb.compute(
    img_ref_gray, keypoint_img_ref)

########## Initialization ###########
# HSV Ranges used in all variables

time = rospy.Duration(1.6)

lower_red_hsv = np.array([170, 160, 115])
upper_red_hsv = np.array([180, 220, 150])
lower_green_hsv = np.array([70, 75, 140])
upper_green_hsv = np.array([75, 180, 198])
lower_blue_hsv = np.array([97, 110, 90])
upper_blue_hsv = np.array([110, 240, 210])

# Sonar sensor initial values
dist_blocking = 100
dist_tunnel = 100

# Variables in setLight()
cnt_green = 1
cnt_red = 0

cnt_blue_sign = 0

# TODO: Change Variable Name
match_len = 0  # Constants in jucha
line_count = 0
park_count = 0
lt = 0

angular = 0  # initial angular vel

############### CONSTANT VALUES ###########
SPEED = 0.06
TRAFFIC_LIGHT = 0
PARKING = 1
BLOCKING = 2
TUNNEL = 3
NORMAL = 4

state = NORMAL

############### Functions #################
# Function that stop the turtlebot3 when node is shut down


def turtlestop():
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    Publisher_coord.publish(twist)

# Function that move the turtlebot


def turtlemove(linear, angular):
    rospy.on_shutdown(turtlestop)

    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    Publisher_coord.publish(twist)


def state_jucha(num):  # Function that save value used in jucha from 'in_jucha'
    global number
    number = num

# Function that save Sonar data used in chadan from sonar sensor


def chadan_dist(distance_blocking):
    global dist_blocking
    dist_blocking = distance_blocking.data

# Function that save Sonar data used in tunnel from sonar sensor


def tunnel_dist(distance_tunnel):
    global dist_tunnel
    dist_tunnel = distance_tunnel.data

# Function that run when state=0


def shinho(blob_ROI, state, angular):
    global cnt_red
    global cnt_green

    # if f_g == 1 and f_r == 0 and s_g == 0:
    if cnt_green == 1 and cnt_red == 0:
        keypoints_red = turtle_video_siljun.find_color(
            blob_ROI, lower_red_hsv, upper_red_hsv, state)
        print('first green signal detected.')

        if keypoints_red:
            # f_r = 1
            cnt_red += 1
        else:
            turtlemove(SPEED, angular)
        return 0

    # if f_g == 1 and f_r == 1 and s_g == 0:
    if cnt_green == 1 and cnt_red == 1:
        keypoints_green = turtle_video_siljun.find_color(
            blob_ROI, lower_green_hsv, upper_green_hsv, state)
        print('red signal detected. waiting secondary green signal.')
        turtlemove(0, 0)

        if keypoints_green:
            cnt_green += 1
        return 0

    # if f_g == 1 and f_r == 1 and s_g == 1:
    if cnt_green >= 2 and cnt_red == 1:
        print('second green signal detected.')
        turtlemove(SPEED, angular)
        # s_g = 2
        cnt_green += 1
        return NORMAL

# Function that run when state=1


def jucha(num, angular):
    global line_count
    global park_count
    global lt
    print(line_count)

    if line_count == 0:
        if num[0] == 0:
            turtlemove(SPEED, angular)
            return PARKING
        else:
            line_count = 1
            return PARKING

    elif line_count == 1:
        if num[0] == 1 and lt == 0:
            turtlemove(SPEED, angular)
            return PARKING
        elif num[0] == 0 and lt == 0:
            if num[1] == 1:
                turtlemove(0.11, -0.7)
                rospy.sleep(rospy.Duration(2))
                turtlemove(0.1, 0)
                rospy.sleep(rospy.Duration(1.7))
                turtlemove(0, 0)
                rospy.sleep(time)
                turtlemove(-0.1, 0)
                rospy.sleep(rospy.Duration(1.7))
                turtlemove(-0.11, 0.7)
                rospy.sleep(rospy.Duration(2))
                turtlemove(0, 0)
                park_count = 1
                lt = 1
                return PARKING
            else:
                lt = 1
                return PARKING
        else:
            turtlemove(SPEED, angular)
            if num[0] == 1:
                line_count = 2
                return PARKING
            return PARKING

    elif line_count == 2:
        if num[0] == 1 and lt == 1:
            turtlemove(SPEED, angular)
            return PARKING
        elif num[0] == 0 and park_count == 0 and lt == 1:
            turtlemove(0.11, -0.7)
            rospy.sleep(rospy.Duration(2))
            turtlemove(0.1, 0)
            rospy.sleep(rospy.Duration(2))
            turtlemove(0, 0)
            rospy.sleep(time)
            turtlemove(-0.1, 0)
            rospy.sleep(rospy.Duration(2))
            turtlemove(-0.11, 0.7)
            rospy.sleep(rospy.Duration(2))
            turtlemove(0, 0)
            lt = 2
            return PARKING
        elif park_count == 1 and lt == 1:
            lt = 2
            return PARKING
        else:
            turtlemove(SPEED, angular)
            if num[0] == 1:
                line_count = 3
                return NORMAL
            return PARKING

# Function that run when state=2


def chadan(dist):
    print(dist)
    if dist < 15:
        turtlemove(0, 0)
        return BLOCKING
    else:
        rospy.sleep(rospy.Duration(2))
        return NORMAL

# Function that run when state=3


def tunnel(dist):
    print(dist)
    if dist < 15:
        return TUNNEL
    else:
        return NORMAL

# Function setting angular velocity


def angular_Selecting(ros_data):
    global angular
    np_arr = np.fromstring(ros_data.data, np.uint8)
    img_src = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    ####<<< Selecting angular velocity >>>####
    img_src, angular = turtle_video_siljun.line_trace(
        img_src, state, verbose=False)

    # <<< Show processed image >>>##############################e
    cv2.imshow('video', img_src)
    cv2.waitKey(1) & 0xFF


def Stage_Selecting(ros_data):
    # color picking tool : https://alloyui.com/examples/color-picker/hsv/

    ####<<< Direct conversion to CV2 >>>####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    img_src = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    ####<<< DEFINE_ROI >>>####
    blob_ROI = img_src
    parking_ROI = img_src

    ####<<< Make variance global >>>####
    global state
    global cnt_blue_sign
    global match_len

    ############################<<< state Selecting >>>############################

    # if s_g < 2 and state == NORMAL:
    if cnt_green < 3 and state == NORMAL:
        keypoints_green = turtle_video_siljun.find_color(
            blob_ROI, lower_green_hsv, upper_green_hsv, 0)
        if keypoints_green:
            # TODO Check: state = NORMAL
            state = TRAFFIC_LIGHT
            print('Shinho(Traffic Light)!')

    if line_count < 3 and state == NORMAL:
        keypoints_blue = turtle_video_siljun.find_color(
            parking_ROI, lower_blue_hsv, upper_blue_hsv, 1)
        if keypoints_blue:
            print("__blue__")
            match_len = turtle_video_siljun.parking_match(
                parking_ROI, orb, bf, descriptor_img_ref)
            if match_len >= 9:
                state = PARKING
                print('jucha(***king)!')

    if state == NORMAL:
        if dist_blocking < 15 and dist_tunnel > 15:
            state = BLOCKING
            print('chadan(Blocking)!')

        elif dist_tunnel < 15:
            state = TUNNEL
            print('tunnel!')

    #########################<<< Select Function depening on state >>##########################
    if state == TRAFFIC_LIGHT:
        state = shinho(blob_ROI, state, angular)
    elif state == PARKING and number:
        state = jucha(number.data, angular)

    elif state == BLOCKING:
        state = chadan(dist_blocking)

    elif state == TUNNEL:
        state = tunnel(dist_tunnel)

    else:
        print("**********normal**************")
        turtlemove(0.09, angular)

    #########################<<< Show processed image >>>##############################
    cv2.imshow('video', img_src)
    cv2.waitKey(1) & 0xFF

    ########################<<< Publisher_coord state to other node >>>############################
    Publisher_state.publish(state)


############## Main Codes #################
rospy.Subscriber('/camera/image/compressed', CompressedImage,
                 Stage_Selecting,  queue_size=1)
rospy.Subscriber('/camera/image/compressed_fisheye',
                 CompressedImage, angular_Selecting,  queue_size=1)
rospy.Subscriber('/state', Int8MultiArray, state_jucha)
rospy.Subscriber('/sonar_dist_pub_1', Float32, chadan_dist)
rospy.Subscriber('/sonar_dist_pub_2', Float32, tunnel_dist)

Publisher_coord = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
Publisher_state = rospy.Publisher('/state', Int8, queue_size=5)


rospy.init_node('video_processor', anonymous=True)

rospy.spin()
