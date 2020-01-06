""" constants """
#!/usr/bin/env python3

PATH_RASPICAM = "/raspicam_node/image/compressed"
PATH_USBCAM = "/usb_cam/image_raw/compressed"
PATH_LIDAR = "/scan"
PATH_ASSETS = "../assets/"
PATH_GALAPAGOS_STATE = "/galapagos_state"
IS_DEBUG_MODE = False
SELECTED_STATE = ""


MAX_SPEED = 0.16
TURNING_SPEED = 0.13
TUNNEL_SPEED = 0.22
SPEED_VALUES = {
    'fast': 1,
    'normal': 0.7,
    'slow': 0.3,
    'stop': 0
}


#RUN_MODE = 'debug'  # ! Deprecated
RUN_MODE = 'nono'
RUN_TYPE = ''  # ! Deprecated

#############################################################
ARRAY_K = [[480.0, 0, 320.0], 
            [0.0, 320.0, 230.0], 
            [0.0, 0.0, 1.0]]
# [  [클수록 아래볼록 작을수록 위볼록] , [+ :오른쪽치우침, - :왼쪽치우침] , ]
ARRAY_D = [0.0, 0.0, 0.0, 0.0]
#### modified by minsoo -190810 ##############################

SIGNAL_IMAGES = {
    "parking": "parking.jpg",
    "left": "left.png",
    "right": "right.png",
    "three": "T_2.png",
}

# threshold values
THRESHOLD_PARKING = 26
# threshold values
THRESHOLD_LEFT_RIGHT_MIN = 8
# threshold values
THRESHOLD_LEFT_RIGHT_MAX = 17
# threshold values
THRESHOLD_INTERSECTION = 15

DEFAULT_LIDAR_DEGREE = 4

# NOTE: New Contstants
THRESHOLDS = {
    "parking": 26,
    "left_right_min": 8,
    "left_right_max": 17,
    "intersection": 8,
    "turnel": 10,
    "construction": 30
}

LIDAR_DIRECTIONS = {
    'front': 0,
    'leftside': 10,
    'frontleft': 45,
    'left': 90,
    'back': 180,
    'right': 270,
    'frontright': 315,
    'rightside': 350
}
