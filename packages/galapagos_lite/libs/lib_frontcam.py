import cv2
import numpy as np
import rospy
import matplotlib.pyplot as plt
import sys
from scheduler import SCHEDULER
from constants import *
import os
import sys


# * Variables

# variables for matching
# Initiate SIFT description detector
Orb = cv2.ORB_create()
# create BFMatcher object
Bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

ref_images = {}

# HSV Ranges used in each state
""" before 
lower_blue = np.array([90, 80, 0])
upper_blue = np.array([130, 255, 255])
"""

lower_blue = np.array([90, 110, 100])
upper_blue = np.array([130, 200, 160])
lower_red = np.array([0, 120, 100])
upper_red = np.array([20, 180, 200])
lower_green = np.array([60, 55, 50])
upper_green = np.array([85, 255, 255])
# lower_green = np.array([65, 60, 60])
# upper_green = np.array([80, 255, 255])

value_default = 1

# * Methods


def init_ref_images():
    global ref_images

    for idx, key in enumerate(REF_IMAGE_PATH):
        image = cv2.imread(SCHEDULER.path + "/images/" + REF_IMAGE_PATH[key])
        try:
            image = cv2.medianBlur(cv2.GaussianBlur(image, (11, 11), 0), 11)
        except cv2.error as e:
            rospy.logfatal("[LIB_FRONT] ref image '" + key +
                           "' not found in constants.py!")
            rospy.signal_shutdown("Shutdown by fatal error.")

        if SCHEDULER.debug_option["show_loaded_ref_images"]:
            rospy.logdebug("[LIB_FRONT] {:8s} image loaded. ({})".format(
                key, str(SCHEDULER.path + "/" + REF_IMAGE_PATH[key])))

        keypoints, descriptor = Orb.detectAndCompute(image, None)

        ref_images[key] = {
            "image": image,
            "keypoints": keypoints,
            "descriptor": descriptor
        }


def blob_parameter(state_type):
    ''' blob_parameter function for making detector for some blob shapes(circle or triangle)
            & setting parameter of detector
        * Input 
            state_type : recognition type in recognition_list (ex : 'parking')
        * Output
            blob_detector : blob detector that has parameters setted by recognition type 
    '''
    if state_type == 'traffic_light':
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 0
        params.maxThreshold = 256
        params.filterByArea = True
        params.minArea = 500
        params.maxArea = 2300
        params.filterByCircularity = True
        params.minCircularity = 0.4
        params.filterByConvexity = True
        params.minConvexity = 0.1
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

    elif state_type == 'intersection' or state_type == 'construction' or state_type == 'turnel':
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 10
        params.maxThreshold = 200
        params.filterByArea = True
        params.minArea = 500
        params.filterByCircularity = True
        params.minCircularity = 0.1
        params.filterByConvexity = False
        params.minConvexity = 0.1
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

    else:
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 0
        params.maxThreshold = 256
        params.filterByArea = True
        params.minArea = 1000
        params.maxArea = 35000
        params.filterByCircularity = True
        params.minCircularity = 0.5
        params.filterByConvexity = True
        params.minConvexity = 0.1
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')

    if int(ver[0]) < 3:
        blob_detector = cv2.SimpleBlobDetector(params)
    else:
        blob_detector = cv2.SimpleBlobDetector_create(params)

    return blob_detector


def blob_detecting(image, blob_detector, state_type):
    ''' blob_detecting function for finding center point of ROI
        by HSV range thresholding & detecting specific blob shape
        * Input
            image : front camera image from pi camera --> BGR image
            blob_detector : HSV blob detector made by blob_parameter() function
        * Output
            blob_centers : center points of blob (specific blob shape -> potential ROI center point)
            centers : if `blob_centers` is detected, change the type of data to pt array
    '''
    #cv2.imshow("raw_image", image)
    # cv2.waitKey(1)
    _hsv_maxThreshold = value_default
    _hsv_minThreshold = value_default
    if state_type == 'intersection' or state_type == 'construction' or state_type == 'turnel':
        _hsv_maxThreshold = upper_red
        _hsv_minThreshold = lower_red
    elif state_type == 'traffic_light':
        _hsv_maxThreshold = upper_green
        _hsv_minThreshold = lower_green
    else:
        _hsv_maxThreshold = upper_blue
        _hsv_minThreshold = lower_blue

    # thresholding process rgb_image to hsv_image by HSV Threshold
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # make mask and pre-image-processing : morphology (erode or dilate)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.inRange(hsv, _hsv_minThreshold, _hsv_maxThreshold)
    mask = cv2.dilate(mask, kernel, iterations=5)
    #maks = cv2.erode(mask, kernel, iterations = 3)
    reversemask = 255 - mask

    # Detect specific blobshape and center point of blob
    if state_type == 'intersection' or state_type == 'turnel' or state_type == 'to_intersection':
        blob_centers = blob_detector.detect(mask)
    else:
        blob_centers = blob_detector.detect(reversemask)

    BGR_ROI = cv2.cvtColor(reversemask, cv2.COLOR_GRAY2BGR)

    # if IS_DEBUG_MODE == True:

    if SCHEDULER.debug_option["show_blob_detecting"]:
        print(len(blob_centers))
        show_centers = cv2.drawKeypoints(reversemask, blob_centers, np.array(
            []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow('hsv', hsv)
        cv2.imshow('mask', mask)
        cv2.imshow('reverse', reversemask)
        cv2.imshow('result', show_centers)
        cv2.waitKey(0)

    if len(blob_centers) >= 1:
        centers = []
        for i in blob_centers:
            centers.append(i.pt)
        return centers
    else:
        return blob_centers


def signROI_detecting(image, state_type):
    ''' signROI_detecting function for detecting signROI
        using HSV range thresholding and specific blob shape detectiong, different by cases
        * Input 
            image : front camera image from pi camera --> BGR image
            state_type : recognition type 
        * Output
            signROI : sign detection result image --> BGR image
            True : thiere is singROI
            False : there is no signROI
    '''
    # image ROI detecting using HSV range
    #image = cv2.GaussianBlur(image, (5, 5), 0)
    sign_detector = blob_parameter(state_type)
    sign_centers = blob_detecting(image, sign_detector, state_type)
    # cv2.imshow('input', image)  # ! code for debugging
    # cv2.waitKey()  # ! code for debugging
    # print 'test'                                                                             #! code for debugging
    # print len(sign_centers)                                                                  #! code for debugging
    if len(sign_centers) >= 1:
        xx = int(sign_centers[0][0])
        yy = int(sign_centers[0][1])
        # print sign_centers[0][1], sign_centers[0][0]                                         #! code for debugging
        if sign_centers[0][1] - 45 < 0 or sign_centers[0][0] < 45:
            if sign_centers[0][0] < sign_centers[0][1]:
                signROI_size = int(sign_centers[0][0])
            else:
                signROI_size = int(sign_centers[0][1])
        else:
            signROI_size = 45
        signROI = image[yy - signROI_size: yy +
                        signROI_size, xx - signROI_size: xx + signROI_size]
        if SCHEDULER.debug_option["show_roi_detecting"]:
            cv2.imshow('ROI', signROI)  # ! code for debugging
            cv2.waitKey(1)  # ! code for debugging
        # print("blob detected!!!!!")
        # print("blob detected!!!!!")
        # print("blob detected!!!!!")
        return signROI, True
    else:
        # print("blob detected failed!!!!")
        # print("blob detected failed!!!!")
        # print("blob detected failed!!!!")
        signROI = image
        return signROI, False


def ORB_matching(_roi, _ref_img, _ref_keypoints, _ref_descriptor):
    ''' ORB_matching function for matching two input image and output is matching result
        * Input
            _roi : sign ROI image --> BGR image
            _ref : sign ref image --> gray image
        * Output
            matches : ORB descriptor matching result     
    '''

    global Orb
    global Bf

    # image pretreatment
    _roi = cv2.cvtColor(_roi, cv2.COLOR_BGR2GRAY)
    _roi = cv2.medianBlur(_roi, 5)

    # find the keypoints and descriptors with SIFT
    ROI_keypoints, ROI_descriptor = Orb.detectAndCompute(_roi, None)

    # Match descriptors.
    matches = Bf.match(ROI_descriptor, _ref_descriptor)

    # Sort them in the order of their distance.
    # Not use distance values yet, but can use it at thresholding
    matches = sorted(matches, key=lambda x: x.distance)

    if SCHEDULER.debug_option["show_orb_matching"] == True:
        print(len(matches))  # ! code for debugging
        matching_image = cv2.drawMatches(
            _roi, ROI_keypoints, _ref_img, _ref_keypoints, matches, None, flags=2)  # ! code for debugging
        cv2.imshow('matching', matching_image)  # ! code for debugging
        cv2.waitKey()  # ! code for debugging

    return matches


def check_if_left_or_right(image):
    """ 
        Check direction where to go in intersection
        @Input: image - sign roi image from front pi camera image --> gray image
        @Output: direction where to go: ["left", "right]
    """


def left_or_right(_frontcam_roi):
    ''' left_or_right function for check direction of arrow sign
        * Input
            _frontcam_roi : sign roi image from front pi camera image --> gray image
        * Output
            direction left or right
    '''
    _frontcam_roi = cv2.cvtColor(_frontcam_roi, cv2.COLOR_BGR2GRAY)
    # Threshloding --> binary image by otsu's method
    # cv2.imwrite('gray_frontcam_roi.png', _frontcam_roi)
    _frontcam_roi = cv2.GaussianBlur(_frontcam_roi, (7, 7), 0)
    # cv2.imwrite('gaussian_gray_frontcam_roi.png', _frontcam_roi)
    tmp, binary_roi = cv2.threshold(
        _frontcam_roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    #height = np.size(binary_roi, 0)/2
    #width = np.size(binary_roi, 1)/2
    # cutting small ROI from under center point 40 X 20 box
    #small_roi = binary_roi[height:height+20, width-20:width+20]
    # compare sum of left pixels' value and right's one
    sum_left = 0
    sum_right = 0
    '''
    for j in range(20):
        for i in range(20):
            sum_left += small_roi[j, i]
        for i in range(20, 40):
            sum_right += small_roi[j, i]
    '''
    bin_roi_H = binary_roi.shape[0]
    bin_roi_W = binary_roi.shape[1]
    for j in range(bin_roi_H // 2):
        for i in range(bin_roi_W // 2):
            sum_left += binary_roi[j + bin_roi_H // 2, i]
        for i in range(bin_roi_W // 2):
            sum_right += binary_roi[j + bin_roi_H //
                                    2, i + binary_roi.shape[1] // 2]
    print("=========================================")
    print("sum left: ", sum_left / 255, ", sum right: ",
          sum_right / 255)  # ! code for debugging
    print("==============sum printing================")
    # cv2.imwrite('binary_roi.png', binary_roi)  # ! code for debugging
    # cv2.imshow('small_roi',small_roi)                                                           #! code for debugging
    # cv2.waitKey(0)                                                                               #! code for debugging
    if sum_left > sum_right:
        print("right detected!!!!")
        print("right detected!!!!")
        print("right detected!!!!")
        return 'right'
    else:
        print("left detected!!!!")
        print("left detected!!!!")
        print("left detected!!!!")
        return 'left'


def is_light_green(image):
    """ check the traffic_light's light
    image: front image
    return True if light is green
    return False if light is not green
    """
    raw_data = np.fromstring(image.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)

    detector = blob_parameter('traffic_light')
    centers = blob_detecting(cv_img, detector, 'traffic_light')

    if len(centers) >= 1:
        return True
    else:
        return False


def is_intersection(image):
    """ check whether the image is intersection
    return True if intersection
    return False if not intersection
    """

    ROI_img, ROI_OX = signROI_detecting(image, 'intersection')
    if ROI_OX != False:
        # matching & thresholding
        result_matching = ORB_matching(
            ROI_img, ref_intersection, ref_intersection_keypoints, ref_intersection_descriptor)
        if len(result_matching) >= threshold_intersection:
            return True
        else:
            return False
    else:
        return False


def is_parking(image):
    """ check whether the image is parking
    return True if parking
    return False if not parking
    """

    ROI_img, ROI_OX = signROI_detecting(image, 'parking')

    if ROI_OX != False:
        # matching & thresholding
        result_matching = ORB_matching(
            ROI_img, ref_images["parking"]["images"], ref_images["parking"]["keypoints"], ref_images["parking"]["descriptor"])

        if len(result_matching) >= threshold_parking:
            return True
        else:
            return False

    else:
        return False


def is_construction(image):
    raw_data = np.fromstring(image.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)

    ROI_img, ROI_OX = signROI_detecting(cv_img, 'construction')

    if ROI_OX != False:
        # matching & thresholding
        result_matching = ORB_matching(
            ROI_img, ref_images["construction"]["image"], ref_images["construction"]["image"], ref_images["construction"]["descriptor"])

        if SCHEDULER.debug_option["show_image_matching"]:
            rospy.logdebug("result matching : " + str(len(result_matching)))

        if len(result_matching) >= THRESHOLDS["construction"]:
            return True
        else:
            return False

    else:
        return False


def is_tunnel(image):
    ROI_img, ROI_OX = signROI_detecting(image, 'tunnel')

    if ROI_OX != False:
        # matching & thresholding
        result_matching = ORB_matching(
            ROI_img, ref_images["tunnel"], ref_images["tunnel"]["keypoints"], ref_images["tunnel"]["descriptor"])

        if SCHEDULER.debug_option["show_image_matching"] == True:
            rospy.logdebug("result matching : " + str(len(result_matching)))

        if len(result_matching) >= threshold_tunnel:
            return True
        else:
            return False

    else:
        return False


def check_left_right_sign(image):
    """ check the sign of left or right sign
    return 'left' if the sign means 'left'
    return 'right' if the sign means 'right'
    return 'none' if there isn't a sign
    """
    global threshold_lrmin
    global threshold_lrmax

    raw_data = np.fromstring(image.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)
    # NOTE: ROI Setting
    blob_ROI = cv_img[100:, :]

    ROI_img, ROI_OX = signROI_detecting(cv_img, 'left_or_right')

    if ROI_OX != False:
        # LEFT or RIGHT
        result_matching_left = ORB_matching(
            ROI_img, ref_images["left"]["image"], ref_images["left"]["keypoints"], ref_images["left"]["descriptor"])
        result_matching_right = ORB_matching(
            ROI_img, ref_images["right"]["image"], ref_images["right"]["keypoints"], ref_images["right"]["descriptor"])

        #print("left length: ",len(result_matching_left))
        #print("right length: ",len(result_matching_right))

        if len(result_matching_left) >= THRESHOLDS["left_right_min"] and \
                len(result_matching_left) <= THRESHOLDS["left_right_max"] and \
                len(result_matching_right) >= THRESHOLDS["left_right_min"] and \
                len(result_matching_right) <= THRESHOLDS["left_right_max"]:
            return left_or_right(ROI_img)
        else:
            return 'none'

    else:
        return 'none'


def check_sign(image):
    """ check what the sign means
    image: front image
    return 'intersection' if the sign means 'intersection' state
    return 'construction' if the sign means 'construction' state
    return 'parking' if the sign means 'parking' state
    return 'tunnel' if the sign means 'tunnel' state
    return 'nothing' if there is no sign
    """
    if(is_intersection(image) == True):
        return 'intersection'
    elif(is_parking(image) == True):
        return 'parking'
    else:
        return 'nothing'


def has_curve_in(distance, image):
    """ check if there is a curve in distance"""
    # TODO: future task
    return False


def is_straight_in(distance, image):
    """ check if the way is straight in distance"""
    # TODO: future task
    return False


def is_stopping_sign(image):
    """ check if the sign means 'stop """
    # TODO: future task
    return False


def has_crossing_line(image):
    """ returns true if there is a crossing line from left to right"""
    # TODO: future task
    return False


# reference image & keypoints & descriptors initialize
# init_ref_images()
'''
from lib_frontcam import signROI_detecting
import cv2
right_img = cv2.imread(
    "/home/kusw-004/steamcup_2020/assets/images_final/image1.png")
left_img = cv2.imread(
    "/home/kusw-004/steamcup_2020/assets/images_final/image2.png")
ref_left = cv2.imread(
    "/home/kusw-004/steamcup_2020/packages/galapagos_v2/libs/left.png")
ref_right = cv2.imread(
    "/home/kusw-004/steamcup_2020/packages/galapagos_v2/libs/right.png")
'''
