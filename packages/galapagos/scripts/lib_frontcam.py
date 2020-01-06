import cv2
import numpy as np
import matplotlib.pyplot as plt
import sys
from constants import *
import os
import sys

############################################################## Variables ##############################################################
# variables for matching
# Initiate SIFT description detector
Orb = cv2.ORB_create()
# create BFMatcher object
Bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# reference images
ref_parking = cv2.imread(os.path.dirname(os.path.abspath(__file__)) + '/parking.jpg', cv2.IMREAD_GRAYSCALE)
# print("path: ", os.getcwd())
print("ref parking shape1: ",ref_parking.shape)
ref_left = cv2.imread(os.path.dirname(os.path.abspath(__file__))+'/left.png', cv2.IMREAD_GRAYSCALE)
ref_right = cv2.imread(os.path.dirname(os.path.abspath(__file__))+'/right.png', cv2.IMREAD_GRAYSCALE)
ref_intersection = cv2.imread(os.path.dirname(os.path.abspath(__file__))+ '/T_2.png', cv2.IMREAD_GRAYSCALE)
ref_construction = cv2.imread(os.path.dirname(os.path.abspath(__file__))+ '/construction.jpg', cv2.IMREAD_GRAYSCALE)
ref_turnel = cv2.imread(os.path.dirname(os.path.abspath(__file__))+ '/turnel.png', cv2.IMREAD_GRAYSCALE)

# reference keypoints & descriptor
just_init = 1
ref_parking_keypoints = just_init
ref_parking_descriptor = just_init
ref_left_keypoints = just_init
ref_left_descriptor = just_init
ref_right_keypoints = just_init
ref_right_descriptor = just_init
ref_intersection_keypoints = just_init
ref_intersection_descriptor = just_init
ref_construction_keypoints = just_init
ref_construction_descriptor = just_init
ref_turnel_keypoints = just_init
ref_turnel_descriptor = just_init

# threshold values
threshold_parking = THRESHOLDS["parking"]
threshold_lrmin = THRESHOLDS["left_right_min"]
threshold_lrmax = THRESHOLDS["left_right_max"]
threshold_intersection = THRESHOLDS["intersection"]
threshold_turnel = THRESHOLDS["turnel"]
threshold_construction = THRESHOLDS["construction"]

# HSV Ranges used in each state
lower_blue = np.array([90,80,0])
upper_blue = np.array([130,255,255])
#lower_blue = np.array([93, 40, 60])
#upper_blue = np.array([115, 140, 140])
lower_red = np.array([0, 100, 100])
upper_red = np.array([20, 255, 255])
lower_green=np.array([57,34,70])
upper_green=np.array([72,255,255])

############################################################## Functions ##############################################################

def refimage_init():
    global ref_parking; global ref_left; global ref_right; global ref_intersection; global ref_construction; global ref_turnel
    global ref_parking_keypoints; global ref_left_keypoints; global ref_right_keypoints; global ref_intersection_keypoints; global ref_construction_keypoints; global ref_turnel_keypoints
    global ref_parking_descriptor; global ref_left_descriptor; global ref_right_descriptor; global ref_intersection_descriptor; global ref_construction_descriptor; global ref_turnel_descriptor

    ref_parking = cv2.GaussianBlur(ref_parking,(11, 11), 0)
    ref_parking = cv2.medianBlur(ref_parking, 11)
    ref_parking_keypoints, ref_parking_descriptor = Orb.detectAndCompute(
        ref_parking, None)

    ref_left = cv2.GaussianBlur(ref_left, (11, 11), 0)
    ref_left = cv2.medianBlur(ref_left, 11)
    ref_left_keypoints, ref_left_descriptor = Orb.detectAndCompute(
        ref_left, None)

    ref_right = cv2.GaussianBlur(ref_right, (11, 11), 0)
    ref_right = cv2.medianBlur(ref_right, 11)
    ref_right_keypoints, ref_right_descriptor = Orb.detectAndCompute(
        ref_right, None)

    ref_intersection = cv2.GaussianBlur(ref_intersection, (11, 11), 0)
    ref_intersection = cv2.medianBlur(ref_intersection, 11)
    ref_intersection_keypoints, ref_intersection_descriptor = Orb.detectAndCompute(
        ref_intersection, None)

    ref_construction = cv2.GaussianBlur(ref_construction, (11, 11), 0)
    ref_construction = cv2.medianBlur(ref_construction, 11)
    ref_construction_keypoints, ref_construction_descriptor = Orb.detectAndCompute(
        ref_construction, None)

    ref_turnel = cv2.GaussianBlur(ref_turnel, (11, 11), 0)
    ref_turnel = cv2.medianBlur(ref_turnel, 11)
    ref_turnel_keypoints, ref_turnel_descriptor = Orb.detectAndCompute(
        ref_turnel, None)



def blob_parameter(_recog_type):
    ''' blob_parameter function for making detector for some blob shapes(circle or triangle)
            & setting parameter of detector
        * Input 
            _recog_type : recognition type in recognition_list (ex : 'parking')
        * Output
            blob_detector : blob detector that has parameters setted by recognition type 
    '''
    if _recog_type == 'traffic_light':
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 256

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 100
        params.maxArea = 2300

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.1

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01
    
    elif _recog_type == 'intersection' or _recog_type == 'construction' or _recog_type == 'turnel':
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

    else:
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 256

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 4000
        params.maxArea = 35000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.7

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.1

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')

    if int(ver[0]) < 3:
        blob_detector = cv2.SimpleBlobDetector(params)
    else:
        blob_detector = cv2.SimpleBlobDetector_create(params)

    return blob_detector


def blob_detecting(_input_img, _blob_detector, _recog_type):
    ''' blob_detecting function for finding center point of ROI
        by HSV range thresholding & detecting specific blob shape
        * Input
            _input_img : front camera image from pi camera --> BGR image
            _blob_detector : HSV blob detector made by blob_parameter() function
        * Output
            blob_centers : center points of blob (specific blob shape -> potential ROI center point)
            centers : if blob_centers is detected, change the type of data to pt array
    '''


    _hsv_maxThreshold = just_init
    _hsv_minThreshold = just_init
    if _recog_type=='intersection' or _recog_type == 'construction' or _recog_type == 'turnel':
        _hsv_maxThreshold = upper_red
        _hsv_minThreshold = lower_red
    elif _recog_type=='traffic_light':
        _hsv_maxThreshold = upper_green
        _hsv_minThreshold = lower_green
    else:
        _hsv_maxThreshold = upper_blue
        _hsv_minThreshold = lower_blue

    # thresholding process rgb_image to hsv_image by HSV Threshold
    _input_img = cv2.GaussianBlur(_input_img, (7, 7), 0)
    hsv = cv2.cvtColor(_input_img,cv2.COLOR_BGR2HSV)
    
    # make mask and pre-image-processing : morphology (erode or dilate)
    kernel = np.ones((3, 3), np.uint8)
    kernel2 = np.ones((5, 5), np.uint8)
    mask = cv2.inRange(hsv, _hsv_minThreshold, _hsv_maxThreshold)
    if _recog_type == 'traffic_light':
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel2)
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel2)
        mask = cv2.dilate(mask, kernel, iterations=10)
    else:
        mask = cv2.dilate(mask, kernel, iterations=5)

    #maks = cv2.erode(mask, kernel, iterations = 3)
    reversemask = 255-mask

    # Detect specific blobshape and center point of blob
    if _recog_type=='intersection' or _recog_type == 'construction' or _recog_type == 'turnel':
        blob_centers = _blob_detector.detect(mask)
    else:
        blob_centers = _blob_detector.detect(reversemask)

    BGR_ROI = cv2.cvtColor(reversemask, cv2.COLOR_GRAY2BGR)
    
    IS_DEBUG_MODE = True
    if IS_DEBUG_MODE == True:
        print(len(blob_centers))
        show_centers = cv2.drawKeypoints(reversemask, blob_centers, np.array(
            []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)  # ! code for debugging
        cv2.imshow('org',_input_img)
        cv2.imshow('hsv',hsv)                                                                                                          #! code for debugging
        cv2.imshow('mask',mask)                                                                                                        #! code for debugging
        cv2.imshow('reverse',reversemask)                                                                                               #! code for debugging
        cv2.imshow('result', show_centers)  # ! code for debugging
        cv2.waitKey(1)  # ! code for debugging

    if len(blob_centers) >= 1:
        centers = []
        for i in blob_centers:
            centers.append(i.pt)
        return centers
    else:
        return blob_centers


def signROI_detecting(_input_img, _recog_type):
    ''' signROI_detecting function for detecting signROI
        using HSV range thresholding and specific blob shape detectiong, different by cases
        * Input 
            _input_img : front camera image from pi camera --> BGR image
            _recog_type : recognition type 
        * Output
            signROI : sign detection result image --> BGR image
            True : thiere is singROI
            False : there is no signROI
    '''

    # _input_img ROI detecting using HSV range
    #_input_img = cv2.GaussianBlur(_input_img, (5, 5), 0)

    sign_detector = blob_parameter(_recog_type)   
    sign_centers = blob_detecting(_input_img, sign_detector, _recog_type)

    # cv2.imshow('input',_input_img)                                                           #! code for debugging
    # cv2.waitKey()                                                                            #! code for debugging
    # print 'test'                                                                             #! code for debugging
    # print len(sign_centers)                                                                  #! code for debugging

    if len(sign_centers) >= 1:
        xx = int(sign_centers[0][0])
        yy = int(sign_centers[0][1])
        # print sign_centers[0][1], sign_centers[0][0]                                         #! code for debugging

        if sign_centers[0][1]-70 < 0 or sign_centers[0][0] < 70:
            if sign_centers[0][0] < sign_centers[0][1]:
                signROI_size = int(sign_centers[0][0])
            else:
                signROI_size = int(sign_centers[0][1])
        else:
            signROI_size = 70

        signROI = _input_img[yy - signROI_size: yy +
                             signROI_size, xx - signROI_size: xx + signROI_size]
        # cv2.imshow('ROI',signROI)                                                            #! code for debugging
        # cv2.waitKey()                                                                        #! code for debugging
        return signROI, True
    else:
        signROI = _input_img
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

    if IS_DEBUG_MODE == True:
        print(len(matches))                                                                                       #! code for debugging
        matching_image = cv2.drawMatches(
            _roi, ROI_keypoints, _ref_img, _ref_keypoints, matches, None, flags=2)  # ! code for debugging
        cv2.imshow('matching',matching_image)                                                                          #! code for debugging
        cv2.waitKey()                                                                                                  #! code for debugging

    return matches


def left_or_right(_frontcam_roi):
    ''' left_or_right function for check direction of arrow sign
        * Input
            _frontcam_roi : sign roi image from front pi camera image --> gray image
        * Output
            direction left or right
    '''

    _frontcam_roi = cv2.cvtColor(_frontcam_roi, cv2.COLOR_BGR2GRAY)
    # Threshloding --> binary image by otsu's method
    #cv2.imshow('gray_frontcam_roi',_frontcam_roi)
    _frontcam_roi = cv2.GaussianBlur(_frontcam_roi, (7, 7), 0)
    #cv2.imshow('gaussian_gray_frontcam_roi',_frontcam_roi) 
    tmp, binary_roi = cv2.threshold(
        _frontcam_roi, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
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
    for j in range(bin_roi_H//2):
        for i in range(bin_roi_W//2):
            sum_left += binary_roi[j + bin_roi_H//2, i]
        for i in range(bin_roi_W//2):
            sum_right += binary_roi[j + bin_roi_H//2, i+binary_roi.shape[1]//2]    
    print("=========================================")
    print("sum left: ",sum_left/255,", sum right: ", sum_right/255)                                                          #! code for debugging
    print("==============sum printing================")
    #cv2.imshow('binary_roi',binary_roi)                                                         #! code for debugging
    #cv2.imshow('small_roi',small_roi)                                                           #! code for debugging
    #cv2.waitKey(1)                                                                               #! code for debugging

    if sum_left > sum_right:
        return 'right'
    else:
        return 'left'


def is_light_green(image):
    """ check the traffic_light's light
    image: front image
    return True if light is green
    return False if light is not green
    """

    detector = blob_parameter('traffic_light')
    centers = blob_detecting(image, detector, 'traffic_light')  

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
    cv2.imshow("inter_ROI",ROI_img)
    if ROI_OX != False:
        # matching & thresholding
        result_matching = ORB_matching(ROI_img, ref_intersection,ref_intersection_keypoints,ref_intersection_descriptor)
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
            ROI_img, ref_parking, ref_parking_keypoints, ref_parking_descriptor)

        if len(result_matching) >= threshold_parking:
            return True
        else:
            return False

    else:
        return False

    #TODO : is it needed?
    return False / True

def is_construction(image):
    ROI_img, ROI_OX = signROI_detecting(image, 'construction')

    if ROI_OX != False:
        # matching & thresholding
        result_matching = ORB_matching(
            ROI_img, ref_construction, ref_construction_keypoints, ref_construction_descriptor)
        
        IS_DEBUG_MODE = True
        if IS_DEBUG_MODE == True:
            print("result matching : " + str(len(result_matching)))

        if len(result_matching) >= threshold_construction:
            return True
        else:
            return False

    else:
        return False

def is_turnel(image):
    ROI_img, ROI_OX = signROI_detecting(image, 'turnel')

    if ROI_OX != False:
        # matching & thresholding
        result_matching = ORB_matching(
            ROI_img, ref_turnel, ref_turnel_keypoints, ref_turnel_descriptor)

        if IS_DEBUG_MODE == True:
            print("result matching : " + len(result_matching))
        
        if len(result_matching) >= threshold_turnel:
            return True
        else:
            return False

    else:
        return False


def check_left_right_sign(_frontcam):
    """ check the sign of left or right sign
    return 'left' if the sign means 'left'
    return 'right' if the sign means 'right'
    return 'none' if there isn't a sign
    """
    global threshold_lrmin
    global threshold_lrmax
    global ref_left
    global ref_left_keypoints
    global ref_left_descriptor
    global ref_right 
    global ref_right_keypoints
    global ref_right_descriptor

    ROI_img, ROI_OX = signROI_detecting(_frontcam, 'left_or_right')

    if ROI_OX != False:
        # LEFT or RIGHT
        result_matching_left = ORB_matching(
            ROI_img, ref_left, ref_left_keypoints, ref_left_descriptor)
        result_matching_right = ORB_matching(
            ROI_img, ref_right, ref_right_keypoints, ref_right_descriptor)
        
        #print("left length: ",len(result_matching_left))
        #print("right length: ",len(result_matching_right))


        if len(result_matching_left) >= threshold_lrmin and len(result_matching_left) <= threshold_lrmax and len(result_matching_right) >= threshold_lrmin and len(result_matching_right) <= threshold_lrmax:
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
refimage_init()
