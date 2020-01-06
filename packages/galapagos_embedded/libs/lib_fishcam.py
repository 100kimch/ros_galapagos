#!/usr/bin/env python3
#!Deprecated
""" this module run in server """
import rospy
import cv2
import numpy as np
from constants import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from turtlebot import TURTLE

# * Variables

K = np.array(ARRAY_K)
D = np.array(ARRAY_D)

# * Functions
angular = 0.0
both_calcul_flag = False
one_calcul_flag = False
def trace_line(compressed_data):
    global angular
    global both_calcul_flag
    if both_calcul_flag is True:
        return
    both_calcul_flag = True
    ##########################################################################
    _DEBUG = True
    """ view image """
    # print("viewing image.")
    # rospy.loginfo("img_viewer()")
    raw_data = np.fromstring(compressed_data.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)

    Knew = K.copy()
    Knew[(0, 1), (0, 1)] = 0.8 * Knew[(0, 1), (0, 1)]
    img = cv2.fisheye.undistortImage(cv_img, K, D=D, Knew=Knew)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv2.line(gray,(200,100),(440,100),(253,244,8),2)        ## top row
    #cv2.line(gray,(200,300),(440,300),(253,244,8),2)        ## bottom row
    #cv2.line(gray,(200,100),(200,300),(253,244,8),2)        ## left col
    #cv2.line(gray,(440,100),(440,300),(253,244,8),2)        ## right col
    #cv2.imshow("org_gray",gray)
    gray[gray < 200] = 0
    gray[100:300, 200:440]=0
    #cv2.imshow("distorted",cv_img)
    
    #cv2.imshow("img",img)
    ROI = gray[140:220, 20:600]
    #ROI_org = ROI.copy()
    #cv2.imshow("undistorted",img)
    #cv2.imshow("ths_gray",gray)
    #cv2.waitKey(1)
    """
    cv2.imshow("roi_full",img)
    cv2.imshow("roi",ROI)
    cv2.waitKey()
    cv2.destroyAllWindows()
    """
    ROI = cv2.GaussianBlur(ROI, (7, 7), 0)
    left_ROI = ROI[:, : ROI.shape[1] // 2]
    right_ROI = ROI[:, ROI.shape[1] // 2 :]
    thr = cv2.adaptiveThreshold(
        ROI, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 9
    )
    edges = cv2.Canny(thr, 180, 460)
    
    """
    cv2.imshow("thr",thr)
    cv2.imshow("ROI",ROI)
    cv2.imshow("edges",edges)
    cv2.waitKey(1)
    cv2.destroyAllWindows()
    """

    # in left side, it finds only '/'type when jucha stage
    left_edge = edges[:, : edges.shape[1] // 2]
    # in right side, it finds only '\'type when jucha stage
    right_edge = edges[:, edges.shape[1] // 2 :]

    L_lines = cv2.HoughLines(left_edge, 1, np.pi / 180, 30)
    R_lines = cv2.HoughLines(right_edge, 1, np.pi / 180, 30)

    L = 0
    Ldegree = 0
    R = 0
    Rdegree = 0
    ii = 0

    R_dlist = []
    L_dlist = []

    ######### for visulization of extracted lines ###########
    bin3 = np.zeros(left_ROI.shape, dtype=np.uint8)
    bin3 = cv2.cvtColor(bin3, cv2.COLOR_GRAY2BGR)
    bin3[:, :, :] = 255
    bin4 = np.zeros(right_ROI.shape, dtype=np.uint8)
    bin4 = cv2.cvtColor(bin4, cv2.COLOR_GRAY2BGR)
    bin4[:, :, :] = 255
    #########################################################

    if L_lines is not None:
        # print("L_lines num: ",len(L_lines))
        for i in range(len(L_lines)):
            for rho, theta in L_lines[i]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))

                degree = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                if degree < -10 and L < 4:
                    L_dlist.append(degree)
                    ii += 1
                    #Ldegree = degree
                    L += 2
                    # cv2.line(left_ROI,(x1,y1),(x2,y2),(0,0,255),2)
                    cv2.line(bin3, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    break
                else:
                    continue

    if R_lines is not None:
        # print("R_lines num: ",len(R_lines))
        for i in range(len(R_lines)):
            for rho, theta in R_lines[i]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))

                degree = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                if degree > 10 and R < 4:
                    R_dlist.append(degree)
                    ii += 1
                    #Rdegree = degree
                    R += 2
                    # cv2.line(right_ROI,(x1,y1),(x2,y2),(0,0,255),2)
                    cv2.line(bin4, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    break
                else:
                    continue
    #print("extracted L: ",L, "R: ", R)
    #Rdegree += 2.
    if len(R_dlist) == 2:
        Rdegree = (R_dlist[0] + R_dlist[1])//2
    elif len(R_dlist) == 1:
        Rdegree = R_dlist[0]
    if len(L_dlist) == 2:
        Ldegree = (L_dlist[0] + L_dlist[1])//2
    elif len(L_dlist) == 1:
        Ldegree = L_dlist[0]

    binmerge_org = np.hstack((bin3, bin4))
    """
    cv2.imshow("binmerge_org",binmerge_org)
    cv2.waitKey()
    cv2.destroyAllWindows()
    """
    ROI = np.hstack((left_ROI, right_ROI))
    img[140:220, 20:600, :] = binmerge_org[:, :, :]
    
    if ii > 2:
        # print("==>2 line detected")
        angular = -(Ldegree + Rdegree) * 0.05
        # print("==>selected angluar :",angular)
        # if there are two lines, then angular_vel depends on difference of angle
    elif R > 0 or L > 0:
        if Ldegree == 0:
            # print("==>1 right line detected")
            if Rdegree > 87:
                angular = -(Rdegree - 92) * 0.03
                # print("==>1 right line detected and unbalanced no center position")
                # print("==>selected angluar :",angular)
            else:
                angular = -(Rdegree - 92) * 0.05
                # print("==>selected angluar :",angular)

        elif Rdegree == 0:
            # print("==>1 left line detected")
            if Ldegree < -87:
                angular = -(Ldegree + 92) * 0.03
                # print("==>1 left line detected and unbalanced no center position")
                # print("==>selected angluar :",angular)
            else:
                angular = -(Ldegree + 92) * 0.05
                # print("==>selected angluar :",angular)
    elif ii == 0:
        angular = -0.001
        #print("no line detected")
        #print("==>selected angluar :",-0.001)
    ### booster version
    
    angular *= 0.9
    if angular != 0.0:
        if angular > 1.0:
            angular = 1.0
        elif angular < -1.0:
            angular = -1.0
    linear = 1.6*(MAX_SPEED - (-0.03*np.exp(-abs(3.3*angular)) + 0.03))
    # print("prev linear: ",linear)
    if ii == 2 and abs(angular) < 0.2 :
        linear += 0.2*(((-1/0.2)*angular*angular) + 0.2)
        #linear += 0.3*(0.23 - abs(angular))
    percent = (linear/MAX_SPEED)
    TURTLE.set_speed_by_percentage(percent)
    TURTLE.set_angular(angular)
    '''
    ### normal version
    if angular > 0.8:
        angular = 0.8
    elif angular < -0.8:
        angular = -0.8
    linear = (MAX_SPEED - (-0.03*np.exp(-abs(3.3*angular)) + 0.03))
    if ii == 2 and abs(angular) < 0.23 :
        linear += 0.2*(((-1/0.23)*angular*angular) + 0.23)
        #linear += 0.3*(0.23 - abs(angular))
    percent = (linear/MAX_SPEED)
    '''
    '''
    if angular>0.23 or angular<-0.23:
        percent = (0.11/MAX_SPEED)
        print('curve')
    else:
        percent = (0.13/MAX_SPEED)
    '''
    #turtlemove(linear, 1.2*angular)
    # assume max speed = 0.15
    # percentage = (linear/max_speed) * 100
    
    _DEBUG = False
    if _DEBUG == True:
        print('Ldegree: %0.2f / Rdegree: %0.2f' % (Ldegree, Rdegree))
        print("detected line: ", ii)
        print("linear:",linear)
        print("percent:", percent)
        print("angular:",angular)
        print("\n")
        img_ = cv2.resize(img, (img.shape[1]*2, img.shape[0]*2), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("final_result", img_)
        cv2.waitKey(1)
    ##########################################################################
    both_calcul_flag = False
    '''
    print("linear: ",linear)
    
    if angular>0.26 or angular<-0.26:
        turtlemove(0.12,1.3*angular)
        print('curve')
    elif angular>0.23 or angular<-0.23:
        turtlemove(0.13,1.2*angular)
        print('curve')
    if angular>0.1 or angular<-0.1:
        turtlemove(0.14,1.1*angular)
        print('curve')
    else:
        print('no curve')
        turtlemove(0.15,angular)
    
    
    if RUN_MODE == 'debug':
        img_ = cv2.resize(img, (img.shape[1]*2, img.shape[0]*2), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("final_result", img_)
        cv2.waitKey(1)
    '''
    
    
def trace_one_line(compressed_data, LINE_BASE):
    global one_calcul_flag
    if one_calcul_flag is True:
        return
    one_calcul_flag = True
    
    ######################################################################
    _DEBUG = False
    """ view image """
    raw_data = np.fromstring(compressed_data.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)

    Knew = K.copy()
    Knew[(0, 1), (0, 1)] = 0.8 * Knew[(0, 1), (0, 1)]
    img = cv2.fisheye.undistortImage(cv_img, K, D=D, Knew=Knew)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray[gray < 200] = 0
    gray[100:300, 200:440] = 0
    # cv2.imshow("distorted",cv_img)
    
    ROI = gray[160:280, 20:600]
    ROI_org = ROI.copy()
    # cv2.imshow("undistorted",img)
    # cv2.imshow("ths_gray",gray)
    # cv2.waitKey(1)
    ROI = cv2.GaussianBlur(ROI, (7, 7), 0)
    degree_ref = 10
    if LINE_BASE == 1:
        ROI = ROI[:, :ROI.shape[1]//2]  ## LEFT BASE
    elif LINE_BASE == 2:
        ROI = ROI[:, ROI.shape[1]//2:]  ## RIGHT BASE
    thr = cv2.adaptiveThreshold(
        ROI, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 9)
    edge = cv2.Canny(thr, 180, 460)
    #cv2.imshow("edge",edge)
    lines = cv2.HoughLines(edge, 1, np.pi/180, 30)
    #print("lines: ",len(lines))
    line_cnt = 0
    degree = 0.
    weight = 0.
    
    x_list = []
    degree_list = []
    distance_list = []
    ii = 0

    ######### for visulization of extracted lines ###########
    bin3 = np.zeros(ROI.shape, dtype=np.uint8)
    bin3 = cv2.cvtColor(bin3, cv2.COLOR_GRAY2BGR)
    bin3[:, :, :] = 255
    #########################################################
    distance = bin3.shape[1]-0.91
    if lines is not None:
        for i in range(len(lines)):
            for rho, theta in lines[i]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0+1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))

                degree_ = np.arctan2(y2-y1, x2-x1)*180/np.pi
                #print("checked degree: ",degree_)
                if abs(degree_) > degree_ref and ii < 2:
                    ii += 1
                    #degree = degree_
                    degree_list.append(degree_)
                    
                    x_list.append(x1)
                    cv2.line(bin3, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    for bin3_x in range(bin3.shape[1]):
                        if LINE_BASE == 1 and bin3[bin3.shape[0]//2,bin3_x,0] != 255:                       ## LEFT SIDE
                            distance_list.append(bin3.shape[1] - bin3_x)
                            break
                        elif LINE_BASE == 2 and bin3[bin3.shape[0]//2,bin3_x,0] != 255:       ## RIGHT SIDE
                            distance_list.append(bin3_x)
                            break
                    #print("real_distance: ",distance)
                    #distance = (distance - 0)/((bin3.shape[1]-0.91)-0)                                         ## distance normalization
                    #print("distance: ",distance) 
                    break
                else:
                    continue
    if len(distance_list) == 2:
        distance = (distance_list[0] + distance_list[1])/2
    elif len(distance_list) ==1:
        distance = distance_list[0]
    else:
        pass 
    distance = (distance - 0)/((bin3.shape[1]-0.91)-0)
    if len(x_list) == 2:
        #print("degree1: ",degree_list[0],"degree2: ",degree_list[1])
        if degree_list[0]*degree_list[1] < 0:
            if LINE_BASE == 1:
                degree = -90
            else:
                degree = 90
        else:
            degree = (degree_list[0] + degree_list[1])/2
    elif len(x_list) == 1:
        degree = degree_list[0]
    #print("extracted L: ",L, "R: ", R)
    #Rdegree += 2.
    
    '''
    cv2.imshow("binmerge_org",binmerge_org)
    cv2.waitKey()
    cv2.destroyAllWindows()
    '''
    if LINE_BASE == 1:
        img[160:280, 20:ROI.shape[1]+20, :] = bin3[:, :, :]         ## LEFT SIDE
    elif LINE_BASE == 2:
        img[160:280, ROI.shape[1]+19:599, :] = bin3[:, :, :]        ## RIGHT SIDE
        
    
    
    if ii != 0:
        '''
        if LINE_BASE == 1:                                          ## LEFT SIDE
            if degree > 0:                                          ## LEFT TURN
                weight = distance**2                                ## the shorter the distance, the weaker the weight 
            else:                                                   ## RIGHT TURN
                weight = (distance - 1)**2 + 1                      ## the shorter the distance, the stronger the weight
        elif LINE_BASE == 2:                                        ## RIGHT SIDE
            if degree > 0:                                          ## LEFT TURN
                weight = (distance - 1)**2 + 1                      ## the shorter the distance, the stronger the weight
            else:                                                   ## RIGHT TURN
                weight = distance**2                                ## the shorter the distance, the weaker the weight
        '''
        weight = 0.6*(1 - distance) 
        if degree < 0 :     ## '/' shape
            if LINE_BASE == 1:
                angular = -(degree+88)*0.04 - weight
            else:
                angular = -(degree+88)*0.04 + weight
            angular -= 0.06
        else:               ## '\' shape
            if LINE_BASE == 1:
                angular = -(degree-88)*0.04 - weight
            else:
                angular = -(degree-88)*0.04 + weight
        if angular > 0.7:
            angular = 0.7
        elif angular < -0.7:
            angular = -0.7
        #print("init angular: ",angular)
        #print("init weight: ",weight)
    else:
        if LINE_BASE == 1:
            angular = 0.7
        if LINE_BASE == 2:
            angular = -0.7
    ################ Boosted speed ###################################
    linear = 1.5*(MAX_SPEED -(-0.03*np.exp(-abs(3.3*angular)) + 0.03))
    if ii == 2 and abs(angular) < 0.15 :
        linear += 0.1*(((-1/0.15)*angular*angular) + 0.15)
    #angular = 1.1*angular
    percent = (linear/MAX_SPEED)
    TURTLE.set_speed_by_percentage(percent)
    TURTLE.set_angular(angular)
    ##################################################################
    '''
    ############### Normal speed #####################################
    linear = (MAX_SPEED -(-0.03*np.exp(-abs(3.3*angular)) + 0.03))
    if ii == 2 and abs(angular) < 0.15 :
        linear += 0.1*(((-1/0.23)*angular*angular) + 0.15)
    angular = angular
    percent = (linear/MAX_SPEED)
    ##################################################################
    '''
    #turtlemove(linear, 1.2*angular)
    # assume max speed = 0.15
    # percentage = (linear/max_speed) * 100
    _DEBUG = False
    if _DEBUG is True:
        print('degree: %0.2f ' % (degree))
        print("detected line: ", ii)
        print("distance: ",distance) 
        print("percent:", percent)
        print("angular:",angular)
        print("")
        img_ = cv2.resize(img, (img.shape[1]*2, img.shape[0]*2), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("final_result", img_)
        # cv2.waitKey(1)
    #TURTLE.move()
    #############################################################################
    one_calcul_flag = False
    
    


def gogo():
    print("running")
    TURTLE.set_speed_by_percentage(0.5)
    TURTLE.set_angular(0)
    TURTLE.move()


def get_num_of_lines():
    """ get the number of lines detected """
    return 1 / 2
