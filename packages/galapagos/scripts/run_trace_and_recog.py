#! Deprecated

from lib_signal_recognition import *
from lib_line_tracing import *

state = STATES['normal']


def trace_line(compressed_data):
    """ view image """
    # print("viewing image.")
    # rospy.loginfo("img_viewer()")
    raw_data = np.fromstring(compressed_data.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)

    Knew = K.copy()
    Knew[(0, 1), (0, 1)] = 0.8 * Knew[(0, 1), (0, 1)]
    img = cv2.fisheye.undistortImage(cv_img, K, D=D, Knew=Knew)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray[gray < 220] = 0
    '''
    cv2.imshow("ths_gray",gray)
    cv2.waitKey()
    cv2.destroyAllWindows()
    '''
    # cv2.line(img,(35,140),(595,140),(253,244,8),2)        ## top row
    # cv2.line(img,(35,280),(595,280),(253,244,8),2)        ## bottom row
    # cv2.line(img,(35,140),(35,280),(253,244,8),2)        ## left col
    # cv2.line(img,(595,140),(595,280),(253,244,8),2)        ## right col
    ROI = gray[140:280, 30:610]
    ROI_org = ROI.copy()
    '''
    cv2.imshow("roi_full",img)
    cv2.imshow("roi",ROI)
    cv2.waitKey()
    cv2.destroyAllWindows()
    '''
    ROI = cv2.GaussianBlur(ROI, (7, 7), 0)
    left_ROI = ROI[:, :ROI.shape[1]//2]
    right_ROI = ROI[:, ROI.shape[1]//2:]
    thr = cv2.adaptiveThreshold(
        ROI, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 9)
    edges = cv2.Canny(thr, 180, 460)
    '''
    cv2.imshow("thr",thr)
    cv2.imshow("ROI",ROI)
    cv2.imshow("edges",edges)
    cv2.waitKey()
    cv2.destroyAllWindows()
    '''

    # in left side, it finds only '/'type when jucha stage
    left_edge = edges[:, :edges.shape[1]//2]
    # in right side, it finds only '\'type when jucha stage
    right_edge = edges[:, edges.shape[1]//2:]

    L_lines = cv2.HoughLines(left_edge, 1, np.pi/180, 30)
    R_lines = cv2.HoughLines(right_edge, 1, np.pi/180, 30)

    L = 0
    Ldegree = 0

    R = 0
    Rdegree = 0

    ii = 0

    ######### for visulization of extracted lines ###########
    bin3 = np.zeros(left_ROI.shape, dtype=np.uint8)
    bin3 = cv2.cvtColor(bin3, cv2.COLOR_GRAY2BGR)
    bin3[:, :, :] = 255
    bin4 = np.zeros(right_ROI.shape, dtype=np.uint8)
    bin4 = cv2.cvtColor(bin4, cv2.COLOR_GRAY2BGR)
    bin4[:, :, :] = 255
    #########################################################

    if L_lines is not None:
        #print("L_lines num: ",len(L_lines))
        for i in range(len(L_lines)):
            for rho, theta in L_lines[i]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0+1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))

                degree = np.arctan2(y2-y1, x2-x1)*180/np.pi
                if degree < -10 and L < 2:
                    ii += 1
                    Ldegree = degree
                    L += 2
                    # cv2.line(left_ROI,(x1,y1),(x2,y2),(0,0,255),2)
                    cv2.line(bin3, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    break
                else:
                    continue

    if R_lines is not None:
        #print("R_lines num: ",len(R_lines))
        for i in range(len(R_lines)):
            for rho, theta in R_lines[i]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0+1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))

                degree = np.arctan2(y2-y1, x2-x1)*180/np.pi
                if degree > 10 and R < 2:
                    ii += 1
                    Rdegree = degree
                    R += 2
                    # cv2.line(right_ROI,(x1,y1),(x2,y2),(0,0,255),2)
                    cv2.line(bin4, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    break
                else:
                    continue
    #print("extracted L: ",L, "R: ", R)
    Rdegree += 2.
    binmerge_org = np.hstack((bin3, bin4))
    '''
    cv2.imshow("binmerge_org",binmerge_org)
    cv2.waitKey()
    cv2.destroyAllWindows()
    '''
    ROI = np.hstack((left_ROI, right_ROI))
    img[140:280, 30:610, :] = binmerge_org[:, :, :]
    #print('Ldegree: %0.2f / Rdegree: %0.2f' % (Ldegree, Rdegree))
    # print("")
    if ii == 2:
        #print("==>2 line detected")
        angular = -(Ldegree+Rdegree)*0.05
        #print("==>selected angluar :",angular)
        # if there are two lines, then angular_vel depends on difference of angle
    elif ii == 1:
        if Ldegree == 0:
            #print("==>1 right line detected")
            if Rdegree > 88:
                angular = -(Rdegree-92)*0.01
                #print("==>1 right line detected and unbalanced no center position")
                #print("==>selected angluar :",angular)
            else:
                angular = -(Rdegree-92)*0.04
                #print("==>selected angluar :",angular)

        else:
            #print("==>1 left line detected")
            if Ldegree < -88:
                angular = -(Ldegree+92)*0.01
                #print("==>1 left line detected and unbalanced no center position")
                #print("==>selected angluar :",angular)
            else:
                angular = -(Ldegree+92)*0.04
                #print("==>selected angluar :",angular)
    else:
        angular = -0.001
        #print("no line detected")
        #print("==>selected angluar :",-0.001)
    angular = angular*2
    linear = MAX_SPEED - (-0.05*np.exp(-abs(angular)) + 0.05)
    #turtlemove(linear, 1.2*angular)
    # assume max speed = 0.15
    # percentage = (linear/max_speed) * 100
    percent = (linear/MAX_SPEED)
    print("percent:", percent)
    TURTLE.set_speed_by_percentage(percent)
    TURTLE.set_angular(angular)
    TURTLE.move()
    # TURTLE.get_info()
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
    '''

    if RUN_MODE == 'debug':
        img_ = cv2.resize(
            img, (img.shape[1]*2, img.shape[0]*2), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("final_result", img_)
        cv2.waitKey(1)


def trace_and_recog(front_compressed):
    global state
    raw_data = np.fromstring(front_compressed.data, np.uint8)
    cv_img = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)
    tmp_state = 'normal'
    #### ROI SETTING ######
    blob_ROI = cv_img[100:, :]
    #######################
    if state == STATES['normal']:
        tmp_state = is_parking(blob_ROI)
        if tmp_state == True:
            TURTLE.set_speed_by_percentage(0)
            TURTLE.set_angular(0.)
            state = STATES['parking']
        else:
            pass
    elif state == STATES['parking']:
        print("=====================")
        print("present state: ", state)
        print("=====================")
    # TURTLE.move()


rospy.Subscriber(PATH_RASPICAM,CompressedImage, trace_line,  queue_size = 1) ## Used for Detecting Sinho,Jucha
rospy.Subscriber('/usbcam/image_raw/compressed',CompressedImage, trace_and_recog,  queue_size = 1) ## Used for Detecting Sinho,Jucha
rospy.spin()
