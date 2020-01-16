#!/usr/bin/env python3

import rospy
from scheduler import SCHEDULER

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
# from cv_bridge import CvBridge

import sys
import cv2
import numpy as np
import time
from constants import ARRAY_D, ARRAY_K, BUF_SIZE
from operator import itemgetter
from pprint import pprint  # for Debug
# from cv_bridge import CvImage, CvBridge, CvBridgeError

# * Variables
MIN = np.array([int(0 * 255 / 360), int(0 * 255 / 100), int(0 * 255 / 100)])
MAX = np.array([int(360 * 255 / 360), int(22 * 255 / 100), int(100 * 255 / 100)])

MIN_FISH = np.array([int(0 * 255 / 360), int(0 * 255 / 100), int(0 * 255 / 100)])
MAX_FISH = np.array([int(360 * 255 / 360), int(52 * 255 / 100), int(100 * 255 / 100)])

# NOTE: catches yellow line only
# MIN_SUB_Y = np.array([int(1 * 255 / 360), int(78 * 255 / 100), int(0 * 255 / 100)])
# MAX_SUB_Y = np.array([int(270 * 255 / 360), int(100 * 255 / 100), int(100 * 255 / 100)])
MIN_SUB_Y = np.array([int(50 * 255 / 360), int(78 * 255 / 100), int(0 * 255 / 100)])
MAX_SUB_Y = np.array([int(280 * 255 / 360), int(100 * 255 / 100), int(100 * 255 / 100)])

# NOTE: catches both yellow and white line
MIN_SUB = np.array([int(0 * 255 / 360), int(73 * 255 / 100), int(75 * 255 / 100)])
MAX_SUB = np.array([int(360 * 255 / 360), int(100 * 255 / 100), int(100 * 255 / 100)])

CAMERA_MATRIX = np.array(ARRAY_K)
# DISTORTION_MATRIX = np.array([ -0.335978, 0.133403, 0.000953, 0.000789 ])
DISTORTION_MATRIX = np.array([ -0.335978, 0.133403, 0.000953, -0.005089 ])

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

IMAGE_FISH_WIDTH = 640
IMAGE_FISH_HEIGHT = 100

IMAGE_SUB_WIDTH = 640
IMAGE_SUB_HEIGHT = 480

class Eye(dict):
    """ an object to process image """

    def __init__(self):
        # self.has_rospy = True # ! deprecated
        self.images = {}
        self.mask = None
        self.check_yellow = True
        self.is_window_set = False
        self.info_front = {
            "state": "straight",
            "center": 0,
            "bias": 0,
            "turning_to": "None",
            "horizon_position": 0.
        }
        self.info_fish = {
            "left": -1,
            "right": -1,
            "slope": -1
        }
        self.info_sub = {
            "center": 0,
            "slope": 0,
            "has_line": False
        }
        self._buffer_sub = []

        self.threshold_turning = 0
        # self.threshold_tracking = 0 # ! Deprecated
        self.threshold_sub_slope = 0
        self.threshold_sub_center = 0
        self.publishers = {}
        self.publishing_names = []

        if SCHEDULER.debug_mode:
            self.publishing_names = ["front/original", "front/canny", "front/adjusted", "front/result", "sub/original", "sub/canny", "sub/masked"]
            # self.publishing_names = []
        else:
            self.publishing_names = ["front/result", "sub/masked"]
        #     self.publishing_names = []

        for i in self.publishing_names:
            # print("publisher: ", i)
            self.publishers[str(i)] = rospy.Publisher(
                "/eye/" + i + "/compressed", CompressedImage, queue_size=5
            )

        self.calibrate()
        dict.__init__(self)

    def calibrate(self):
        """ process to calibrate initial values """
        self.threshold_turning = 260
        # self.threshold_tracking = 260 # ! Deprecated
        self.threshold_sub_slope = -1.58
        self.threshold_sub_center = 200
        
        rospy.logdebug("[LIB_EYE] threshold_turning set to " + str(self.threshold_turning))
        # rospy.logdebug("[LIB_EYE] threshold_tracking set to " + str(self.threshold_tracking))
        rospy.logdebug("[LIB_EYE] threshold_sub_slope set to " + str(self.threshold_sub_slope))
        rospy.logdebug("[LIB_EYE] threshold_sub_center set to " + str(self.threshold_sub_center))

    def reset_state(self, event=None):
        self.info_front["state"] = "straight"
        self.info_front["turning_to"] = "None"

    def see_front(self, compressed_data=None, image_path=None):
        """ callback handler when front image received """
        SCHEDULER.is_occupied["frontcam"] = True
        self.set_front_image(compressed_data, image_path)

        road_lines = self.get_road_line_by_front()
        if road_lines is None:
            return

        # right_points = [] # ! Deprecated
        # left_points = [] # ! Deprecated
        right_side, left_side = [], []
        right_cnt, left_cnt = 0, 0

        # NOTE: Threshholds for getting the starting point to flood
        limit_max_x = 500
        limit_min_y = 170

        # NOTE: Select two lines per each side(right or left) which are closed to the bottom
        for line in road_lines:
            if right_cnt == 2 and left_cnt == 2:
                break
            if  (line["slope"] > 0
           and line["points"][2] > IMAGE_WIDTH - limit_max_x
              and line["points"][3] > limit_min_y
              and right_cnt < 2):
                right_cnt += 1
                right_side.append(line)
            elif (line["slope"] < 0
           and line["points"][2] < limit_max_x
              and line["points"][3] > limit_min_y
              and left_cnt < 2):
                left_cnt += 1
                left_side.append(line)
            # else:
                # print(f"other lines: {line}")
        
        # NOTE: To get the middle point of two lines
        # rospy.logdebug("right_side: " + str(right_side))
        if len(right_side) is 2:
            right_point = []
            for idx, key in enumerate(right_side):
                right_point.append((right_side[0]["points"][idx] + right_side[1]["points"][idx]) / 2)
        elif len(right_side) is 1:
            right_point = right_side[0]["points"]

        if len(left_side) is 2:
            left_point = []
            for idx, key in enumerate(left_side):
                left_point.append((left_side[0]["points"][idx] + left_side[1]["points"][idx]) / 2)
        elif len(left_side) is 1:
            left_point = left_side[0]["points"]
        
        # rospy.logdebug("right: " + str(right_point))


        # ! deprecated
        # for line in road_lines:
        #     if line["slope"] > 0:
      #         right_points.append(line["middle_point"])
        #     else:
        #         left_points.append(line["middle_point"])

        # print(f"road_lines: \nright: {right_side}, \n left: {left_side}\n\n")

        # print(f"right: {right_points}, left: {left_points}")
        # self.images["front/test"] = cv2.cvtColor(self.images["front/test"], cv2.COLOR_GRAY2BGR)
        mask = np.zeros(shape=[IMAGE_HEIGHT, IMAGE_WIDTH], dtype=np.uint8)
        mask.fill(0)
        if right_side:
            mask = cv2.bitwise_or(self.get_flooded(self.images["front/test"], right_point), mask)
            # mask = cv2.bitwise_or(self.get_flooded(self.images["front/test"], right_side[0]["points"]), mask)
        if left_side:
            mask = cv2.bitwise_or(self.get_flooded(self.images["front/test"], left_point), mask)
            # mask = cv2.bitwise_or(self.get_flooded(self.images["front/test"], left_side[0]["points"]), mask)

        self.images["front/result"] = cv2.bitwise_and(self.images["front/original"], self.images["front/original"], mask=mask)
        self.images["front/canny"] = cv2.bitwise_and(self.images["front/canny"], self.images["front/canny"], mask=mask)
        # self.images["canny3"]
        # print(f"shape: {self.images['result'].shape}")

        horizons = self.get_horizon()

        if horizons:
            # rospy.logdebug("horizons: " + str(horizons))
            self.set_horizon_position(horizons)

        # NOTE: set_center() must be placed after set_horizon_position to use horizon_position
        self.set_center(left_side, right_side)

        # if self.info_front["center"] is 0:
        #     # NOTE: self.info_front["center"] == 0 is unconsidered
        #     rospy.logdebug("unconsidered")
        #     SCHEDULER.is_occupied["frontcam"] = False
        #     return self.info_front

        if self.threshold_turning < self.info_front["horizon_position"]:
            self.info_front["state"] = "turning"
        # elif self.threshold_tracking < self.info_front["horizon_position"]:
        #     self.info_front["state"] = "lost_track"
        # else:
        #     self.info_front["state"] = "straight"

        self.draw_lines(horizons, self.images["front/canny"])
        self.draw_lines(road_lines, self.images["front/canny"])
        self.draw_lines(right_side, self.images["front/canny"], color="brown")
        self.draw_lines(left_side, self.images["front/canny"], color="brown")
        self.set_info_to_image(self.images["front/result"])
    
        self.publish_image()

        return self.info_front
    
    def see_bottom(self, compressed_data=None):
        """ callack handler when fish image received """
        SCHEDULER.is_occupied["fishcam"] = True
        self.set_fish_image(compressed_data)

        # rospy.logdebug("\n[EYE] Calculating bottom cam...")
        road_lines = self.get_road_line_by_fish()
        if road_lines is None:
            self.info_fish = {
                # "len": len(road_lines),
                "left": 0,
                "right": IMAGE_FISH_WIDTH,
                "slope": 0
            }
            return self.info_fish
        
        # rospy.logdebug("road_lines: " + str(road_lines))
        left_outer = road_lines[0]["points"][0]
        left_inner = road_lines[-1]["points"][0]
        right_outer = road_lines[-1]["points"][0]
        right_inner = road_lines[0]["points"][0]

        if (left_inner > 2 * IMAGE_FISH_WIDTH / 3) or (left_inner - left_outer < 10):
            left = 0
        else:
            left = left_outer
        if (right_inner < 2 * IMAGE_FISH_WIDTH / 3) or (right_outer - right_inner < 10):
            right = IMAGE_FISH_WIDTH
        else:
            right = right_outer

        self.info_fish = {
            # "len": len(road_lines),
            "left": left,
            "right": right,
            "slope": road_lines[-1]["slope"]
        }
        
        # self.draw_lines(road_lines, self.images["fish_canny"], color="blue")
        # self.set_info_fish_to_image(self.images["fish_canny"])
        # self.publish_image()

        # self.release_fish_occupied()
        return self.info_fish

    def see_sub(self, compressed_data=None):
        """ callback handler when sub image received """
        SCHEDULER.is_occupied["subcam"] = True
        self.set_sub_image(compressed_data)

        road_lines = self.get_road_line_by_sub()
        if road_lines is not None and len(road_lines) is not 4:
            if SCHEDULER.debug_option["warn_road_lines"]:
                rospy.logwarn("[LIB_EYE] road_lines: ")
                pprint(road_lines)
        # if road_lines is None:
        #     self.info_fish = {
        #         # "len": len(road_lines),
        #         "left": 0,
        #         "right": IMAGE_FISH_WIDTH,
        #         "slope": 0
        #     }
        #     return self.info_fish

        # rospy.logdebug("\n[EYE] SUB lines: " + str(road_lines))

        if not isinstance(road_lines, list):
            self.info_sub["has_line"] = False
        elif len(road_lines) < 2:
            self.info_sub["has_line"] = False
        else:
            self.info_sub["has_line"] = True

        if road_lines and len(road_lines) > 1:
            sum_x, sum_y = 0, 0
            divider = len(road_lines) * 2
            for line in road_lines:
                sum_x = sum_x + line['points'][0] + line['points'][2]
                sum_y = sum_y + line['points'][1] + line['points'][3]
            average_x, average_y = sum_x / divider, sum_y / divider

            self._buffer_sub.append(average_x)
            # rospy.logdebug("\n[EYE] average: {:.2f} {:.2f}".format(average_x, average_y))
            if len(self._buffer_sub) == BUF_SIZE:
                self.info_sub["center"] = int(sum(self._buffer_sub) / BUF_SIZE)  - self.threshold_sub_center
                # self.info_sub["line_center"][0] = self.info_sub["line_center"][2] = int(sum(self._buffer_sub) / BUF_SIZE)
                self._buffer_sub.pop(0)
            else:
                rospy.logdebug("[LIB_EYE] len(_buffer_sub): {:d}".format(len(self._buffer_sub)))

            # self.info_sub["line_center"][3] = int(average_y)
            # self.info_sub["line_center"][3] = 240

        if road_lines:
            slope = 0
            for line in road_lines:
                slope = slope + line['slope']
            # NOTE: 1 added to slope due to the lens is biased to 45 degree
            self.info_sub["slope"] = float("{:.2f}".format(
                1 + (slope / len(road_lines))))
            
            image_center = int(self.info_sub["center"] + (IMAGE_SUB_WIDTH /2))
            line_center = [{
                "points": [image_center, 0, image_center, 480]
            }]
            self.draw_lines(line_center, self.images["sub/canny"], color="blue")
        self.draw_lines(road_lines, self.images["sub/canny"], color="blue")
        # self.set_info_fish_to_image(self.images["sub/canny"])

        self.publish_image()
        return self.info_sub

    def set_front_image(self, compressed_data=None, image_path=None):
        if compressed_data:
            raw_data = np.fromstring(compressed_data.data, np.uint8)
            self.images["front/original"] = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)
        elif image_path is not None:
            self.images["front/original"] = cv2.imread(image_path, cv2.IMREAD_COLOR)
        else:
            rospy.logerror("no compressed image and image on folder are found")
            return

        try:
            self.images["front/original"] = cv2.resize(
                self.images["front/original"], dsize=(IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA
            )
        except cv2.error:
            print("error occurred while loading image_path")
            return []

        # print("self.images['original']: ", self.images["front/original"])
    
    def set_fish_image(self, compressed_data=None):
        if compressed_data:
            raw_data = np.fromstring(compressed_data.data, np.uint8)
            self.images["fish"] = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)
        else:
            rospy.logerror("no compressed image and image on folder are found")
            return
        
        self.images["fish"] = cv2.resize(
            self.images["fish"], dsize=(IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA
        )
        self.images["fish"] = cv2.undistort(self.images["fish"], CAMERA_MATRIX, DISTORTION_MATRIX)[50:50+IMAGE_FISH_HEIGHT, :]
        # self.images["fish"] = self.images["fish"][:150, :]

        return

    def set_sub_image(self, compressed_data=None):
        if compressed_data:
            raw_data = np.fromstring(compressed_data.data, np.uint8)
            self.images["sub"] = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)
        else:
            rospy.logerror("no compressed image and image on folder are found")
            return
        
        self.images["sub"] = cv2.resize(
            self.images["sub"], dsize=(IMAGE_SUB_WIDTH, IMAGE_SUB_HEIGHT), interpolation=cv2.INTER_AREA
        )
        # self.images["sub"] = cv2.undistort(self.images["sub"], CAMERA_MATRIX, DISTORTION_MATRIX)
        # self.images["sub"] = self.images["sub"][:150, :]

        return

    def draw_lines(self, lines, to, color="green"):
        """ draw lines to a canvas """
        # print('lines: ', lines)
        if lines is not None:
            # for idx, key in enumerate(lines):
            #     cv2.line(to, (key['points'][0], key['points'][1]), (key['points'][2], key['points'][3]),
            #              (255, 30 + 60 * idx, 255), 3, cv2.LINE_AA)
            idx = 60
            for l in lines:
                if color is "brown":
                    cv2.line(
                        to,
                        (l["points"][0], l["points"][1]),
                        (l["points"][2], l["points"][3]),
                        (idx*0.7, idx*0.7, 50 + idx*0.9),
                        cv2.LINE_AA
                    )
                elif color is "blue":
                    cv2.line(
                        to,
                        (l["points"][0], l["points"][1]),
                        (l["points"][2], l["points"][3]),
                        (50 + idx*0.9, idx*0.7, idx*0.7),
                        cv2.LINE_AA
                    )
                elif isinstance(l, dict):
                    cv2.line(
                        to,
                        (l["points"][0], l["points"][1]),
                        (l["points"][2], l["points"][3]),
                        (50, idx, 120),
                        3,
                        cv2.LINE_AA,
                    )
                else:
                    cv2.line(
                        to,
                        (l[0], l[1]),
                        (l[2], l[3]),
                        (100 + idx, 30 + 60 * idx, 70),
                        3,
                        cv2.LINE_AA,
                    )
                idx = idx + 60

    def get_front_state(self):
        return self.info_front["state"]
    
    def get_turning_to(self):
        return self.info_front["turning_to"]
    
    def get_road_line_by_front(self):
        """ getting lines from front """
        # image = cv2.GaussianBlur(image, (7, 7), 0)

        # self.images["front/original"] = cv2.cvtColor(self.images["front/original"], cv2.COLOR_BGR2HSV)
        # self.images["front/original"] = cv2.fastNlMeansDenoisingColored(self.images["front/original"], 10, 10, 7, 21)

        adjusted = None
        # NOTE: loop over various values of gamma
        # for gamma in np.arange():
        # NOTE: ignore when gamma is 1 (there will be no change to the image)
        # if gamma == 1:
        # continue

        # NOTE: apply gamma correction and show the images
        # gamma = gamma if gamma > 0 else 0.1

        gamma = 0.2
        adjusted = self.adjust_gamma(self.images["front/original"], gamma=gamma)
        # adjusted = self.images["front/original"]

        # cv2.putText(
        #     adjusted,
        #     "g={}".format(gamma),
        #     (10, 30),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     0.8,
        #     (0, 0, 255),
        #     3,
        # )

        # img_yuv = cv2.cvtColor(adjusted, cv2.COLOR_BGR2YUV)
        # # NOTE: equalize the histogram of the Y channel
        # img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
        # # NOTE: convert the YUV image back to BGR format
        # img_yuv = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
        # blur = cv2.GaussianBlur(img_yuv, (9, 9), 0)

        # NOTE: BilateralFilter is slow compared to most filter processes
        bilateral = cv2.bilateralFilter(adjusted, 5, 10, 300)

        # self.images["front/test"] = bilateral
        self.images["front/adjusted"] = bilateral
        # gray = cv2.cvtColor(self.images["front/original"], cv2.COLOR_BGR2GRAY)
        # blur = cv2.GaussianBlur(gray, (9, 9), 0)
        # log = cv2.Laplacian(blur, cv2.CV_64F, ksize=7)
        # self.images["front/original"] = cv2.Canny(log, 100, 200, None, 7)

        # canny = cv2.Canny(self.images["front/original"], 100, 200, None, 7)
        # log = np.uint8(cv2.Laplacian(self.images["front/original"], cv2.CV_64F, ksize=7))
        # self.images["front/test"] = log
        # self.images["front/test"] = cv2.Canny(np.uint8(log), 100, 200, None, 7)

        # self.images["front/original"] = cv2.Sobel(self.images["front/original"],cv2.CV_64F,1,0,ksize=31)

        white = np.zeros(shape=[IMAGE_HEIGHT, IMAGE_WIDTH, 3], dtype=np.uint8)
        # white[:] = (255, 255, 255)
        white.fill(255)

        self.mask = cv2.inRange(self.images["front/adjusted"], MIN, MAX)
        # _masked = cv2.cvtColor(cv2.bitwise_and(
        # image, image, mask=mask), cv2.COLOR_HSV2BGR)
        self.images["front/masked"] = cv2.cvtColor(
            cv2.bitwise_and(white, white, mask=self.mask), cv2.COLOR_HSV2BGR
        )

        # kernel = np.ones((5, 5), np.uint8)
        # images['erode'] = cv2.erode(images['masked'], kernel, iterations=1)
        # images['dilation'] = cv2.dilate(images['erode'], kernel, iterations=1)
        # image_opening = cv2.morphologyEx(images['dilation'], cv2.MORPH_OPEN, kernel)

        self.images["front/_canny"] = cv2.Canny(self.images["front/masked"], 100, 200, None, 7)

        gray = self.images["front/_canny"]
        # # gray = cv2.cvtColor(self.images["front/_canny"], cv2.COLOR_BGR2GRAY)
        # _, contours, _ = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)  # convert to grayscale
        blur = cv2.blur(gray, (3, 3))  # blur the image
        ret, thresh = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY)

        # NOTE: Finding contours for the thresholded image
        im2, contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        # NOTE: for openCV 4.0
        # contours, hierarchy = cv2.findContours(
        #     thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        # )

        # # NOTE: create hull array for convex hull points
        # hull = []

        # print(f"contours: {contours}")
        selected_contours = []
        for idx, key in enumerate(contours):
            area = cv2.contourArea(key)
            # print(f"contour area: {area}")
            if area > 500:
                selected_contours.append(idx)

        # # NOTE: calculate points for each contour
        # for i in range(len(contours)):
        #     # creating convex hull object for each contour
        #     hull.append(cv2.convexHull(contours[i], False))
        # # hull = cv2.convexHull(contours[10])

        # NOTE: create an empty black image
        drawing = np.zeros((thresh.shape[0], thresh.shape[1], 3), np.uint8)

        # NOTE: draw contours and hull points
        # for i in range(len(contours)):
        for i in selected_contours:
            color_contours = (0, 255, 0)  # green - color for contours
            # color = (255, 0, 0)  # blue - color for convex hull
            # NOTE: draw ith contour
            # self.images["front/test"] = cv2.drawContours(drawing, contours, i, color_contours, 3, 8, hierarchy)
            self.images["front/_canny"] = cv2.drawContours(
                drawing, contours, i, color_contours, 1, 4, hierarchy
            )
            self.images["front/_canny"] = cv2.cvtColor(
                self.images["front/_canny"], cv2.COLOR_BGR2GRAY
            )

            # NOTE: draw ith convex hull object
            # self.images["front/test"] = cv2.drawContours(drawing, hull, i, color, 1, 8)

        # NOTE: crop the top
        # print(self.images["front/_canny"].shape, white.shape)
        # self.images["front/_canny"][:100, :].fill(0)
        # print(self.images["front/_canny"])

        # NOTE: watershed algorithm
        gray = cv2.cvtColor(self.images["front/masked"], cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        # NOTE: noise removal
        kernel = np.ones((3, 3), np.uint8)
        # opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)

        # NOTE: sure background area
        # sure_bg = cv2.dilate(opening, kernel, iterations=3)
        sure_bg = cv2.dilate(thresh, kernel, iterations=2)

        # # NOTE: Finding sure foreground area
        # dist_transform = cv2.distanceTransform(thresh, cv2.DIST_L2, 5)
        # result_dist_transform = cv2.normalize(
        #     dist_transform, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8UC1
        # )
        # ret, sure_fg = cv2.threshold(
        #     dist_transform, 0.7 * dist_transform.max(), 255, cv2.THRESH_BINARY
        # )

        # # NOTE: Finding unknown region
        # sure_fg = np.uint8(sure_fg)
        # unknown = cv2.subtract(sure_bg, sure_fg)

        # # NOTE: Marker labelling
        # ret, markers = cv2.connectedComponents(sure_fg)

        # # NOTE: Add one to all labels so that sure background is not 0, but 1
        # markers = markers + 1

        # # NOTE: Now, mark the region of unknown with zero
        # markers[unknown == 255] = 0

        # markers = cv2.watershed(self.images["front/masked"], markers)

        # self.images["front/masked"][markers == -1] = [255, 0, 0]
        # self.images["front/masked"][markers == 1] = [255, 255, 0]
        self.images["front/test"] = sure_bg

        # self.images["front/test"] = cv2.drawContours(
        #     self.images["front/test"], contours, -1, (0, 50, 255), 3
        # )

        self.images["front/canny"] = cv2.cvtColor(
            np.copy(self.images["front/_canny"]), cv2.COLOR_GRAY2BGR
        )
        # self.images["canny2"] = cv2.cvtColor(
        #     np.copy(self.images["front/_canny"]), cv2.COLOR_GRAY2BGR
        # )

        # image_roi = self.images["front/original"][240:, :]
        # _image_roi = np.copy(image_roi)

        hough_lines = cv2.HoughLinesP(
            self.images["front/_canny"], 1, np.pi / 180, 140, None, 50, 5
        )
        # print('hough_lines:', len(hough_lines))
        # horizons = []
        lines = []
        ref_point = [320, 310]

        if hough_lines is None:
            return None

        for i in range(0, len(hough_lines)):
            line = hough_lines[i][0]

            # # to select horizons:
            # if abs(line[3] - line[1]) < 3:
            #     if line[0] > line[2]:
            #         line = np.asarray([line[2], line[3], line[0], line[1]])
            #     horizons.append(line)

            # if the line seems "road line", append to lines:
            if ((line[3] - line[1]) == 0) or ((line[2] - line[0]) == 0):
                pass
            else:
                # if tan(theta) is less than 3:
                _slope = abs((line[3] - line[1]) / (line[2] - line[0]))
                if (_slope < 1000 and _slope > 0.07):
                # if (_slope < 1000 and _slope > 0.01):
                    # if the first point is x-axis higher than the second point, swap:
                    if line[1] > line[3]:
                        line = np.asarray([line[2], line[3], line[0], line[1]])
                    slope = float("%.2f" % ((line[2] - line[0]) / (line[3] - line[1])))
                    lines.append(
                        {
                            "slope": slope,
                            "points": line
                            # "y_intercept": line[1] / (line[0] * slope),
                        }
                    )

        lines = sorted(lines, key=itemgetter("slope"))
        # lines = sorted(lines, key=itemgetter("slope"))
        # print('lines: ')
        # pprint(lines)

        # pprint(lines)

        road_lines = []
        line_set = []
        # current_slope = None
        ref_line = {}
        for line in lines:
            if ref_line == {}:
                ref_line = line
                # print()
                # print(f"ref line:            {line}")
                line_set.append(np.hsplit(line["points"], 2))
                continue

            if line["points"][3] == ref_line["points"][3]:
                slope_between_lines = 1000
            else:
                slope_between_lines = (
                    line["points"][2] - ref_line["points"][2]
                ) / (line["points"][3] - ref_line["points"][3])

            ref_gap = abs(slope_between_lines - ref_line["slope"])

            if ((ref_gap > 1.9) or (abs(line["slope"] - ref_line["slope"]) > 0.08)
            ):
                min_set, max_set = np.stack(line_set, axis=1)
                # Find index of maximum value from 2D numpy array
                min_point = np.where(min_set == np.amin(min_set, axis=0)[1])
                max_point = np.where(max_set == np.amax(max_set, axis=0)[1])
                # print(min_point, max_point)

                line_result = np.concatenate(
                    (min_set[min_point[0][0]], max_set[max_point[0][0]])
                )
                # print('line_result: ', line_result)
                road_lines.append(
                    {
                        "points": line_result,
                        "slope": float(
                            (line_result[2] - line_result[0])
                            / (line_result[3] - line_result[1])
                        ),
                        "middle_point": np.asarray(
                            [
                                (line_result[0] + line_result[2]) / 2,
                                (line_result[1] + line_result[3]) / 2,
                            ]
                        ),
                        "distance_square": (line_result[3] - ref_point[1]) ** 2
                        + (line_result[2] - ref_point[0]) ** 2,
                    }
                )
                line_set = []
                # print(f"\nref line:  {ref_gap: 6.2f},   {line}")
            # else:
                # print(f"           {ref_gap: 6.2f},   {line}")

            # print(
            # f"slope: {line['slope']: .02f}    points: {line['points']}   slope between lines: {slope_between_lines: .02f}"
            # )
            ref_line = line
            line_set.append(np.hsplit(line["points"], 2))

        if not (line_set == []):
            # print("merge ended")
            # current_slope = line["slope"]
            ref_line = line
            min_set, max_set = np.stack(line_set, axis=1)
            # Find index of maximum value from 2D numpy array
            # print("min_set, max_set:")
            # print(min_set)
            # print(max_set)
            min_point = np.where(min_set == np.amin(min_set, axis=0)[1])
            max_point = np.where(max_set == np.amax(max_set, axis=0)[1])

            # print(min_point, max_point)

            line_result = np.concatenate(
                (min_set[min_point[0][0]], max_set[max_point[0][0]])
            )
            # print('line_result: ', line_result)
            road_lines.append(
                {
                    "points": line_result,
                    "slope": float(
                        (line_result[2] - line_result[0])
                        / (line_result[3] - line_result[1])
                    ),
                    "middle_point": np.asarray(
                        [
                            (line_result[0] + line_result[2]) / 2,
                            (line_result[1] + line_result[3]) / 2,
                        ]
                    ),
                    "distance_square": (line_result[3] - ref_point[1]) ** 2
                    + (line_result[2] - ref_point[0]) ** 2,
                }
            )
            line_set = []

        # print(f"\nROAD_LINES: {road_lines}\n\n")
        # NOTE: sort by lower point's y value
        road_lines = sorted(road_lines, key=lambda k: k["points"][3], reverse=True)

        # print('road_lines: ')
        # pprint(road_lines)
        # print('horizons: ', horizons)

        # if hough_lines is not None:
        #     for i in range(0, len(hough_lines)):
        #         l = hough_lines[i][0]
        #         cv2.line(self.images['canny'], (l[0], l[1]), (l[2], l[3]),
        #                  (0, 100, 235), 3, cv2.LINE_AA)

        # if hough_lines is not None:
        #     for i in range(0, len(hough_lines)):
        #         l = hough_lines[i][0]
        #         cv2.line(
        #             self.images["front/canny"],
        #             (l[0], l[1]),
        #             (l[2], l[3]),
        #             (0, 100, 235),
        #             1,
        #             cv2.LINE_AA,
        #         )

        # if horizons is not None:
        #     for i in horizons:
        #         cv2.line(images['canny'], (i[0], i[1]), (i[2], i[3]),
        #                  (255, 100, 0), 3, cv2.LINE_AA)

        # # if lines is not None:
        # #     for idx, key in enumerate(lines[:4]):
        # #         cv2.line(images['canny'], (key['points'][0], key['points'][1]), (key['points'][2], key['points'][3]),
        # #                  (255, 255, 30 + 60 * idx), 3, cv2.LINE_AA)
        # #     for idx, key in enumerate(lines[4:]):
        # #         cv2.line(images['canny'], (key['points'][0], key['points'][1]), (key['points'][2], key['points'][3]),
        # #                  (255, 30 + 60 * idx, 255), 3, cv2.LINE_AA)

        # if road_lines is not None:
        #     for idx, key in enumerate(road_lines):
        #         cv2.line(images['canny'], (key['points'][0], key['points'][1]), (key['points'][2], key['points'][3]),
        #                  (255, 30 + 60 * idx, 255), 3, cv2.LINE_AA)
        # self.draw_lines(horizons, self.images["canny2"])
        # self.draw_lines(road_lines, self.images["front/canny"])

        return road_lines

    def get_road_line_by_fish(self):
        """ get line by fishcam """
        adjusted = None
        # NOTE: apply gamma correction and show the images
        # gamma = gamma if gamma > 0 else 0.1
        gamma = 0.2
        self.images["fish"] = self.adjust_gamma(self.images["fish"], gamma=gamma)

        white = np.zeros(shape=[IMAGE_FISH_HEIGHT, IMAGE_FISH_WIDTH, 3], dtype=np.uint8)
        white.fill(255)
        self.mask = cv2.inRange(self.images["fish"], MIN_FISH, MAX_FISH)
        self.images["fish_masked"] = cv2.cvtColor(
            cv2.bitwise_and(white, white, mask=self.mask), cv2.COLOR_HSV2BGR
        )
        self.images["fish_canny"] = cv2.Canny(self.images["fish_masked"], 100, 200, None, 5)
        
        # kernel = np.ones((5, 5), np.uint8)
        # self.images["fish_canny"] = cv2.dilate(self.images['fish_canny'], kernel, iterations=1)

        # self.images["fish_canny"] = cv2.cvtColor(
        #     np.copy(self.images["fish_canny"]), cv2.COLOR_GRAY2BGR
        # )
        # self.images["fish_test"] = cv2.undistort(self.images["fish_canny"], CAMERA_MATRIX, DISTORTION_MATRIX)
        hough_lines = cv2.HoughLinesP(
            self.images["fish_canny"], 1, np.pi / 180, 50, None, 50, 5
        )
        lines = []
        ref_point = [320, 50]

        # rospy.logdebug("\n[EYE] Hough lines: " + str(hough_lines))
        if hough_lines is None:
            return None

        for i in range(0, len(hough_lines)):
            line = hough_lines[i][0]

            # if the line seems "road line", append to lines:
            if line[3] - line[1] is not 0:
                if line[1] > line[3]:
                    line = np.asarray([line[2], line[3], line[0], line[1]])
                
                if line[3] - line[1] == 0:
                    slope = 1000
                else:
                    slope = float("%.2f" % ((line[2] - line[0]) / (line[3] - line[1])))
                lines.append(
                    {
                        "slope": slope,
                        "points": line
                        # "y_intercept": line[1] / (line[0] * slope),
                    }
                )

        lines = sorted(lines, key=lambda k: k["points"][0], reverse=False)
        # lines = sorted(lines, key=itemgetter("slope"))
        # rospy.logdebug("\n[EYE] lines: " + str(lines))

        # road_lines = []
        # line_set = []
        # # current_slope = None
        # ref_line = {}
        # for line in lines:
        #     if ref_line == {}:
        #         ref_line = line
        #         # print()
        #         # print(f"ref line:            {line}")
        #         line_set.append(np.hsplit(line["points"], 2))
        #         continue

        #     if line["points"][3] == ref_line["points"][3]:
        #         slope_between_lines = 1000
        #     else:
        #         slope_between_lines = (
        #             line["points"][2] - ref_line["points"][2]
        #         ) / (line["points"][3] - ref_line["points"][3])

        #     ref_gap = abs(slope_between_lines - ref_line["slope"])

        #     if ((ref_gap > 1.9) or (abs(line["slope"] - ref_line["slope"]) > 0.08)
        #     ):
        #         min_set, max_set = np.stack(line_set, axis=1)
        #         # Find index of maximum value from 2D numpy array
        #         min_point = np.where(min_set == np.amin(min_set, axis=0)[1])
        #         max_point = np.where(max_set == np.amax(max_set, axis=0)[1])
        #         # print(min_point, max_point)

        #         line_result = np.concatenate(
        #             (min_set[min_point[0][0]], max_set[max_point[0][0]])
        #         )
        #         # print('line_result: ', line_result)
        #         road_lines.append(
        #             {
        #                 "points": line_result,
        #                 "slope": float(
        #                     (line_result[2] - line_result[0])
        #                     / (line_result[3] - line_result[1])
        #                 ),
        #                 "middle_point": np.asarray(
        #                     [
        #                         (line_result[0] + line_result[2]) / 2,
        #                         (line_result[1] + line_result[3]) / 2,
        #                     ]
        #                 ),
        #                 "distance_square": (line_result[3] - ref_point[1]) ** 2
        #                 + (line_result[2] - ref_point[0]) ** 2,
        #             }
        #         )
        #         line_set = []

        #     ref_line = line
        #     line_set.append(np.hsplit(line["points"], 2))

        # if not (line_set == []):
        #     ref_line = line
        #     min_set, max_set = np.stack(line_set, axis=1)
        #     min_point = np.where(min_set == np.amin(min_set, axis=0)[1])
        #     max_point = np.where(max_set == np.amax(max_set, axis=0)[1])

        #     line_result = np.concatenate(
        #         (min_set[min_point[0][0]], max_set[max_point[0][0]])
        #     )

        #     road_lines.append(
        #         {
        #             "points": line_result,
        #             "slope": float(
        #                 (line_result[2] - line_result[0])
        #                 / (line_result[3] - line_result[1])
        #             ),
        #             "middle_point": np.asarray(
        #                 [
        #                     (line_result[0] + line_result[2]) / 2,
        #                     (line_result[1] + line_result[3]) / 2,
        #                 ]
        #             ),
        #             "distance_square": (line_result[3] - ref_point[1]) ** 2
        #             + (line_result[2] - ref_point[0]) ** 2,
        #         }
        #     )
        #     line_set = []

        # # NOTE: sort by lower point's y value
        # road_lines = sorted(road_lines, key=lambda k: k["points"][3], reverse=True)

        # rospy.logdebug("\n[EYE] lines:" + str(lines))

        return lines

    def get_road_line_by_sub(self):
        """ get line by fishcam """
        adjusted = None
        # NOTE: apply gamma correction and show the images
        # gamma = gamma if gamma > 0 else 0.1
        gamma = 1.2
        contrast = 100
        brightness = -70
        # sub = self.images["sub"]
        # self.images["sub"] = np.int16(self.images["sub"]) * (contrast/127+1) - contrast + brightness
        # self.images["sub"] = np.uint8(np.clip(self.images["sub"], 0, 255))
        self.images["sub"] = self.adjust_gamma(self.images["sub"], gamma=gamma)
        self.images["sub/original"] = np.copy(self.images["sub"])
        self.images["sub"][60:IMAGE_SUB_HEIGHT-60, 60:IMAGE_SUB_WIDTH-60] = 0 

        white = np.zeros(shape=[IMAGE_SUB_HEIGHT, IMAGE_SUB_WIDTH, 3], dtype=np.uint8)
        white.fill(255)
        if self.check_yellow:
            self.mask = cv2.inRange(self.images["sub"], MIN_SUB_Y, MAX_SUB_Y)
        else:
            self.mask = cv2.inRange(self.images["sub"], MIN_SUB, MAX_SUB)
        self.images["sub/masked"] = cv2.cvtColor(
            cv2.bitwise_and(white, white, mask=self.mask), cv2.COLOR_HSV2BGR
        )
        self.images["sub/canny"] = cv2.Canny(self.images["sub/masked"], 100, 200, None, 5)
        hough_lines = cv2.HoughLinesP(
            self.images["sub/canny"], 1, np.pi / 180, 30, None, 30, 5
        )
        # hough_lines = cv2.HoughLinesP(
        #     self.images["sub/canny"], 1, np.pi / 180, 10, None, 30, 25
        # )
        lines = []
        ref_point = [320, 50]

        # rospy.logdebug("\n[EYE] Hough lines: " + str(hough_lines))
        if hough_lines is None:
            return None

        for i in range(0, len(hough_lines)):
            line = hough_lines[i][0]

            # if the line seems "road line", append to lines:
            if abs(line[3] - line[1]) > 5  and abs(line[2] - line[0]) > 5:
                if line[1] > line[3]:
                    line = np.asarray([line[2], line[3], line[0], line[1]])
                
                if line[3] - line[1] == 0:
                    slope = 1000
                else:
                    slope = float("%.2f" % ((line[2] - line[0]) / (line[3] - line[1])))
                lines.append(
                    {
                        "slope": slope,
                        "points": line
                        # "y_intercept": line[1] / (line[0] * slope),
                    }
                )

        lines = sorted(lines, key=lambda k: k["points"][0], reverse=False)

        # for idx, key in enumerate(lines):
        #     print(idx, key)
        # print()
        # print("deleted:")
        
        idx = 0
        while(idx + 2 < len(lines)):
            posX, posY = lines[idx]["points"][0], lines[idx]["points"][1]
            nextX, nextY = lines[idx + 1]["points"][0], lines[idx + 1]["points"][1]
            nnextX, nnextY = lines[idx + 2]["points"][0], lines[idx + 2]["points"][1]

            if (abs(nextX - posX) < 25) and (abs(nextY - posY) < 25):
                # print(idx, posX, posY)
                lines.pop(idx)
                if idx > 0: idx -= 1
                continue
            elif (abs(nnextX - posX) < 25) and (abs(nnextY - posY) < 25):
                # print(idx, posX, posY)
                lines.pop(idx)
                if idx > 0: idx -= 1
                continue
            
            posX, posY = lines[idx]["points"][2], lines[idx]["points"][3]
            nextX, nextY = lines[idx + 1]["points"][2], lines[idx + 1]["points"][3]
            nnextX, nnextY = lines[idx + 2]["points"][2], lines[idx + 2]["points"][3]

            if (abs(nextX - posX) < 25) and (abs(nextY - posY) < 25):
                # print(idx, posX, posY)
                lines.pop(idx)
                if idx > 0: idx -= 1
                continue
            elif (abs(nnextX - posX) < 25) and (abs(nnextY - posY) < 25):
                # print(idx, posX, posY)
                lines.pop(idx)
                if idx > 0: idx -= 1
                continue

            idx += 1
        
        # operation for checking [N-1]th element
        if idx + 1 < len(lines):
            posX, posY = lines[idx]["points"][0], lines[idx]["points"][1]
            nextX, nextY = lines[idx + 1]["points"][0], lines[idx + 1]["points"][1]

            if (abs(nextX - posX) < 25) and (abs(nextY - posY) < 25):
                # print(idx, posX, posY)
                lines.pop(idx)
            else:
                posX, posY = lines[idx]["points"][2], lines[idx]["points"][3]
                nextX, nextY = lines[idx + 1]["points"][2], lines[idx + 1]["points"][3]
                if (abs(nextX - posX) < 25) and (abs(nextY - posY) < 25):
                    # print(idx, posX, posY)
                    lines.pop(idx)

        return lines
        
    def set_center(self, set_left, set_right):
        if set_right and set_left:
            rospy.Timer(rospy.Duration(0.5), self.reset_state, oneshot=True, reset=False)

            # NOTE: [1] to select column y
            set_right = np.array([key["points"][1] for key in set_right])
            set_left = np.array([key["points"][1] for key in set_left])
            # print(f"set_right: {set_right}")
            center = self.info_front["bias"] + (sum(set_right) / len(set_right)) - (sum(set_left) / len(set_left))
        elif self.threshold_turning > self.info_front["horizon_position"]:
            if set_right:
                center = 1000
            elif set_left:
                center = -1000
            else:
                center = 0
        else:
            center = 0
        
        # if self.has_rospy:
        #     # rospy.logdebug("frontcam center: " + str(center))
        #     pass
        # else:
        #     print("frontcam center: ", str(center))

        self.info_front["center"] = center

    def set_info_to_image(self, image):
        self.put_text(image, self.info_front['turning_to'] + "  "
            + self.info_front['state'] + "   "
            + "horizon: {:.1f}   ".format(self.info_front['horizon_position'])
            + "center: {:.1f}".format(self.info_front["center"])
            , 40, 30)
        # self.put_text(image, f"center: {self.info_front['center']: .2f}   turning_to: {self.info_front['turning_to']}   horizon_pos: {self.info_front['horizon_position']}", 40, 30)

    def set_info_fish_to_image(self, image):
        if self.info_fish['left'] is 0:
            self.put_text(image, "  right: {:d}".format(self.info_fish['right'])
                + "  slope: {:.1f}".format(self.info_fish['slope'])
            , 100, 50)
        elif self.info_fish['right'] is IMAGE_FISH_WIDTH:
            self.put_text(image, "left: {:d}".format(self.info_fish['left'])
                + "  slope: {:.1f}".format(self.info_fish['slope'])
            , 200, 50)

    def adjust_gamma(self, image, gamma=1.0):
        # NOTE: build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values
        invGamma = 1.0 / gamma
        table = np.array(
            [((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]
        ).astype("uint8")

        # NOTE: apply gamma correction using the lookup table
        return cv2.LUT(image, table)

    def get_horizon(self):
        hough_lines = cv2.HoughLinesP(
            cv2.cvtColor(self.images["front/canny"], cv2.COLOR_BGR2GRAY), 1, np.pi / 180, 40, None, 50, 5
        )
        # print('hough_lines:', len(hough_lines))
        lines = []

        if hough_lines is None:
            return None

        for idx, _ in enumerate(hough_lines):
            line = hough_lines[idx][0]
            if abs(line[3] - line[1]) < 3:
                if line[0] > line[2]:
                    line = np.asarray([line[2], line[3], line[0], line[1]])
                lines.append(line)

        lines = sorted(lines, key=lambda k: k[3], reverse=True)
        horizons = []
        line_set = []
        ref_line = []

        for line in lines:
            if len(ref_line) is 0:
                ref_line = line
                line_set.append(line)
                continue
            if abs(ref_line[1] - line[1]) > 20:
                horizons.append(np.array([ref_line[0], ref_line[1], ref_line[2], ref_line[1]]))
                # min_x, max_x = 1000, 0
                line_set = []
            
            ref_line = line
            line_set.append(line)
        
        if line_set:
            horizons.append(np.array([ref_line[0], ref_line[1], ref_line[2], ref_line[1]]))
            line_set = []

        # print(f"horizons: {horizons}")
        return horizons

    def set_horizon_position(self, horizons):
        # Sum of x ot two_points
        pos_x = np.array([val[0] + val[2] / 2 for val in horizons])
        pos_y = np.array([val[1] for val in horizons])
        average_x = sum(pos_x) / len(pos_x)
        average_y = sum(pos_y) / len(pos_y)
        closest_y = horizons[0][1]

        # print(f"average_x: {average_x} average_y: {average_y}")
        if average_x < 100:
            turning_to = "left"
        elif average_x > IMAGE_WIDTH - 100:
            turning_to = "right"
        else:
            turning_to = self.info_front["turning_to"]
            # turning_to = "None"

        self.info_front["horizon_position"] = closest_y
        # self.info_front["horizon_position"] = average_y
        self.info_front["turning_to"] = turning_to

    def show_image(self):
        """ NOTE: Deprecated soon """
        for idx, name in enumerate(self.publishing_names):
            cv2.imshow(name, self.images[name])
            if not self.is_window_set:
                # cv2.moveWindow(name, -650 * ((idx >> 1) % 2 + 1), 550 * (idx % 2))
                cv2.moveWindow(name, 650 * ((idx >> 1) % 2 + 1), 550 * (idx % 2))
        
        self.is_window_set = True
        # cv2.imshow("original", self.images["front/original"])
        # cv2.imshow("result", self.images["front/result"])
        # cv2.imshow("canny", self.images["front/canny"])
        # cv2.imshow("canny_edited", self.images["canny2"])
        # # cv2.imshow('fill', self.images['fill'])
        # cv2.moveWindow("original", -1300, 0)
        # cv2.moveWindow("result", -650, 0)
        # cv2.moveWindow("canny", -1300, 550)
        # cv2.moveWindow("canny_edited", -650, 550)
        # # cv2.moveWindow('fill', -1400, 1100)
        # # cv2.imshow('model', image_opening)
        while 1:
            key = cv2.waitKey(1000)
            if key == 27:
                sys.exit()
            elif key == -1:
                continue
            elif key == 97: # 'a'
                return -1
            else:
                return 1
        # time.sleep(3)
        # cv2.destroyAllWindows()

    def publish_image(self):
        """ publish images by publishing_names """
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        for str in self.publishing_names:
            if not str in self.images: continue
            msg.data = np.array(cv2.imencode('.jpg', self.images[str])[1]).tostring()
            self.publishers[str].publish(msg)

    def put_text(self, image, text, pos_x, pos_y):
        cv2.putText(
            image,
            text,
            (pos_x, pos_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

    def get_flooded(self, image, point):
        # print(f"image.shape: ", image.shape)
        h, w = image.shape
        if point is None:
            seed = (320, 240)
        else:
            seed = (int(point[0]), int(point[1]))
        mask = np.zeros((h + 2, w + 2), np.uint8)
        mask[:] = 0

        floodflags = 4
        floodflags |= cv2.FLOODFILL_MASK_ONLY
        floodflags |= (255 << 8)

        num, im, mask, rect = cv2.floodFill(
            image, mask, seed, (100, 50, 50), (10,) * 3, (10,) * 3, floodflags
        )

        return mask[1:-1, 1:-1]

        # print(mask)
        # white = np.zeros(shape=[IMAGE_HEIGHT, IMAGE_WIDTH, 3], dtype=np.uint8)
        # # white[:] = (255, 255, 255)
        # white.fill(255)

        # mask = cv2.inRange(mask, MIN, MAX)
        # # _masked = cv2.cvtColor(cv2.bitwise_and(
        # # image, image, mask=mask), cv2.COLOR_HSV2BGR)
        # self.images['fill'] = cv2.bitwise_and(
        #     white, white, mask=mask
        # )
        # cv2.imshow('images', flooded)
        # cv2.imwrite("test.png", mask)

    def is_boostable(self, image):
        # TODO: make algorighms if boostable
        return False

    # def set_has_rospy(self, flag):
    #     self.has_rospy = flag


EYE = Eye()
