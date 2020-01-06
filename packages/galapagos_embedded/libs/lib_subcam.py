
#!/usr/bin/env python3
# Minimized package from lib_eye.py

import rospy
from scheduler import SCHEDULER

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header, String
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
MIN_SUB = np.array([int(0 * 255 / 360), int(0 * 255 / 100), int(0 * 255 / 100)])
MAX_SUB = np.array([int(360 * 255 / 360), int(70 * 255 / 100), int(100 * 255 / 100)])
# MAX_SUB = np.array([int(360 * 255 / 360), int(14 * 255 / 100), int(100 * 255 / 100)])

CAMERA_MATRIX = np.array(ARRAY_K)
# DISTORTION_MATRIX = np.array([ -0.335978, 0.133403, 0.000953, 0.000789 ])
DISTORTION_MATRIX = np.array([ -0.335978, 0.133403, 0.000953, -0.005089 ])

IMAGE_SUB_WIDTH = 640
IMAGE_SUB_HEIGHT = 480

class Eye(dict):
    """ an object to process image """

    def __init__(self):
        # self.has_rospy = True # ! deprecated
        self.images = {}
        self.mask = None
        self.front_occupied = False
        self.fish_occupied = False
        self.sub_occupied = False
        self.is_window_set = False
        self.info_front = {
            "state": "straight",
            "center": 0,
            "bias": 0,
            "turning_to": "None",
            "horizon_position": 0.
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

        # if SCHEDULER.debug_mode:
        #     self.publishing_names = ["sub/original", "sub/canny"]
        # else:
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
        self.threshold_sub_slope = -1.78
        self.threshold_sub_center = 200
        
        rospy.logdebug("[LIB_EYE] threshold_turning set to " + str(self.threshold_turning))
        # rospy.logdebug("[LIB_EYE] threshold_tracking set to " + str(self.threshold_tracking))
        rospy.logdebug("[LIB_EYE] threshold_sub_slope set to " + str(self.threshold_sub_slope))
        rospy.logdebug("[LIB_EYE] threshold_sub_center set to " + str(self.threshold_sub_center))

    def reset_state(self, event=None):
        self.info_front["state"] = "straight"
        self.info_front["turning_to"] = "None"

    def see_sub(self, compressed_data=None):
        """ callback handler when sub image received """
        self.sub_occupied = True
        self.set_sub_image(compressed_data)

        road_lines = self.get_road_line_by_sub()
        # if road_lines is not None and len(road_lines) is not 4:
        #     if SCHEDULER.debug_option["warn_road_lines"]:
        #         rospy.logwarn("[LIB_EYE] road_lines: ")
        #         pprint(road_lines)
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
            # self.draw_lines(line_center, self.images["sub/canny"], color="blue")
        # self.draw_lines(road_lines, self.images["sub/canny"], color="blue")
        # self.set_info_fish_to_image(self.images["sub/canny"])

        self.publish_image()
        return self.info_sub

    def release_sub_occupied(self, event=None):
        """ release controlling grant """
        self.sub_occupied = False

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
        self.mask = cv2.inRange(self.images["sub"], MIN_SUB, MAX_SUB)
        self.images["sub_masked"] = cv2.cvtColor(
            cv2.bitwise_and(white, white, mask=self.mask), cv2.COLOR_HSV2BGR
        )
        self.images["sub/canny"] = cv2.Canny(self.images["sub_masked"], 100, 200, None, 5)
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

    def is_sub_occupied(self):
        return self.sub_occupied

EYE = Eye()
