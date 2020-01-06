""" This module controls state such as debug mode and mission type. """
#!/usr/bin/env python3

import sys
import os
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import timeit


class Scheduler(dict):
    """ a class to contol state such as debug mode and mission type. """

    def __init__(self):
        """ initializer """
        self.debug_mode = False
        # self.enable_timer = False
        self.debug_option = {
            "show_timer": True,
            "show_front_info": False,
            "show_speed_angular": False,
            "show_center_slope": False,
            "warn_road_lines": False,
            "show_turtle_callback": False,
            "show_loaded_ref_images": False,
            "show_blob_detecting": False,
            "show_image_matching": False,
            "show_orb_matching": False,
            "show_parking_lidar": True,
        }
        self.is_enable = {
            "frontcam": True,
            "subcam": True,
            "lidar": False
        }
        self._state = "stop"
        self._state_option = {
            "default": {
                "frontcam": True,
                "subcam": True,
                "lidar": True
            },
            "traffic_light": {
                "frontcam": True,
                "subcam": False,
                "lidar": False
            },
            "to_intersection": {
                "frontcam": True,
                "subcam": True,
                "lidar": False
            },
            "intersection_right": {
                "frontcam": True,
                "subcam": True,
                "lidar": False
            },
            "intersection_left": {
                "frontcam": True,
                "subcam": True,
                "lidar": False
            },
            "to_construction": {
                "frontcam": True,
                "subcam": True,
                "lidar": False
            },
            "construction": {
                "frontcam": False,
                "subcam": True,
                "lidar": True
            },
            "parking": {
                "frontcam": True,
                "subcam": True,
                "lidar": True
            },
            "zigzag": {
                "frontcam": True,
                "subcam": True,
                "lidar": False
            }
        }
        # self.path = sys.path[0]
        self.path = os.path.split(sys.path[0])[0]
        self._time = {
            "frontcam": timeit.default_timer(),
            "subcam": timeit.default_timer(),
            "publisher": timeit.default_timer(),
            "speed_smooth": timeit.default_timer()
        }
        self._publisher_state_manager = rospy.Publisher(
            '/state', String, queue_size=1
        )
        # rospy.loginfo("[SCH] Scheduler initialized.")

    def enable_lidar(self, event=None):
        self.is_enable["lidar"] = True
        rospy.loginfo("[SCH] lidar enabled.")

    def enable_cams(self, event=None):
        self.is_enable["frontcam"] = True
        self.is_enable["subcam"] = True

    def disable_cams(self, event=None):
        self.is_enable["frontcam"] = False
        self.is_enable["subcam"] = False

    def is_lidar_enable(self):
        return self.is_enable["lidar"]

    def is_subcam_enable(self):
        return self.is_enable["subcam"]

    def is_frontcam_enable(self):
        return self.is_enable["frontcam"]

    def publish_state(self, msg):
        """ publish states one of them below:
            @msg: string to send state
            "stop": do nothing
            "view": view sensor data only
            "run": view sensor data & run turtlebot
        """
        MSG = String()
        MSG.data = msg
        self._publisher_state_manager.publish(MSG)

    def load_module(self, module):
        # module_path = "mypackage.%s" % module
        module_path = module

        if module_path in sys.modules:
            return sys.modules[module_path]

        return __import__(module_path, fromlist=[module])

    def import_modules(self):
        sys.path.append(self.path + '/runners/')
        sys.path.append(self.path + '/libs/')

    def print_state(self):
        print("====================================")
        rospy.loginfo("  State: " + str(self._state))
        print("====================================")

    def set_state(self, str):
        self._state = str
        self.is_enable = self._state_option[str]
        if self.debug_mode:
            self.print_state()
        return

    def get_state(self):
        return self._state

    def check_time(self, msg_title="TIME", min=0):
        now = timeit.default_timer()
        duration = now - self._time[msg_title]
        if (duration > min + 0.1):
            rospy.logerr(
                "[SCH] {:10s} checked at {:.02f}\r".format(msg_title, duration))
        elif (duration > min):
            rospy.logwarn(
                "[SCH] {:10s} checked at {:.02f}\r".format(msg_title, duration))
        self._time[msg_title] = now


SCHEDULER = Scheduler()
