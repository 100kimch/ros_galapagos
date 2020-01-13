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
            "show_roi_detecting": False,
            "show_blob_detecting": False,
            "show_image_matching": False,
            "show_orb_matching": False,
            "show_construction_lidar": True,
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
                "lidar": True
            },
            "construction": {
                "frontcam": False,
                "subcam": False,
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
            },
            "tunnel": {
                "frontcam": False,
                "subcam": False,
                "lidar": True
            }
        }
        # self.path = sys.path[0]
        self.path = os.path.split(sys.path[0])[0]
        self.is_subcam_delaying = False
        self.is_lidar_delaying = False
        self.is_occupied = {
            "subcam": False,
            "fishcam": False,
            "frontcam": False,
            "lidar": False
        }
        self.request_stopping_motor = 0
        self._time = {
            "frontcam": timeit.default_timer(),
            "subcam": timeit.default_timer(),
            "lidar": timeit.default_timer(),
            "publisher": timeit.default_timer(),
            "speed_smooth": timeit.default_timer(),
            "last_moved_time": 0,
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

    def is_subcam_occupied(self):
        return self.is_occupied["subcam"]

    def is_fishcam_occupied(self):
        return self.is_occupied["fishcam"]

    def is_frontcam_occupied(self):
        return self.is_occupied["frontcam"]

    def is_lidar_occupied(self):
        return self.is_occupied["lidar"]

    def release_subcam_occupied(self, event=None):
        self.is_occupied["subcam"] = False

    def release_fishcam_occupied(self, event=None):
        self.is_occupied["fishcam"] = False

    def release_frontcam_occupied(self, event=None):
        self.is_occupied["frontcam"] = False

    def release_lidar_occupied(self, event=None):
        self.is_occupied["lidar"] = False

    def publish_state(self, msg):
        """ publish states one of them below:
            @msg: string to send state
            "stop": do nothing
            "view": view sensor data only
            "run": view sensor data & run turtlebot
        """
        MSG = String()
        MSG.data = msg
        self._publisher_state_manager.publisher(MSG)

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

    def check_time(self, timer_name="TIME", min=0, stop_when_delay=False):
        now = timeit.default_timer()
        duration = now - self._time[timer_name]
        self._time[timer_name] = now

        if stop_when_delay:
            # TODO: find a way to merge these Timer events
            # NOTE: couldn't merge due to callback function cannot get arguments
            if timer_name is "lidar":
                rospy.Timer(rospy.Duration(min),
                            self.warn_lidar_delay, oneshot=True, reset=True)
                rospy.Timer(rospy.Duration(min + 0.3),
                            self.warn_lidar_delay, oneshot=True, reset=True)
            elif timer_name is "subcam":
                rospy.Timer(rospy.Duration(min),
                            self.warn_subcam_delay, oneshot=True, reset=True)
                rospy.Timer(rospy.Duration(min + 0.3),
                            self.warn_subcam_delay, oneshot=True, reset=True)

        else:
            if (duration > min + 0.03):
                rospy.logerr(
                    "[SCH] {:10s} checked at {:.02f}\r".format(timer_name, duration))
                if stop_when_delay:
                    if timer_name is "subcam":
                        self.is_subcam_delaying = True
                    elif timer_name is "lidar":
                        self.is_lidar_delaying = True
                # self.request_stopping_motor += 1
            elif (duration > min):
                rospy.logwarn(
                    "[SCH] {:10s} checked at {:.02f}\r".format(timer_name, duration))
                if stop_when_delay:
                    if timer_name is "subcam":
                        self.is_subcam_delaying = False
                    elif timer_name is "lidar":
                        self.is_lidar_delaying = False
            else:
                if stop_when_delay:
                    if timer_name is "subcam":
                        self.is_subcam_delaying = False
                    elif timer_name is "lidar":
                        self.is_lidar_delaying = False

    def warn_lidar_delay(self, event=None):
        timer_name = "lidar"
        duration = timeit.default_timer() - self._time["subcam"]
        if (duration > 0.3):
            rospy.logerr(
                "[SCH] {:10s} checked at {:.02f}\r".format(timer_name, duration))
            self.is_lidar_delaying = True
            self.request_stopping_motor += 2
        elif (duration > 0.1):
            rospy.logwarn(
                "[SCH] {:10s} checked at {:.02f}\r".format(timer_name, duration))
            self.is_lidar_delaying = False
        else:
            self.is_lidar_delaying = False

    def warn_subcam_delay(self, event=None):
        timer_name = "subcam"
        duration = timeit.default_timer() - self._time["subcam"]
        if (duration > 0.03):
            rospy.logerr(
                "[SCH] {:10s} checked at {:.02f}\r".format(timer_name, duration))
            self.is_subcam_delaying = True
            if self._state != "construction":
                self.request_stopping_motor += 1
        elif (duration > 0.01):
            rospy.logwarn(
                "[SCH] {:10s} checked at {:.02f}\r".format(timer_name, duration))
            self.is_subcam_delaying = False
        else:
            self.is_subcam_delaying = False

    def should_pause_motor(self):
        if self.request_stopping_motor > 0:
            print(SCHEDULER.request_stopping_motor)
            self.request_stopping_motor -= 1
            return True
        return False


SCHEDULER = Scheduler()
