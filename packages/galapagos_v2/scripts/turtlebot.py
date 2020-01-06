""" This module moves turtlebot"""
#!/usr/bin/env python3

import sys
import rospy
from scheduler import SCHEDULER
from geometry_msgs.msg import Twist
# from std_msgs.msg import Int8
from constants import MAX_SPEED, TURNING_SPEED, SPEED_VALUES, RUN_MODE

NAME = 'runner'


class Turtle(dict):
    """ an object how to run the TurtleBot3 """

    def __init__(self):
        """ __init__
        _angular: the value to set position
        """
        self._enable_running = True
        self._enable_setter = True
        self._enable_boost = False
        self._angular = 0
        self._angular_smooth = 0
        self._speed = 0
        self._speed_smooth = 0
        self._publisher_velocity = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1)
        # self._publisher_stage = rospy.Publisher('/stage', Int8, queue_size=5)
        # rospy.spin()
        self.stop()
        rospy.on_shutdown(self.stop)
        self.exitflag = False
        ######################### Modified by minsoo ##########################
        self.LINE_BASE = 3  # 1 : Left, 2: Right, 3: Both
        self.weight = 0.8  # ! Deprecated
        self.enable_fish = True
        ########################################################################
        dict.__init__(self)

    def stop(self, event=None):
        """ stop the running bot """
        self._angular = 0.
        self._angular_smooth = 0.
        self._speed = 0.
        self._speed_smooth = 0.

        twist = Twist()
        twist.linear.x = 0.
        twist.linear.y = 0.
        twist.linear.z = 0.
        twist.angular.x = 0.
        twist.angular.y = 0.
        twist.angular.z = 0.
        self._publisher_velocity.publish(twist)

        self._enable_setter = True

    def fix_delay(self, event=None):
        """stop if image delayed"""
        if rospy.is_shutdown():
            return

        count = SCHEDULER.request_stopping_motor + 1
        # if SCHEDULER.is_time_over("last_moved_time", 0.03):
        rospy.logdebug("[TURTLE] time delay fixed.")
        twist = Twist()
        twist.linear.x = 0.1 / count
        twist.linear.y = 0.
        twist.linear.z = 0.
        twist.angular.x = 0.
        twist.angular.y = 0.
        twist.angular.z = 0.
        self._publisher_velocity.publish(twist)
        self._enable_running = False
        rospy.Timer(rospy.Duration(0.5 / count), self.release_fix_delay,
                    oneshot=True, reset=True)

    def release_fix_delay(self, event=None):
        self._enable_running = True

    def move(self):
        """ move the bot """
        if not self._enable_running:
            return
        if SCHEDULER.should_pause_motor() > 0:
            print("should_pause_motor: ", SCHEDULER.request_stopping_motor)
            self.fix_delay()
            return

        diff_angular = abs(self._angular - self._angular_smooth)
        if diff_angular > 0.1:
            if self._angular < self._angular_smooth:
                self._angular += 0.1
            elif self._angular > self._angular_smooth:
                self._angular -= 0.1
        elif diff_angular is not 0:
            self._angular = self._angular_smooth
        # else:
        #     SCHEDULER.check_time("speed_smooth", min=0.2)

        diff_speed = abs(self._speed - self._speed_smooth)
        if diff_speed > 0.0002:
            if self._speed < self._speed_smooth:
                self._speed += 0.0002
            elif self._speed > self._speed_smooth:
                self._speed -= 0.0002
        elif diff_speed is not 0:
            self._speed = self._speed_smooth

        if SCHEDULER.debug_option["show_speed_angular"]:
            rospy.logdebug("speed: {:0.2f}  angular: {:0.2f}".format(
                self._speed, self._angular))

        twist = Twist()
        twist.linear.x = self._speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self._angular
        self._publisher_velocity.publish(twist)

        # SCHEDULER.set_time("last_moved_time")
        # rospy.Timer(rospy.Duration(0.2), self.fix_delay, reset=True)

    # def stop_after(self, event):
    #     """ callback function to stop """
    #     self._enable_setter = True
    #     self.set_angular(0)
    #     self.set_speed("stop")
    #     return

    def go_forward(self, duration=1):
        """ force to go straight for a while """
        # if not self._enable_setter:
        #     return

        # rospy.loginfo("[TURTLE] going forward")

        self.set_speed("normal")
        self.set_angular(0)
        self._enable_setter = False
        rospy.Timer(rospy.Duration(duration),
                    self.go_after, oneshot=True, reset=True)
        rospy.sleep(rospy.Duration(duration))
        return

    def go_backward(self, duration=1):
        """ force to go straight for a while """
        if not self._enable_setter:
            return

        rospy.loginfo("\n[TURTLE] going backward")

        self.set_speed("back")
        self.set_angular(0)
        self._enable_setter = False
        rospy.Timer(rospy.Duration(duration),
                    self.go_after, oneshot=True, reset=True)
        rospy.sleep(rospy.Duration(duration + 0.1))
        return

    def go_turn_backward(self, duration=1):
        if not self._enable_setter:
            return

        rospy.loginfo("\n[TURTLE] going backward")

        self.set_speed("back")
        self.set_angular(-3)
        self._enable_setter = False
        rospy.Timer(rospy.Duration(duration),
                    self.go_after, oneshot=True, reset=True)
        rospy.sleep(rospy.Duration(duration + 0.1))
        return

    def go_after(self, event):
        """ callback function after going forward """
        self._enable_setter = True
        self.set_angular(0)
        self.set_speed("stop")
        return

    def go_turn(self, direction, duration=1.9, angular=0.9, speed=0.09):
        # if not self._enable_setter:
            # return
        self._enable_setter = False
        if direction == "right":
            self._angular = self._angular_smooth = -angular
        else:
            self._angular = self._angular_smooth = angular

        self._speed = self._speed_smooth = speed

        rospy.Timer(rospy.Duration(duration),
                    self.go_after, oneshot=True, reset=True)
        rospy.sleep(rospy.Duration(duration))

    def turn(self, duration, degree=1, consuming_time=None):
        """ turn for line-tracing """
        if not self._enable_setter:
            return
        self._angular = degree
        self._angular_smooth = 0
        self._enable_setter = False
        rospy.Timer(rospy.Duration(duration),
                    self.turn_after, oneshot=True, reset=True)
        if consuming_time:
            rospy.sleep(rospy.Duration(consuming_time))

    # NOTE: USED in construction
    def _turn(self, direction, radius):
        """ turn to direction with radius by duration """
        if not self._enable_setter:
            return

        rospy.loginfo("\n[TURTLE] turning...")

        if direction == 'right':
            degree = -1.7 / radius
            # degree = -1.7
        else:
            degree = 1.7 / radius
            # degree = 1.7

        self._speed = self._speed_smooth = TURNING_SPEED
        self.set_angular_smooth(degree)
        self._enable_setter = False
        rospy.Timer(rospy.Duration(1.1), self.turn_after)

    def turn_after(self, event):
        """ callback function after turning """
        if SCHEDULER.debug_option["show_turtle_callback"]:
            rospy.logdebug("\n[TURTLE] turning ended: " + str(event))
        self._enable_setter = True
        # self.set_angular(0)
        # self.set_speed("stop")
        # rospy.signal_shutdown("[TURTLE] shutting down...")
        return

    def turn_by_degree(self, degree, radius):
        """ turn by degree with radius """
        return

    def boost(self, amount='fast'):
        TURTLE.set_speed(amount)
        # TURTLE.set_speed_smooth("normal")
        return

    def increase_speed(self):
        self._speed += 0.1

    def decrease_speed(self):
        self._speed -= 0.1

    def disable(self):
        self.stop()
        self._enable_running = False

    def enable(self):
        self._enable_running = True

    # * setting functions

    def set_speed_by_percentage(self, percentage):
        """ set the speed by percentage """
        self._speed = MAX_SPEED * percentage
        # self.move()

    def set_weight(self, weight):
        if not self._enable_setter:
            return
        self.weight = weight

    def set_speed(self, speed_str):
        """ set the speed by string """
        if not self._enable_setter:
            return
        self._speed = self._speed_smooth = MAX_SPEED * SPEED_VALUES[speed_str]

    def set_speed_smooth(self, speed_str):
        """ set the speed_smooth value """
        if not self._enable_setter:
            return
        self._speed_smooth = MAX_SPEED * SPEED_VALUES[speed_str]

    def set_angular(self, angular):
        """ set the angular value """
        if not self._enable_setter:
            return
        self._angular = self._angular_smooth = angular
        # self.move()

    def set_angular_smooth(self, angular):
        """ set the angular_smooth value """
        if not self._enable_setter:
            return
        self._angular_smooth = angular

    # * getting functions

    def get_speed(self):
        """ get the speed value """
        return self._speed

    def get_angular(self):
        """ get the angular value """
        return self._angular

    def is_settable(self):
        # ! deprecated: will be changed to is_occupied()
        return self._enable_setter

    def is_occupied(self):
        return not self._enable_setter

    def get_info(self):
        """ get some information of turtlebot """
        return {
            'angluar': self._angular,
            'linear': self._speed,
        }


# rospy.loginfo("\n[TURTLE] node initialized with NAME: ", NAME + '_' + sys.argv[1])
# rospy.init_node(NAME + '_' + sys.argv[1], anonymous=True)
TURTLE = Turtle()
