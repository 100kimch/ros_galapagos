""" This module makes turtlebot"""
#!/usr/bin/env python3

import sys
import rospy
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
        self._angular = 0
        self._angular_smooth = 0
        self._speed = 0
        self._speed_smooth = 0
        self._publisher_velocity = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=5)
        # self._publisher_stage = rospy.Publisher('/stage', Int8, queue_size=5)
        # rospy.spin()
        self.stop()
        rospy.on_shutdown(self.stop)
        self.exitflag = False
        ######################### Modified by minsoo ##########################
        self.LINE_BASE = 3          ## 1 : Left, 2: Right, 3: Both
        self.weight = 0.8
        self.enable_fish = True
        ########################################################################
        dict.__init__(self)

    def stop(self):
        """ stop the running bot """
        self._angular = 0.
        self._angular_smooth = 0.
        self._speed = 0.

        twist = Twist()
        twist.linear.x = 0.
        twist.linear.y = 0.
        twist.linear.z = 0.
        twist.angular.x = 0.
        twist.angular.y = 0.
        twist.angular.z = 0.
        self._publisher_velocity.publish(twist)

    def move(self):
        """ move the bot """
        if not self._enable_running:
            return

        diff_angular = abs(self._angular - self._angular_smooth)
        if diff_angular > 0.1:
            if self._angular < self._angular_smooth:
                self._angular += 0.1
            elif self._angular > self._angular_smooth:
                self._angular -= 0.1
        elif diff_angular is not 0:
            self._angular = self._angular_smooth

        diff_speed = abs(self._speed - self._speed_smooth)
        if diff_speed > 0.05:
            if self._speed < self._speed_smooth:
                self._speed += 0.05
            elif self._speed > self._speed_smooth:
                self._speed -= 0.05
        elif diff_speed is not 0:
            self._speed = self._speed_smooth

        # print('move speed: ' + str(self._speed) +
        #       ' angular: ' + str(self._angular))
        twist = Twist()
        twist.linear.x = self.weight*self._speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z  = self.weight*self._angular
        self._publisher_velocity.publish(twist)

    def stop_after(self, event):
        """ callback function to stop """
        self._enable_setter = True
        self.set_angular(0)
        self.set_speed("stop")
        return
    
    def go_forward(self, duration=1):
        """ force to go straight for a while """
        if not self._enable_setter:
            return

        rospy.loginfo("\n[TURTLE] going forward")

        self.set_speed("normal")
        self.set_angular(0)
        self._enable_setter = False
        # rospy.loginfo("\n[TURTLE] sleeping")
        # self.set_angular(0)
        # self.set_speed("stop")
        # rospy.loginfo("\n[TURTLE] slept")
        self._enable_setter = True
        rospy.Timer(rospy.Duration(duration), self.go_forward_after, oneshot=True, reset=True)
        return

    def go_forward_after(self, event):
        """ callback function after going forward """
        self._enable_setter = True
        self.set_angular(0)
        self.set_speed("stop")
        # rospy.signal_shutdown("[TURTLE] shutting down...")
        return

    def turn(self, direction, duration, with_stop=False, starting_speed=None):
        """ turn to direction with radius by duration """
        if not self._enable_setter:
            return

        if direction == 'right':
            degree = -1.35
            # degree = -0.38 # when TURNING_SPEED = 0.03
        else:
            degree = 1.35
            # degree = 0.38 # when TURNING_SPEED = 0.03

        if starting_speed is not None:
            self._speed = self._speed_smooth = 0.12
            degree = degree * 2
        elif with_stop:
            self._speed = self._speed_smooth = 0
        else:
            self._speed = self._speed_smooth = TURNING_SPEED
        self.set_angular_smooth(degree)
        self._enable_setter = False
        rospy.Timer(rospy.Duration(duration), self.turn_after, oneshot=True, reset=True)
        # rospy.Timer(rospy.Duration(4.5), self.turn_after)

        return

    def change_line(self, direction, duration):
        """ turn to direction with radius by duration """
        if not self._enable_setter:
            return
        self._enable_setter = False

        if direction == 'right':
            degree = -2.35
        else:
            degree = 2.35

        # self._speed = self._speed_smooth = 0
        print("sleep")
        self._speed = self._speed_smooth = 0
        self._angular = self._angular_smooth = degree

        # rospy.sleep(2)
        # print("slept")
        # self._speed = self._speed_smooth = 0.22
        # rospy.sleep(2)
        # self._speed = self._speed_smooth = 0.1
        # self._angular = -degree
        # self._angular_smooth = 0
        # rospy.sleep(2)
        # self.set_angular_smooth(0.0)
        self.direction = "hi"
        rospy.Timer(rospy.Duration(0.3), self.change_line2, oneshot=True, reset=True)
        rospy.Timer(rospy.Duration(1.0), self.change_line3, oneshot=True, reset=True)
        rospy.Timer(rospy.Duration(1.3), self.turn_after, oneshot=True, reset=True)
        # rospy.sleep(2)
        # self.go_forward(duration)
        # # rospy.sleep(duration)
        # self.set_angular(-degree)
        # self.set_angular_smooth(0.0)
        # # rospy.sleep(2)

        # self._enable_setter = True
        # # rospy.Timer(rospy.Duration(duration), self.turn_after, oneshot=True, reset=True)
        # # rospy.Timer(rospy.Duration(4.5), self.turn_after)
        return
    def change_line2(self, event=None):
        self._speed = self._speed_smooth = 0.22
        self._angular = self._angular_smooth = 0

    def change_line3(self, event=None):
        print("direction: ", str(self.direction))
        self._speed = self._speed_smooth = 0
        self._angular = self._angular_smooth = -2.35

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
        rospy.loginfo("\n[TURTLE] turning ended: " + str(event))
        self._enable_setter = True
        self.set_angular(0)
        self.set_speed("stop")
        # rospy.signal_shutdown("[TURTLE] shutting down...")
        return

    def turn_by_degree(self, degree, radius):
        """ turn by degree with radius """
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
        return self._enable_setter
    
    def get_info(self):
        """ get some information of turtlebot """
        return {
            'angluar': self._angular,
            'linear': self._speed,
        }


rospy.loginfo("\n[TURTLE] node initialized with NAME: ", NAME + '_' + sys.argv[1])
rospy.init_node(NAME + '_' + sys.argv[1], anonymous=True)
TURTLE = Turtle()
