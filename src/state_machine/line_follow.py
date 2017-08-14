#!/usr/bin/python

import rospy
import numpy as np
import cv2
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion, Point
from cv_bridge import CvBridge
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from tuning_vars import TUNING_VARS

class LineFollowNode:
    def __init__(self):

        ###########################################
        # TUNABLE_Kp = float
        self.TUNABLE_KP = TUNING_VARS.line_follow_KP
	#.0018

        # TUNABLE_Kd = float
        self.TUNABLE_KD = TUNING_VARS.line_follow_KD
	#0.01

        # TUNABLE_Cap_Speed = float
        self.SPEED = TUNING_VARS.line_follow_SPEED
	#1.0
        ###########################################

        self.centerSub = rospy.Subscriber("cnt_center",
                                           Point,
                                           self.getCenter)
        self.imgSub = rospy.Subscriber("/image_echo",
                                        Image,
                                        self.follow_line)
        self.yellowFoundSub = rospy.Subscriber("/found_yellow",
                                               Bool,
                                               self.foundYellow)

#        self.recSub = rospy.Subscriber("/contour_rectangle",
#                                        Quaternion,
#                                        self.get_rec)

        self.drive_cmd = rospy.Publisher("/ackermann_cmd_mux/input/navigation",
                                          AckermannDriveStamped,
                                          queue_size=10)

        self.foundYellow = False

        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0
        self.des_speed = self.SPEED
        #self.a = 0
        #self.mag_error = 0
        self.error = 0
        self.error_rate = 0
        self.center_offset = 25 # zed cam is to left of center

        self.Kp = self.TUNABLE_KP
        self.Kd = self.TUNABLE_KD

        self.old_error = 0
        self.bridge = CvBridge()

    def foundYellow(self, found):
        self.foundYellow = found.data

    def unsub_nodes(self):
        self.imgSub.unregister()
 #       self.recSub.unregister()
        self.centerSub.unregister()
        self.yellowFoundSub.unregister()

    def follow_line(self, stream):
#        print ("Following yellow")

        image = self.bridge.imgmsg_to_cv2(stream, "bgr8")

# Proportional controller
        shape = image.shape
        width = shape[1]
        center = width/2 + self.center_offset
        self.error = center - (self.x+(self.w/2))

        #if self.a < 0:
            #self.a = -1
        #elif self.a > 0:
            #self.a = 1
        #if self.w < 65:
            #self.w = 65
        #self.mag_error = self.w - 65
        #print self.mag_error

    # Derivative
        new_error = self.error
        delta_error = new_error - self.old_error
        self.old_error = self.error
        self.error_rate = delta_error

        self.run_park_command(self.error, self.error_rate)

        ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")

    def run_park_command(self, error, error_rate):
        p_control = self.Kp*error
        d_control = self.Kd*error_rate

        u_steer = (p_control + d_control)

        steer_saturation = 0.34

        if u_steer > 0.34:
            u_steer = 0.34
        elif u_steer < -0.34:
            u_steer = -0.34

        speed = self.des_speed
	#if abs(u_steer) > .1:
        speed -= TUNING_VARS.line_follow_TURNING_SLOW_FACTOR*abs(u_steer)
        if not self.foundYellow:
            speed = 0

        u_msg = AckermannDriveStamped()
        u_msg.drive.steering_angle = u_steer
        u_msg.drive.speed = speed
        u_msg.header.stamp = rospy.Time.now()
        self.drive_cmd.publish(u_msg)

    def getCenter(self, center):
        self.x = center.x
        self.y = center.y

    def get_rec(self, rec):
        self.x = rec.x
        self.y = rec.y
        self.w = rec.z
        self.h = rec.w

if __name__ == "__main__":
    rospy.init_node("line_follow_controller")
    node = LineFollowNode()
    rospy.spin()

