#!/usr/bin/python

import rospy 
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge
from ackermann_msgs.msg import AckermannDriveStamped

class lineFollowNode:
    def __init__(self):

        ###########################################
        # TUNABLE_Kp = float
        self.TUNABLE_KP = .0018

        # TUNABLE_Kd = float
        self.TUNABLE_KD = .01

        # TUNABLE_Cap_Speed = float
        self.SPEED = 1.0
        ###########################################

        rospy.Subscriber("/image_echo", Image, self.follow_line)
        rospy.Subscriber("/contour_rectangle", Quaternion, self.get_rec)
        self.drive_cmd = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)
        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0
        self.des_speed = self.SPEED
        #self.a = 0
        #self.mag_error = 0
        self.error = 0
        self.error_rate = 0
        self.center_offset = 25

        self.Kp = self.TUNABLE_KP
        self.Kd = self.TUNABLE_KD

        self.old_error = 0
        self.bridge = CvBridge()


    def follow_line(self, stream):
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

        u_steer = p_control + d_control

        steer_saturation = 0.34

        if u_steer > 0.34:
            u_steer = 0.34
        elif u_steer < -0.34:
            u_steer = -0.34

        speed = self.des_speed

        u_msg = AckermannDriveStamped()
        u_msg.drive.steering_angle = u_steer
        u_msg.drive.speed = speed
        u_msg.header.stamp = rospy.Time.now()
        self.drive_cmd.publish(u_msg)

    def get_rec(self, rec):
        self.x = rec.x
        self.y = rec.y
        self.w = rec.z
        self.h = rec.w

if __name__ == "__main__":
    rospy.init_node("line_follow_controller")
    node = lineFollowNode()
    rospy.spin()

