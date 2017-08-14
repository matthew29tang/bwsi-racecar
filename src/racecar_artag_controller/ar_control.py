#!/usr/bin/python

# Note: [x,y] plane parallel to ground

import rospy
import numpy as np
import time
import cv2

from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

class arController:
    def __init__(self):
	# Subscribe to detected AR tag topic
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.navigate_to_ar)

	# Publish commands to ackermann mux
	self.cmd_pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

	# Desired constants Note
	self.speed_cap = 0.7
	self.K = 3
	self.id_des = 0
	self.dist_des = 0.2

    def navigate_to_ar(self, msg):
	markers = msg.markers
	x = 0
	y = 0
	speed = 0
	steering_angle = 0

	# Find specific tag, get center point of AR tag
	for x in markers:
		if x.id == self.id_des:
			print 'detected: {}'.format(self.id_des)
			ar_tag = x
			ar_tag_pos = ar_tag.pose.pose.position
			x = ar_tag_pos.x
			y = ar_tag_pos.z
        		speed = y - self.dist_des
			steering_angle = -np.arctan2((x - 0.005), y)
        		self.calculate_turn(speed, steering_angle)

    def calculate_turn(self, speed, steering_angle):
	u_steer = steering_angle
	saturation = 0.34
        if abs(u_steer) > saturation:
            u_steer = np.sign(u_steer)*saturation

	z = self.K*speed
        if abs(z) > self.speed_cap:
                z = np.sign(z)* self.speed_cap

        cmd_msg = AckermannDriveStamped()
       	cmd_msg.drive.steering_angle = u_steer
	cmd_msg.drive.speed = z
        cmd_msg.header.stamp = rospy.Time.now()

        self.cmd_pub.publish(cmd_msg)

if __name__ == "__main__":
    rospy.init_node("ar_control")
    node = arController()
    rospy.spin()
