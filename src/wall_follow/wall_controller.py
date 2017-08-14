#!/usr/bin/python

import rospy
import numpy as np
import time
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from std_msgs.msg import String

class wallController:
	def __init__(self):
		
		rospy.Subscriber("/scan", LaserScan, self.wall_controller)
		rospy.Subscriber("/which_wall", String, self.choose_side)
		
                self.cmd_pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)
		
		
		# Desired distance from the wall
		self.d_des = 1
		self.speed_des = 2 #2

		# Controller gains
		self.Kp = .18  #.5 #.7
		#self.Kd = 0

		# Error variables
		self.error = 0
		#self.error_rate = 0

		# Time variables
		#self.old_time = time.time()
		#self.old_error = 0

		# Right(0) or left(1)
		self.wall_orientation = "left"

		self.u_steer = 0
	
	def choose_side(self, side):
            self.wall_orientation = side
            # print ("wall = ", self.wall_orientation)
	
	def wall_controller(self, scan):
 		smallestDistance = 70
                
                print (self.wall_orientation)
                print ("wall = right", self.wall_orientation == "right")
                if self.wall_orientation == String("right"):
		    print ("go right")
                    a = 172
                    b = 300
                else:  # left side
                    a = 780
		    b = 908
                    print ("go left")

		for i in range(a, b):
			if scan.ranges[i] < smallestDistance:
				smallestDistance = scan.ranges[i]
 				smallestIndex = i
		self.error = self.d_des - smallestDistance

	# Derivative

		#new_time = time.time()
		#new_error = self.error
		#delta_time = new_time - self.old_time
		#delta_error = new_error - self.old_error
		#self.error_rate = delta_error/delta_time
		#self.old_time = time.time()
		#self.old_error = self.error
	# Send error values to position method
		self.run_position_controller(self.error)
		#self.run_position_controller(self.error)

	def run_position_controller(self, error):
		p_control = self.Kp * self.error
		#d_control = self.Kd * self.error_rate

		saturation = 0.3
                if self.wall_orientation == String("right"):
                        self.u_steer = p_control
                else:
                        self.u_steer = -p_control

		if self.u_steer > saturation:
			self.u_steer = saturation
		elif self.u_steer < -saturation:
			self.u_steer = -saturation
		
                u_steer_msg = AckermannDriveStamped()
		u_steer_msg.drive.steering_angle = self.u_steer
		u_steer_msg.drive.speed = self.speed_des
		u_steer_msg.header.stamp = rospy.Time.now()
		self.cmd_pub.publish(u_steer_msg)

if __name__ == "__main__":
    rospy.init_node("wall_controller")
    node = wallController()
    rospy.spin()

