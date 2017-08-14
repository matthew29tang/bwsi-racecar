#!/usr/bin/python
import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class SafetyControllerNode:
	def __init__(self):
		# Subscribe to drive command and laser scan topics
		rospy.Subscriber("/scan", LaserScan, self.laser_scan_input_callback)
		rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, self.ackermann_cmd_input_callback)
		rospy.Subscriber("/safety_on", Bool, self.safety_callback)

		# Publish safe drive commands to final drive command topic
		self.cmd_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size = 10)

		# Variables
		self.safe = 2 # 0: safe, 1: not safe, 2: scanning
		self.speed = 1
		self.curr_steering_angle = 0
		self.subscriber_safe = True
		self.count = 0

		self.CAR_W = 0.45
		self.FRONT_DIST = .5

	def safety_callback(self, state):
			self.subscriber_safe = state.data

	def laser_scan_input_callback(self, scan):
		if self.subscriber_safe:
			# Method variables and calculations
			dists = scan.ranges
			semi_range = len(dists)*2/3
			mid_ind = len(dists)/2
			low_bound = mid_ind - semi_range/2
			SCAN_PARTITIONS = 8
			# Keep even to avoid cancelation of vectors in center partition
			up_bound = mid_ind + semi_range/2
			partition_width = semi_range/SCAN_PARTITIONS

			for i in range(0,SCAN_PARTITIONS):
				x_sum = 0
				y_sum = 0
				first_ind = low_bound + i*partition_width
				for j in range (0, partition_width):
					theta = (j + i*partition_width)*scan.angle_increment
					x_sum += dists[first_ind+j]*np.cos(theta)
					y_sum += dists[first_ind+j]*np.sin(theta)
				x_avg = x_sum/partition_width
				y_avg = y_sum/partition_width
				self.safe = self.in_range_for_steer_angle(x_avg, y_avg)
				if i == 7 and self.safe == 2:
					self.safe = 0
				if self.safe == 1:
					break
		else:
			print 'safety off'

	def in_range_for_steer_angle(self, x, y):
		if abs(self.curr_steering_angle) <= 0.2: # If car going straight
			x_range = 0.25
			y_range = self.FRONT_DIST + 0.5*self.speed
			#print y_range
			if abs(x) < x_range and abs(y) < y_range:
				return 1
			else:
				return 2

		elif abs(self.curr_steering_angle) >= 0.2: # If car turning left
			z = np.tan(self.curr_steering_angle)
			r = self.CAR_W/z
			q = np.sign(r)
			return_num = 0
			scan_point = ((x + q*r)**2)+((y + self.CAR_W)**2)

                        # scan_point = (x-r)**2 + (y-(-self.CAR_W))**2

			up_circ_bound = (q*(abs(r) + 0.3))**2
			low_circ_bound = (q*(abs(r) - 0.3))**2
			if abs(x) > abs(r):
				return 2
#			if x < -1*q*((self.speed*.05) + r/3): #-radius/3
#				return 2

                        # if object is too far down rainbow to care about
                        #if abs(x) > abs(self.speed * 0.5 + r/3): # abs(r):
                        #        return 2

                        # if point is in rainbow path, not safe
                        if scan_point < up_circ_bound and scan_point > low_circ_bound:
				return 1
			else:
				return 2

#		elif self.curr_steering_angle < -0.2: # If car turning right
#			z = np.tan(self.curr_steering_angle)
#			radius = self.CAR_W/z
#			scan_point = ((x + radius)**2)+((y + self.CAR_W)**2)
#			up_circ_bound = (radius - 0.3)**2
#			low_circ_cound = (radius + 0.3)**2
#			# safe if too far
#                       if x > (self.speed*.05) + radius/4: #radius/3
#				return 2
#			#print radius
#                        if scan_point < up_circ_bound and scan_point > low_circ_cound:
#				return 1
#			else:
#				return 2

	def ackermann_cmd_input_callback(self, msg):
		if self.subscriber_safe:
			self.curr_steering_angle = msg.drive.steering_angle
			self.speed = msg.drive.speed
			if self.safe == 0:
				print "safe"
				self.cmd_pub.publish(msg)
				self.count = 0
			elif self.safe == 1:
				stop_msg = AckermannDriveStamped()
				if self.speed < 0:
					stop_msg.drive.speed = self.speed
				else:
					stop_msg.drive.speed = 0
				stop_msg.drive.steering_angle = self.curr_steering_angle
				self.cmd_pub.publish(stop_msg)
				self.count += 1
				if self.count >= 40:
					stop_msg.drive.speed = -1
					self.cmd_pub.publish(stop_msg)
				print "not safe"
		else:
			self.cmd_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("safety_controller")
    node = SafetyControllerNode()
    rospy.spin()
