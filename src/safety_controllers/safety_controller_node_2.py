#!usr/bin/python
import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class SafetyControllerNode:
	def __init__(self):
		# Subscribe to drive command and laser scan topics
		rospy.Subscriber("/scan", LaserScan, self.laser_scan_input_callback)
		rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, self.ackermann_cmd_input_callback)

		# Publish safe drive commands to final drive command topic
		self.cmd_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size = 10)

		# Variables
		self.safe = True
		self.car_w = 0.45
		self.speed = 1
		self.curr_steering_angle = 0

	def laser_scan_input_callback(self, scan):
		dists = scan.ranges
		semi_range = len(dists)*2/3
		self.mid_ind = len(dists)/2
		low_bound = self.mid_ind - semi_range/2
		up_bound = self.mid_ind + semi_range/2
		for i in range(0,8):
			dist_sum = 0
			first_ind = low_bound + 90*i
			self.theta = (self.mid_ind - ((first_ind)+45))*scan.angle_increment
			for j in range (0,90):
				dist_sum += dists[first_ind+j]
				#print "range: {}, distance: {}".format(i, dists[first_ind+j])
			avg = dist_sum/90
			x = 30
			y = 10
			if i != 3 and i != 4:
				x = avg*np.sin(self.theta)
			print "range: {}, avg: {}".format(i, avg) 
			if i != 0 and i != 7:
				y = avg*np.cos(self.theta)
			self.safe = self.in_range_for_steer_angle(x,y)
			if not self.safe:
				break

	def in_range_for_steer_angle(self, x, y):
		if self.curr_steering_angle >= -0.1 and self.curr_steering_angle <= 0.1:
			x_range = 0.25
			y_range = 0.7
			print "x: {}, y: {}".format(x,y)
			if abs(x) > x_range and abs(y) > y_range:
				return True
			else:
				return False

		elif abs(self.curr_steering_angle) > 0.1:
			#print "rainbow"
			z = np.tan(self.curr_steering_angle)
			c_rad = self.car_w/z
			low_rad_bound = c_rad - 0.3
			up_rad_bound = c_rad + 0.3
			radius = np.sqrt((x+c_rad)**2+(y+self.car_w)**2)
			if radius < low_rad_bound:
				return True
			elif radius > up_rad_bound:
				return True
			else:	
				return False

	def ackermann_cmd_input_callback(self, msg):
		self.curr_steering_angle = msg.drive.steering_angle 
		self.speed = msg.drive.speed 
		if self.safe:
			#print "safe"
			self.cmd_pub.publish(msg)
		else:
			stop_drive = AckermannDriveStamped()
			stop_drive.drive.speed = 0
			stop_drive.drive.steering_angle = self.curr_steering_angle
			self.cmd_pub.publish(stop_drive)
			#print "not safe"

if __name__ == "__main__":
    rospy.init_node("safety_controller")
    node = SafetyControllerNode()
    rospy.spin()
		
