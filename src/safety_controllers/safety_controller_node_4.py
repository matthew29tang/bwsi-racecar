#!usr/bin/python
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
		self.safe = True
		self.car_w = 0.45
		self.speed = 1
		self.curr_steering_angle = 0
		self.state = True

	def safety_callback(self, state):
		self.state = state.data

	def laser_scan_input_callback(self, scan):
		# Method variables and calculations
		if self.state:
			dists = scan.ranges
			semi_range = len(dists)*2/3
			mid_ind = len(dists)/2
			low_bound = mid_ind - semi_range/2
			up_bound = mid_ind + semi_range/2

			x = 0
			y = 0
			scan_partitions = 8
			# Keep even to avoid cancelation of vectors in center partition

			for i in range(0,scan_partitions):
				dist_sum = 0
				partition_width = semi_range/scan_partitions
				first_ind = low_bound + i*partition_width

				# Get center index of given partition and determine theta at that index
				self.theta = (mid_ind - ((first_ind)+(partition_width/2)))*scan.angle_increment 

				for j in range (0, partition_width):
					dist_sum += dists[first_ind+j]
					#print "range: {}, distance: {}".format(i, dists[first_ind+j])
				avg = dist_sum/partition_width

				x = -avg*np.sin(self.theta)
				#print "range: {}, x: {}".format(i, x) 
				y = avg*np.cos(self.theta)
				#print "range: {}".format(i)
				self.safe = self.in_range_for_steer_angle(x,y)
				if not self.safe:
					break
				#print "range: {}, boolean: {}".format(i, self.safe)
		else:
			print "success"

	def in_range_for_steer_angle(self, x, y):
		#if abs(self.curr_steering_angle) <= 0.1:
			x_range = 0.2
			y_range = .3 + .2*self.speed #.9 speed 2
			#print y_range
			
			if abs(x) < x_range and abs(y) < y_range:
				return False
			elif abs(x) > x_range and abs(y) > y_range:
				return True
			else:
				return True
			#print "x: {}, y: {}".format(x,y)

		#elif self.curr_steering_angle < 0:
			#print "rainbow"
			#z = np.tan(self.curr_steering_angle)
			##c_rad = 5*(self.car_w/z)
			#low_rad_bound = c_rad - 0.15
			#up_rad_bound = c_rad + 0.15
			#print "low radius: {}".format(low_rad_bound)
			#print "upper radius: {}".format(up_rad_bound)
			#radius = np.sqrt((x+c_rad)**2+(y+self.car_w)**2)
			#print "radius: {}".format(radius)
			#if radius < low_rad_bound:
				#return True
			#elif radius > up_rad_bound:
				#return True
			#elif radius > low_rad_bound and radius < up_rad_bound:
				#return False
			#else:
				#return True
		#else:
			#x_range = 0.1
			#y_range = .3 #.9 speed 2
			#print "x: {}, y: {}".format(x,y)
			#if abs(x) < x_range and abs(y) < y_range:
				#return False
			#else:
				#return True
	def ackermann_cmd_input_callback(self, msg):
		if self.state:
			self.curr_steering_angle = msg.drive.steering_angle
			#print self.curr_steering_angle
			self.speed = msg.drive.speed 
			if self.safe:
				print "safe"
				self.cmd_pub.publish(msg)
			else:
				stop_drive = AckermannDriveStamped()
				if msg.drive.speed < 0:
					stop_drive.drive.speed = msg.drive.speed
				else:
					stop_drive.drive.speed = 0
				stop_drive.drive.speed = 0
				stop_drive.drive.steering_angle = self.curr_steering_angle
				self.cmd_pub.publish(stop_drive)
				print "not safe"
		else:
			self.cmd_pub.publish(msg)
		#print "on/off: {}".format(self.state)

if __name__ == "__main__":
    rospy.init_node("safety_controller")
    node = SafetyControllerNode()
    rospy.spin()
