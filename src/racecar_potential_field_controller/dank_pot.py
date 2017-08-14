import rospy
import numpy as np
import math

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class Potential_Fields:
    def __init__(self):

        # Subscribe to laser scan
        rospy.Subscriber("/scan",
                         LaserScan,
                         self.laserscan_callback)

        # Publish drive commands
        self.cmd_pub = rospy.Publisher("ackermann_cmd_mux/input/navigation",
                                        AckermannDriveStamped,
                                        queue_size=10)
        
        self.speed_cap = 2 #1.5 #2.5
		######################################
		# TUNABLE_KX = Adjust weight of repulsive x forces (int)
                #               - Converts x_force to steering angle
        self.TUNABLE_KX = .0004 #.001 is good for speedcap 2.5 and 3
		# TUNABLE_KY = Adjust weight of repulsive y forces (int)
                #               - Converts y_force to racecar speed
	self.TUNABLE_KY = 1.0 / 5000 #good for speedcap 3 #1.0 / 10000 good for speedcap 2.5
		######################################

    # called whenever new distances are measured by lidar
    # Argumemts:
    #   * scan = laser measurement. 
    #       - scan.ranges provides distance within 270 deg of racecar.
    #       - scan.ranges[0] is at rightmost corner of lidar
    def laserscan_callback(self, scan):
        ranges = scan.ranges
        mid_index = len(ranges) / 2
        
        # want to search full 180 deg in front of in front of racecar
        search_range = float(2)/3 * len(ranges)
        start_index = int(mid_index - (search_range / 2))
        end_index = int(mid_index + (search_range / 2) + 1)

        steering_angle = 0
        speed = self.speed_cap
        
        # total forces acting on racecar
        x_total = 0
        y_total = 0

        # print ("mid_index = {}, incr = {}".format(mid_index, scan.angle_increment))
        for i in range(start_index, end_index):
            dist = ranges[i]
            theta = (i - mid_index) * scan.angle_increment # in radians
            #print (theta)

            # repulsive force applied horizontally to racecar. Sum up to determine turn angle
            if theta != 0:
            	x_force = -1.0 / (dist**2 * np.sin(theta))
            else:
            	x_force = 0
            x_total += x_force * self.TUNABLE_KX
            
            # repulsive force to slow down racecar as it gets close to object
            if abs(np.cos(theta)) > 0.1:
                y_force = -1.0 / (dist**2 * np.cos(theta))
                #if y_force < -1000:
                #    print ("y_force = {}, i = {}, dist = {}, cos(theta) = {}, theta = {}".format(y_force, i, dist, np.cos(theta), theta))
            else:
            	y_force = 0
            y_total += y_force * self.TUNABLE_KY
            
# ===================================== Method 2 ===========================================
            #mag_force = 1.0 / (dist**2)  # dist farther, force weaker
 
            #dir_force = 1.0 / theta      # Greater magitude of theta means farther from center of robot
                                          # Farther off  from center = affect robot less
                                          # Points closer to center weighed heavily

            # Points are weighed proportionally based on their thetas
            #dir_force = 1 * np.sign(theta) - (theta / math.radians(90))

            #sum up all directional forces for final steering angle
            #steering_angle += self.TUNABLE_KX  * (mag_force * dir_force)
        
        
	#print "x_repulsion: {}".format(x_force)

        # apply force onto racecar
        speed += y_total
        steering_angle += x_total

	#assume no crashing and stop with safety controller
        speed = self.speed_cap

        # cap speeds and steering angle
	saturation = 0.3
	if steering_angle > saturation:
		steering_angle = saturation
	elif steering_angle < -saturation:
		steering_angle = -saturation
	if speed > self.speed_cap:
		speed = self.speed_cap
	elif speed < .2 and speed > 0:
	        speed = .2
	if speed < -self.speed_cap:
		speed = -self.speed_cap
	elif speed > -.2 and speed < 0:
	        speed=.2


        print ("speed = {}, steering = {}, y_force = {}".format(speed, steering_angle, y_total))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.cmd_pub.publish(drive_msg)

if __name__ == '__main__':
    rospy.init_node("potential_fields")
    node = Potential_Fields()
    rospy.spin()

