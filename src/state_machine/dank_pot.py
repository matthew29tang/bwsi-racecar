import rospy
import numpy as np
import math

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt32, String
from tuning_vars import TUNING_VARS
from ar_track_alvar_msgs.msg import AlvarMarkers

class PotentialFields():
    def __init__(self):
        #print ("init")
        # Subscribe to laser scan
        self.speed_cap = TUNING_VARS.dank_pot_SPEED_CAP #2.5
        self.scanSub = rospy.Subscriber("/scan",
                         LaserScan,
                         self.laserscan_callback)

        #self.arSub = rospy.Subscriber("/ar_pose_marker",
                                       #AlvarMarkers,
                                       #self.ar_tag_callback)
        
        self.stateSub = rospy.Subscriber("/state", 
                                          String,
                                          self.state_callback)
        # Publish drive commands
        self.cmd_pub = rospy.Publisher("ackermann_cmd_mux/input/navigation",
                                        AckermannDriveStamped,
                                        queue_size=10)
        


        # TUNABLE_KX = Adjust weight of repulsive x forces (int)
        #               - Converts x_force to steering angle
        self.TUNABLE_KX = TUNING_VARS.dank_pot_KX

	# TUNABLE_KY = Adjust weight of repulsive y forces (int)
        #               - Converts y_force to racecar speed
	self.TUNABLE_KY = TUNING_VARS.dank_pot_KY

        self.hairping_STATE = False
        self.hairpin_id = 18
        #print (self.TUNABLE_KX)

        self.speed_up = False

    def unsub_nodes(self):
        self.scanSub.unregister()
        self.stateSub.unregister()

    def state_callback(self, msg):
        if msg.data == "OVERPASS":
            self.speed_up = True
        else:
            self.speed_up = False

    #def ar_tag_callback(self, msg):
        #markers = msg.markers
        #for marker in markers:
            #if marker.id == self.hairpin_id:
                #self.hairpin_STATE = True
                #return

    # called whenever new distances are measured by lidar
    # Argumemts:
    #   * scan = laser measurement.
    #       - scan.ranges provides distance within 270 deg of racecar.
    #       - scan.ranges[0] is at rightmost corner of lidar
    def laserscan_callback(self, scan):
#        print ("pot_field")
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
        if self.speed_up:
            print ("lower kx")
            self.TUNABLE_KX = TUNING_VARS.overpass_KX #0.0003

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
                # print ("y_force = {}, i = {}, dist = {}, cos(theta) = {}, theta = {}".format(y_force, i, dist, np.cos(theta), theta))
            else:
            	y_force = 0
            y_total += y_force * self.TUNABLE_KY

        # apply force onto racecar
        speed += y_total
        steering_angle += x_total

        # cap steering angle
	saturation = 0.3
	if steering_angle > saturation:
		steering_angle = saturation
	elif steering_angle < -saturation:
		steering_angle = -saturation

        # slow down when racecar is turning a lot
        if abs(steering_angle) > 0.15:
            speed -= abs(steering_angle) * TUNING_VARS.dank_pot_TURNING_SLOW_FACTOR #works
            #speed -= abs(steering_angle)**2 * TUNING_VARS.dank_pot_TURNING_SLOW_FACTOR_EXPERIMENTAL

        # hairpin turn
        # if self.hairpin_STATE:
        #    speed -= 0.5

        if self.speed_up:
            print ("overpasing")
            speed += TUNING_VARS.overpass_speed_boost
            
            
        # cap speed
	if speed > self.speed_cap:
		speed = self.speed_cap
	if speed < -self.speed_cap:
		speed = -self.speed_cap

        # print ("speed = {}, steering = {}, y_total = {}".format(speed, steering_angle, y_total))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.cmd_pub.publish(drive_msg)

if __name__ == '__main__':
    rospy.init_node("potential_fields")
    node = PotentialFields()
    rospy.spin()

