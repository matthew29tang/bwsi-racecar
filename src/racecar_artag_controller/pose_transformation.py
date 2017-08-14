#!/usr/bin/python
import roslib
import rospy
import tf

from std_msgs.msg import UInt32
from geometry_msgs.msg import Quaternion, Pose2D
from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers

class arMonitor:
    def __init__(self):
	rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_pose)
        self.pose_pub = rospy.Publisher('/ar_2D_pose', Pose2D, queue_size=10)
        self.id_pub = rospy.Publisher('ar_id', UInt32, queue_size=10)

    def ar_pose(self, msg):
    	markers = msg.markers

        # find x, y, theta of first ar and publish them
        if len(markers) != 0:
            # publish ar id
            ar1 = markers[0]
            ar1_id = ar1.id
            self.id_pub.publish(ar1_id)

    	    ar1_orientation = markers[0].pose.pose.orientation
            ar1_pos = markers[0].pose.pose.position
    	    
            #print (ar1_pos)
            #print (ar1_orientation)
           
            (roll,pitch,yaw) = euler_from_quaternion([ar1_orientation.x, \
                            ar1_orientation.y, ar1_orientation.z, ar1_orientation.w])
            # print ("roll = {}, pitch = {}, yaw = {}".format(roll,pitch,yaw))
            
            # convert 3D point to 2D point
            # 2D point describes depth(z) and angle from camera(pitch) relative to racecar
            new_pose = Pose2D()
            new_pose.x = ar1_pos.x
            new_pose.y = ar1_pos.z
            new_pose.theta = pitch

            self.pose_pub.publish(new_pose)

if __name__ == '__main__':
    rospy.init_node('ar_monitor')
    node = arMonitor()
    rospy.spin()
