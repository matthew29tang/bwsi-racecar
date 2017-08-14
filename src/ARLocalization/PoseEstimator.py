#!/usr/bin/env python

import numpy as np

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseArray, PoseWithCovarianceStamped, Pose2D
from nav_msgs.msg import Odometry
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers

from ARLocalization.Utils import *
from ARLocalization.MotionModel import MotionModel
from ARLocalization.SensorModel import SensorModel

from tf.transformations import euler_from_quaternion

class PoseEstimator:

    def __init__(self):

        # Get ros params
        self.num_particles = rospy.get_param('/ar_localization/num_particles', 500)
        init_pose = rospy.get_param('/ar_localization/init_pose', (0., 0., 0.))
        self.ar_locations = rospy.get_param('/ar_localization/ar_locations', {"1": [5., 5., 0]})
        self.init_pos_dev = rospy.get_param('/ar_localization/init_pos_dev', 0.4)
        self.init_ang_dev = rospy.get_param('/ar_localization/init_ang_dev', 1.)

        self.prev_odom_pose = None

        # Create a transformation frame
        self.transform_broadcaster = tf.TransformBroadcaster()
        self.publish_transformation_frame(init_pose)

        # Initialize the motion and sensor models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        # Publisher for the expected pose
        self.expected_pose_pub = rospy.Publisher(
                "expected_pose", 
                PoseStamped, 
                queue_size=1)
        # Publisher to display particles
        self.particles_pub = rospy.Publisher(
                "particles", 
                PoseArray, 
                queue_size=1)

        # Initialize particles 
        self.initialize_particles(init_pose)

        # Subscribe to odometry data
        self.odom_cb = rospy.Subscriber(
                "/odom/", 
                Odometry,
                self.odom_callback, 
                queue_size=1)
        # Subscribe to ar tag data
        self.sub = rospy.Subscriber(
                "/ar_pose_marker", 
                AlvarMarkers, 
                self.marker_callback, 
                queue_size=1)
        # Subscribe to clicks in rviz
        self.pose_sub = rospy.Subscriber(
                "/initialpose",
                PoseWithCovarianceStamped, 
                self.clicked_pose, 
                queue_size=1)

        rospy.loginfo("Pose estimator is initialized.")


    def initialize_particles(self, init_pose):
        """
        TODO
        initialize the particles randomly around the init_pose

        Use np.random.normal to normally distribute the position
        and angle based on the standard deviations self.init_pos_dev and 
        self.init_ang_dev 
        """
        self.particles = np.zeros((self.num_particles, 3))

        #######################
        # Your work here
        self.particles[:,0] = np.random.normal(init_pose[0], self.init_pos_dev, self.num_particles)
        self.particles[:,1] = np.random.normal(init_pose[1], self.init_pos_dev, self.num_particles)
        self.particles[:,2] = np.random.normal(init_pose[2], self.init_ang_dev, self.num_particles)
        #######################

        self.publish_pose()

    def ar_tag_pose_to_2D(self, pose):
        """
        TODO

        Copy your code from ar_monitor.py to compute
        the (x, y, theta) pose of an AR tag given a pose.
        """

        #######################
        # Your work here

        pose_pos = pose.position
        pose_ori = pose.orientation
    	(roll,pitch,yaw) = euler_from_quaternion([pose_ori.x, \
    		pose_ori.y, pose_ori.z, pose_ori.w])
        new_pose = Pose2D()
        new_pose.x = pose_pos.x
        new_pose.y = pose_pos.z
        new_pose.theta = pitch 

        return (new_pose.x, new_pose.y, new_pose.theta)

    def marker_callback(self, msg):
        
        observations = []

        for marker in msg.markers:

            if str(marker.id) not in self.ar_locations:
                continue

            # Convert the marker pose to
            # (x, y, theta) tuple
            observed_relative_pose = self.ar_tag_pose_to_2D(marker.pose.pose)

            # The position of the ar tag on the map
            ar_pose = np.array(self.ar_locations[str(marker.id)])

            # Pack the observation for the sensor model
            observation = (observed_relative_pose, ar_pose)
            observations.append(observation)

        if observations:
            # Update the sensor model
            self.particles = self.sensor_model.update(observations, self.particles)

            # Publish the pose
            self.publish_pose()

    def odom_callback(self, data):
        odom_pose = odom_to_pose(data)

        if self.prev_odom_pose is None:
            self.prev_odom_pose = odom_pose

        # Update the motion model
        self.particles = self.motion_model.update(
                self.prev_odom_pose, 
                odom_pose,
                self.particles)
        
        self.prev_odom_pose = odom_pose

        # Publish the pose
        self.publish_pose()

    def clicked_pose(self, msg):
        # Receive pose messages from RViz and initialize 
        # the particle distribution in response.
        rospy.loginfo("Setting pose.")
        pose = msg.pose.pose

        x = pose.position.x
        y = pose.position.y
        theta = quaternion_to_angle(pose.orientation)

        self.initialize_particles((x, y, theta))

    def compute_expected_pose(self):
        """
        TODO
        
        Use the self.particles variable to compute the
        expected pose. 

        np.mean is useful

        Be careful when averaging angles! 
        Google mean of circular quantities
        """

        #######################
        # Your work here
        x = np.mean(self.particles[:,0])
        y = np.mean(self.particles[:,1])
        theta_x = np.mean(np.cos(self.particles[:,2]))
        theta_y = np.mean(np.sin(self.particles[:,2]))
        theta = np.arctan2(theta_y, theta_x)

        return np.array((x, y, theta))

    def publish_pose(self):

        # Compute the expected pose
        # by averaging all of the particle's poses
        expected_pose = self.compute_expected_pose()

        # Publish the particles
        particle_msg = PoseArray()
        particle_msg.header = make_header("/map")
        particle_msg.poses = particles_to_poses(self.particles)
        self.particles_pub.publish(particle_msg)

        # Publish the expected pose
        pos, quat = pose_to_pos_quat(expected_pose)
        pose_msg = PoseStamped()
        pose_msg.header = make_header("/map")
        pose_msg.pose = Pose(Point(*pos), Quaternion(*quat))
        self.expected_pose_pub.publish(pose_msg)

        # Publish the transformation frame
        self.publish_transformation_frame(expected_pose)

    def publish_transformation_frame(self, pose):
        pos, quat = pose_to_pos_quat(pose)
        self.transform_broadcaster.sendTransform(
                pos,
                quat,
                rospy.Time.now(),
                "/map",
                "/base_link")

if __name__ == "__main__":
    rospy.init_node("pose_estimator")
    pe = PoseEstimator()
    rospy.spin()

