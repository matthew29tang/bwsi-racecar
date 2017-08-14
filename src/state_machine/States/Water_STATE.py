#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt32, String
from smach import State, StateMachine
from find_yellow_line import FindYellow
from line_follow import LineFollowNode
from state_ar_ids import STATE_AR_IDS

from ar_track_alvar_msgs.msg import AlvarMarkers

from time import sleep

class Water_STATE(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.ar_id_des = STATE_AR_IDS.Water_ID
        self.found_correct_ar_id = False

        self.id_dist_des = STATE_AR_IDS.Water_dist_des
        self.ar_close_enough = False

	self.passed_ar_tag = False

        self.counter = 0 # how many frames you've lost ar tag for

    def execute(self, userdata):
        # run pot node
        self.yellow_node = FindYellow()
        self.line_follower_node = LineFollowNode()
        
        self.arSub = rospy.Subscriber('/ar_pose_marker',
                                       AlvarMarkers,
                                       self.ar_callback)

        self.state_pub = rospy.Publisher('/state',
                                          String,
                                          queue_size=10)
        self.state_pub.publish("WATER")

        r = rospy.Rate(10) # 10hz
        # rospy is on, found id, id is close enough, and anad don't  see tag anymore
        #print ("r on = {}, found ar = {}, close = {}, counter = {}".format(rospy.is_shutdown(), self.found_correct_ar_id, self.ar_close_enough, self.counter))
        #print ("")
        while not rospy.is_shutdown() and not (self.found_correct_ar_id and self.ar_close_enough): # and self.counter < 5:
                #not self.passed_ar_tag:
            r.sleep()
        #print 'WaterHazard'
        
        #sleep(1)
        # unsubscribe and destroy class
        self.arSub.unregister()
        self.yellow_node.unsub_nodes()
        self.yellow_node = None

        self.line_follower_node.unsub_nodes()
        self.line_follower_node = None

        return 'success'

    def ar_callback(self, msg):
        markers = msg.markers

	# after ar_id is found detected, wait until we lose sight of it
        # before switching states
        # if len(markers) == 0 and (self.found_correct_ar_id and self.ar_close_enough):
            #self.passed_ar_tag = True
        #    self.counter += 1
        #    print ("counters = {}".format(self.counter))
        #    return
        #else:
        #    self.counter = 0

        # loop through all ar tags and find desired one
        for marker in markers:
            id_dist = marker.pose.pose.position.z
            print ("`id found: {}, dist = {}".format(marker.id, id_dist))
            if (marker.id == self.ar_id_des) and (id_dist <= self.id_dist_des):
                print ("`id found: {}, dist = {}".format(marker.id, id_dist))
                self.found_correct_ar_id = True
                self.ar_close_enough = True

