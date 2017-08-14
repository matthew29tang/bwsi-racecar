#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt32
from smach import State, StateMachine
from dank_pot import PotentialFields 
from state_ar_ids import STATE_AR_IDS

from ar_track_alvar_msgs.msg import AlvarMarkers
from time import sleep

class Underpass_STATE(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.ar_id_des = STATE_AR_IDS.Underpass_ID
        self.found_correct_ar_id = False

        self.id_dist_des = STATE_AR_IDS.Underpass_dist_des
        self.ar_close_enough = False

	self.passed_ar_tag = False

    def execute(self, userdata):
        # run pot node
        self.pot_node = PotentialFields()
        self.arSub = rospy.Subscriber('/ar_pose_marker',
                                       AlvarMarkers,
                                       self.ar_callback)

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown() and \
                not (self.found_correct_ar_id and self.ar_close_enough): #  and \
        #        not self.passed_ar_tag:
            r.sleep()
        print 'Underpass'
        
        sleep(1.25)
        # unsubscribe and destroy class
        self.arSub.unregister()
        self.pot_node.unsub_nodes()
        self.pot_node = None

        return 'success'

    def ar_callback(self, msg):
        markers = msg.markers

	# after ar_id is found detected, wait until we lose sight of it
        # before switching states
        #if len(markers) == 0 and (self.found_correct_ar_id and self.ar_close_enough):
        #    self.passed_ar_tag = True
        #    return

        # loop through all ar tags and find desired one
        for marker in markers:
            id_dist = marker.pose.pose.position.z
            print ("id found: {}, dist = {}".format(marker.id, id_dist))
            if (marker.id == self.ar_id_des) and (id_dist <= self.id_dist_des) :
                print ("id found: {}, dist = {}".format(marker.id, id_dist))
                self.found_correct_ar_id = True
                self.ar_close_enough = True

