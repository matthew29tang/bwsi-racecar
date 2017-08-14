#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt32, Bool, String
from smach import State, StateMachine
from dank_pot import PotentialFields 
from state_ar_ids import STATE_AR_IDS

from time import sleep

from ar_track_alvar_msgs.msg import AlvarMarkers

class Overpass_STATE(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.id_dist_des = STATE_AR_IDS.Overpass_dist_des
        self.ar_close_enough = False

        self.ar_id_des = STATE_AR_IDS.Overpass_ID
        self.found_correct_ar_id = False

	self.passed_ar_tag = False

        self.safetyOn_pub = rospy.Publisher('/safety_on',
                                             Bool,
                                             queue_size=10)
        
        self.state_pub = rospy.Publisher('/state',
                                         String,
                                         queue_size=10)

    def execute(self, userdata):

        # run pot node
        self.pot_node = PotentialFields()
        self.arSub = rospy.Subscriber('/ar_pose_marker',
                                       AlvarMarkers,
                                       self.ar_callback)

        msg = Bool()
        # msg.data = False
        msg.data = True
        self.safetyOn_pub.publish(msg)

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown() and \
                not (self.found_correct_ar_id and self.ar_close_enough) and \
                not self.passed_ar_tag:
            self.state_pub.publish("OVERPASS")
            r.sleep()
        print 'Overpass'

        # turn safety back on after overpass
        sleep(0.5)
        self.safetyOn_pub.publish(Bool(True))
        
        # unsubscribe
        self.arSub.unregister()
        self.pot_node.unsub_nodes()
        self.pot_node = None

        return 'success'
    
    def ar_callback(self, msg):
        markers = msg.markers

	# after ar_id is found detected, wait until we lose sight of it
        # before switching states
        if len(markers) == 0 and (self.found_correct_ar_id and self.ar_close_enough):
            self.passed_ar_tag = True
            return

        # loop through all ar tags and find desired one
        for marker in markers:
            id_dist = marker.pose.pose.position.z
            if (marker.id == self.ar_id_des) and (id_dist <= self.id_dist_des) :
                print ("id found: {}, dist = {}".format(marker.id, id_dist))
                self.found_correct_ar_id = True
                self.ar_close_enough = True
        
if __name__ == "__main__":
    rospy.init_node("overpass")
    node = Overpass_STATE()
    node.execute()
    rospy.spin()
