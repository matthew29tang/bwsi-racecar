#!/user/bin/env python

import rospy
from smach import State, StateMachine

from green_light import GreenLight
from std_msgs.msg import Bool

from time import sleep

class GreenLight_STATE(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.found_green = False

    def execute(self, userdata):
        # Will try to find green light
        self.green_node = GreenLight()
        self.greenSub = rospy.Subscriber('/found_green',
                                          Bool,
                                          self.callback)

        # r = rospy.Rate(10) # 10hz
        # wait until we detect green light to move on
        #while not rospy.is_shutdown() and not self.found_green_light:
        #    r.sleep()

        print 'green_light'
        sleep(1)
        # unsubscribe
        self.greenSub.unregister()
        self.green_node.unsub_nodes()
        self.green_node = None
        
        return 'success'

    def callback(self, found_green):
        self.found_green = found_green
