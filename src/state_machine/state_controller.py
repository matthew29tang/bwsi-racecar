#!/usr/bin/env python
import rospy
from smach import State, StateMachine

# import states
from States.GreenLight_STATE import GreenLight_STATE
from States.RollingWeave_STATE import RollingWeave_STATE
from States.Overpass_STATE import Overpass_STATE
from States.Water_STATE import Water_STATE
from States.Underpass_STATE import Underpass_STATE
from States.MeshWall_STATE import MeshWall_STATE

if __name__ == '__main__':
    rospy.init_node("state_controller")

    sm = StateMachine(outcomes=['success', 'failure'])
    with sm:
# === ======================= COMPETITION ORDER ==================================

        StateMachine.add('GREEN_LIGHT',
                          GreenLight_STATE(),
                          transitions={'success':'ROLLING_WEAVE'})

        StateMachine.add('ROLLING_WEAVE',
                          RollingWeave_STATE(),
                          transitions={'success':'OVERPASS'})

        StateMachine.add('OVERPASS',
                          Overpass_STATE(),
                          transitions={'success':'WATER_HAZARD'})

        StateMachine.add('WATER_HAZARD',
                          Water_STATE(),
                          transitions={'success':'UNDERPASS'})

        StateMachine.add('UNDERPASS',
                          Underpass_STATE(),
                          transitions={'success':'OVERPASS'})
        
        StateMachine.add('MESH_WALL',
                          MeshWall_STATE(),
                          transitions={'success':'success'})

# ========================= TESTING ================================

#        StateMachine.add('GREEN_LIGHT',
#                          GreenLight_STATE(),
#                          transitions={'success':'WATER_HAZARD'})
#
#        StateMachine.add('ROLLING_WEAVE',
#                          RollingWeave_STATE(),
#                          transitions={'success':'success'})
#
#        StateMachine.add('OVERPASS',
#                          Overpass_STATE(),
#                          transitions={'success':'WATER_HAZARD'})
#
#        StateMachine.add('WATER_HAZARD',
#                          Water_STATE(),
#                          transitions={'success':'UNDERPASS'})
#
#        StateMachine.add('UNDERPASS',
#                          Underpass_STATE(),
#                          transitions={'success':'success'})
#        
#        StateMachine.add('MESH_WALL',
#                          MeshWall_STATE(),
#                          transitions={'success':'WATER_HAZARD'})
    
    sm.execute()
    rospy.spin()
