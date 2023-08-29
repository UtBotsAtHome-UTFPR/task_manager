#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from utbots_navigation.nav_main.src.smGoTo import SmGoTo
def main():
    rospy.init_node("navigation_and_follow_me")
    
    sm = smach.StateMachine(outcomes=["succeded", "aborted"])
    
    with sm:
        
        sm_wp = smach.StateMachine(outcomes=["reached, aborted"])
        sm_wp.userdata.action_type = "pose"
        
        with sm_wp:
            smach.StateMachine.add("Get_Wp", SmGetWaypoint(),
                                transitions={'got_wp':'Go_Wp'},
                                remapping={'pose':'coordinate'})
            smach.StateMachine.add("Go_Wp", SmGoTo(),
                               transitions={'sent':'reached'},
                               remapping={'type':'action_type'})
            
        smach.StateMachine.add("Waypoint_1", sm_wp,
                               transitions={'reached':"Waypoint_2"})
        smach.StateMachine.add("Waypoint_2", sm_wp,
                               transitions={'reached':"Waypoint_3"})
        
        outcome - sm.execute()
    
if __name__ == '__main__':
    main()
        