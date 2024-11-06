#!/usr/bin/env python

import rospy
import smach
import yaml
import rospkg
import smach_ros
from geometry_msgs.msg import PoseStamped
from utbots_navigation.nav_main.src.smGoTo import SmGoTo
from std_msgs.msg import String

# Initialize the rospkg instance
rospack = rospkg.RosPack()

# Get the path to the desired package (replace 'your_package' with the actual package name)
package_path = rospack.get_path('task_manager')

def retrieve_waypoint(nametag):
    # Open and read the YAML file
    with open(f"{package_path}/waypoints.yaml", 'r') as file:
        data = yaml.safe_load(file)

    # Find the pose with the given nametag
    for pose_data in data['poses']:
        if pose_data['nametag'] == nametag:
            # Create a PoseStamped message
            pose_stamped = PoseStamped()

            # Fill in the header
            pose_stamped.header.seq = pose_data['header']['seq']
            pose_stamped.header.stamp.secs = pose_data['header']['stamp']['secs']
            pose_stamped.header.stamp.nsecs = pose_data['header']['stamp']['nsecs']
            pose_stamped.header.frame_id = pose_data['header']['frame_id']

            # Fill in the pose (position and orientation)
            pose_stamped.pose.position.x = pose_data['pose']['position']['x']
            pose_stamped.pose.position.y = pose_data['pose']['position']['y']
            pose_stamped.pose.position.z = pose_data['pose']['position']['z']

            pose_stamped.pose.orientation.x = pose_data['pose']['orientation']['x']
            pose_stamped.pose.orientation.y = pose_data['pose']['orientation']['y']
            pose_stamped.pose.orientation.z = pose_data['pose']['orientation']['z']
            pose_stamped.pose.orientation.w = pose_data['pose']['orientation']['w']

            return pose_stamped

# This state will wait for the door to be open
class enter_arena(smach.State):
    # def __init__(self, waypoint):
    #     smach.State.__init__(self, 
    #                         outcomes=['succeeded', 'aborted'],
    #                         input_keys=['waypoint'])

    # def execute(self, userdata):

    #     rospy.loginfo('Executing state go_to_shelf')

    #     try:
    #         # Sleep briefly to ensure the message is sent
    #         rospy.sleep(2)
    #         rospy.loginfo("Publish waypoint")
    #         self.pub.publish(retrieve_waypoint(self.waypoint))

    #         self.result = rospy.wait_for_message('/bridge_navigate_to_pose/result', String)
    #         print(self.result)
    #         if self.result.data == "Succeeded":
    #             self.result.data = ""
    #             return 'succeeded'
    #         else:
    #             self.result.data = ""
    #             return 'aborted'
        
    #     except rospy.ROSInterruptException:
    #         return 'aborted'


# This state will send a nav goal to the place the objects are located
class go_to_waypoint(smach.State):
    def __init__(self, waypoint):
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'aborted'],
                            input_keys=['waypoint'])
        self.result = "" 
        self.pub = rospy.Publisher('/bridge_navigate_to_pose/goal', PoseStamped, queue_size=1)
        self.waypoint = waypoint

    def execute(self, userdata):

        rospy.loginfo('Executing state go_to_shelf')

        try:
            # Sleep briefly to ensure the message is sent
            rospy.sleep(2)
            rospy.loginfo("Publish waypoint")
            self.pub.publish(retrieve_waypoint(self.waypoint))

            self.result = rospy.wait_for_message('/bridge_navigate_to_pose/result', String)
            print(self.result)
            if self.result.data == "Succeeded":
                self.result.data = ""
                return 'succeeded'
            else:
                self.result.data = ""
                return 'aborted'
        
        except rospy.ROSInterruptException:
            return 'aborted'


# This state initiates and manages operator following
class follow_operator(smach.State):
    # def __init__(self, waypoint):
    #     smach.State.__init__(self, 
    #                         outcomes=['succeeded', 'aborted'],
    #                         input_keys=['waypoint'])
    #     self.result = "" 
    #     self.pub = rospy.Publisher('/bridge_navigate_to_pose/goal', PoseStamped, queue_size=1)
    #     self.waypoint = waypoint

    # def execute(self, userdata):

    #     rospy.loginfo('Executing state go_to_shelf')

    #     try:
    #         # Sleep briefly to ensure the message is sent
    #         rospy.sleep(2)
    #         rospy.loginfo("Publish waypoint")
    #         self.pub.publish(retrieve_waypoint(self.waypoint))

    #         self.result = rospy.wait_for_message('/bridge_navigate_to_pose/result', String)
    #         print(self.result)
    #         if self.result.data == "Succeeded":
    #             self.result.data = ""
    #             return 'succeeded'
    #         else:
    #             self.result.data = ""
    #             return 'aborted'
        
    #     except rospy.ROSInterruptException:
    #         return 'aborted'

def main():
    rospy.init_node("sm_navigation_and_follow_me")
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])
    
    with sm:
        # State that waits for the door opening to enter the arena
        smach.StateMachine.add('ENTER_ARENA', 
                                enter_arena(),
                                transitions={'succeeded': 'GO_TO_WAYPOINT1',
                                            'aborted': 'failed'})
        
        # State that goes to the waypoint1
        smach.StateMachine.add('GO_TO_WAYPOINT1', 
                                go_to_waypoint("waypoint1"),
                                transitions={'succeeded': 'GO_TO_WAYPOINT2',
                                            'aborted': 'failed'})
                
        # State that goes to the waypoint2
        smach.StateMachine.add('GO_TO_WAYPOINT2', 
                                go_to_waypoint("waypoint2"),
                                transitions={'succeeded': 'FOLLOW_OPERATOR',
                                            'aborted': 'failed'})

        # State that initiates and manages operator following
        smach.StateMachine.add('FOLLOW_OPERATOR', 
                                follow_operator(),
                                transitions={'succeeded': 'GO_TO_WAYPOINT2_BACK',
                                            'aborted': 'failed'})
                                        
        # State that goes back to the waypint 2
        smach.StateMachine.add('GO_TO_WAYPOINT2_BACK', 
                                go_to_waypoint("waypoint2"),
                                transitions={'succeeded': 'LEAVE_ARENA',
                                            'aborted': 'failed'})
        # State that goes outside
        smach.StateMachine.add('LEAVE_ARENA', 
                                go_to_waypoint("outside"),
                                transitions={'succeeded': 'done',
                                            'aborted': 'failed'})

    sis = smach_ros.IntrospectionServer('navigation_and_follow_me', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()
    print(outcome)

    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    main()
        