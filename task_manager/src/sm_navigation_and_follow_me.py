#!/usr/bin/env python

import rospy
import smach
import yaml
import rospkg
import smach_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from utbots_navigation.nav_main.src.smGoTo import SmGoTo
from std_msgs.msg import String
import time
import math
import tf2_ros

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

    # def execute(self, user    #         self.result = rospy.wait_for_message('/bridge_navigate_to_pose/result', String)
#data):

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


def find_and_follow():
    
    current_position = rospy.wait_for_message()

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Wait for the transform to become available
    try:
        transform = tf_buffer.lookup_transform('odom', 'camera_link', rospy.Time(0), rospy.Duration(1.0))

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.loginfo(f"Transform lookup failed: {e}")
        return 'aborted'
    

    # Transform the point to the 'odom' frame
    point_in_camera = tf_buffer.transform(current_position, 'camera_link', rospy.Duration(1.0))
    
    # Calculate the angle between A and B
    theta = math.atan2(point.point.y[1] - point_in_camera.pose.orientation.y, point.point.x - point_in_camera.pose.orientation.x)

    # Create the quaternion (w, x, y, z)
    quaternion = (math.cos(theta / 2), 0, 0, math.sin(theta / 2))

    goal = PoseStamped()
    goal.header.frame_id = "camera_link"
    goal.pose.position.x = point.point.x
    goal.pose.position.y = point.point.y
    goal.pose.orientation.w = quaternion[0]
    goal.pose.orientation.z = quaternion[3]

    return goal
    

# This state initiates and manages operator following
class follow_operator(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'aborted'])
        self.result = "" 
        self.pub_goal = rospy.Publisher('/bridge_navigate_to_pose/goal', PoseStamped, queue_size=1)
        self.pub_bt = rospy.Publisher('/bridge_navigate_to_pose/bt', String, queue_size=1)

    def execute(self, userdata):

        rospy.loginfo('Executing state follow_operator')

        point = rospy.wait_for_message("tópico da posição da pessoa", PointStamped)


        bt = String()
        bt.data = "follow_point"
        self.pub_bt(bt)

        time.sleep(1)
        
        self.pub_goal.publish(find_and_follow())

        while(True):
            rospy.wait_for_message('bridge_navigate_to_pose/result')

            if stop_flag:
                bt.data = ""
                self.pub_bt(bt)
                time.sleep(1)
                return 'succeeded'

            self.pub_goal.publish(find_and_follow())
            
        
global stop_flag 
stop_flag = False

def stop_sub(msg):
    stop_flag = True if msg.data == 'stop' else False

def main():
    rospy.init_node("sm_navigation_and_follow_me")
    rospy.Subscriber('/tópico_de_falar_pare', String, stop_sub)
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
        