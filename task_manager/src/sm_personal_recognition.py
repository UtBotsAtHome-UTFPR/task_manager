import rospy
import smach
import smach_ros
import actionlib

import utbots_actions.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np
from scipy.spatial.transform import Rotation as R
# State that calls the adding new images phase
class new_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['new_face_added', 'failed'])

        rospy.loginfo('Checking new_face action')

        self.client = actionlib.SimpleActionClient('new_face', utbots_actions.msg.new_faceAction)
        self.client.wait_for_server()

    def execute(self, userdata):

        rospy.loginfo('Executing state new_face')

        goal = utbots_actions.msg.new_faceGoal()
        
        goal.n_pictures.data = 1
        goal.name.data = "Operator"

        self.client.send_goal_and_wait(goal)

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'new_face_added'
        return 'failed'
        

# State that calls the training phase
class train(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['training_done', 'failed'])

        rospy.loginfo('Checking train action')       

        self.client = actionlib.SimpleActionClient('train', utbots_actions.msg.trainAction)
        self.client.wait_for_server()

    def execute(self, userdata):

        rospy.loginfo('Executing state train')

        goal = utbots_actions.msg.trainGoal()

        self.client.send_goal_and_wait(goal)

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'training_done'
        return 'failed'

# State that calls the recognition phase and publishes marked image and people
class recognize(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['recognized', 'failed'])

        rospy.loginfo('Checking recognize action')

        self.client = actionlib.SimpleActionClient('recognize', utbots_actions.msg.recognitionAction)
        self.client.wait_for_server()


    def execute(self, userdata):

        rospy.loginfo('Executing state recognize')

        goal = utbots_actions.msg.recognitionGoal()
        goal.ExpectedFaces.data = 0

        self.client.send_goal_and_wait(goal, rospy.Duration(3))

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'recognized'
        return 'failed'
    
class init_pose(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'])
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    def execute(self, userdata):

        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

        init_msg = PoseWithCovarianceStamped()
        init_msg.header.frame_id = "map"

        odom_msg = rospy.wait_for_message('/odom', Odometry)

        init_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
        init_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
        init_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
        init_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
        init_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        init_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

        rospy.sleep(1)

        rospy.loginfo("setting initial pose")
        self.pub.publish(init_msg)
            
        return 'succeeded'

class turn_around(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        rospy.loginfo('Checking navigation action')

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):

        rospy.loginfo('Executing state turn_around')

        odom_msg = rospy.wait_for_message('/odom', Odometry)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = odom_msg.pose.pose.position.x
        goal.target_pose.pose.position.y = odom_msg.pose.pose.position.y
        goal.target_pose.pose.position.z = 0.0

        quaternion = np.array([odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z])


        rotation_180_x = R.from_euler('x', 180, degrees=True).as_quat()

        # Multiply the quaternions to apply the 180-degree rotation
        flipped_quaternion = R.from_quat(rotation_180_x) * R.from_quat(quaternion)

        # Get the result as a quaternion
        flipped_quaternion = flipped_quaternion.as_quat()

        goal.target_pose.pose.orientation.x = flipped_quaternion[1]
        goal.target_pose.pose.orientation.y = flipped_quaternion[2]
        goal.target_pose.pose.orientation.z = flipped_quaternion[3]
        goal.target_pose.pose.orientation.w = flipped_quaternion[0]

        self.client.send_goal(goal)

        finished = self.client.wait_for_result()

        if not finished: rospy.logerr("Action server not available")
        else:
            rospy.loginfo(self.client.get_result())

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        return 'failed'

# Transformar isso numa concurrency state machine pois as tarefas são perfeitamente cíclicas
def main():

    rospy.init_node('face_recognition_task')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INITIAL_POSE', init_pose(), 
                               transitions={'succeeded':'NEW_FACE'})
        
        smach.StateMachine.add('NEW_FACE', new_face(), 
                               transitions={'new_face_added':'TRAIN',
                                            'failed':'failed'})
        
        smach.StateMachine.add('TRAIN', train(), 
                               transitions={'training_done':'TURN_AROUND',
                                            'failed':'failed'})
        
        smach.StateMachine.add('TURN_AROUND', turn_around(), 
                               transitions={'succeeded':'RECOGNIZE',
                                            'failed':'failed'})

        smach.StateMachine.add('RECOGNIZE', recognize(), 
                               transitions={'recognized':'done',
                                            'failed':'failed'})

    # Execute SMACH plan
    outcome = sm.execute()
    print (outcome)

if __name__ == '__main__':
    main()