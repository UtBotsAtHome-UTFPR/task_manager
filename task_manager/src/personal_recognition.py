import rospy
import smach
from std_msgs.msg import String
import os
import smach_ros
from std_srvs.srv import Empty

# State that calls the adding new images phase
class new_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['new_face_added', 'failed'])        

    def execute(self, userdata):

        rospy.loginfo('Executing state new_face')

        rospy.wait_for_service('/utbots_face_recognition/add_new_face')
        try:
            add_new_face = rospy.ServiceProxy('/utbots_face_recognition/add_new_face', Empty)
            resp1 = add_new_face()
            return 'new_face_added'
        
        except rospy.ServiceException as e:

            return 'failed'

# State that calls the training phase
class train(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['training_done', 'failed'])        

    def execute(self, userdata):

        rospy.loginfo('Executing state train')

        rospy.wait_for_service('/utbots_face_recognition/train')
        try:
            train_srv = rospy.ServiceProxy('/utbots_face_recognition/train', Empty)
            resp1 = train_srv()
            return 'training_done'
        
        except rospy.ServiceException as e:
            return 'failed'

# State that calls the recognition phase and publishes marked image and people
class recognize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recognized', 'failed'])        

    def execute(self, userdata):

        rospy.wait_for_service('/utbots_face_recognition/recognize')
        try:
            train_srv = rospy.ServiceProxy('/utbots_face_recognition/recognize', Empty)
            resp1 = train_srv()
            return 'recognized'
        
        except rospy.ServiceException as e:
            return 'failed'

# Transformar isso numa concurrency state machine pois as tarefas são perfeitamente cíclicas
def main():

    rospy.init_node('face_recognition_task')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NEW_FACE', new_face(), 
                               transitions={'new_face_added':'TRAIN',
                                            'failed':'failed'})
        
        smach.StateMachine.add('TRAIN', train(), 
                               transitions={'training_done':'RECOGNIZE',
                                            'failed':'failed'})

        smach.StateMachine.add('RECOGNIZE', recognize(), 
                               transitions={'recognized':'done',
                                            'failed':'failed'})

    # Execute SMACH plan
    outcome = sm.execute()
    print (outcome)

if __name__ == '__main__':
    main()