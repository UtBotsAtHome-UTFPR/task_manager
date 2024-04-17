import rospy
import smach
from std_msgs.msg import String
import os
import smach_ros
from std_srvs.srv import Empty


# Wait for someone to show up, possibly voice queue or find a face in the image: 1 outcome

# Run new face routine: if failed for whatever reason go to previous step and tell the user it failed

# Run training: If failed try again, if fail twice go back one step

# Tell the user to go to the crowd and tell the user what is the voice queue to start looking for him, and to repeat in case the robot doesn't start imediatly: 1 outcome after voice queue

# TURN AROUND EVERY NOW AND THEN i GET A LITTLE BIT LONELY: 1 outcome

# Run recognition
# Only correct when only one person is identified as something other than unknown, check for that: 1 outcome

# Save the recognized image: Not really a state but will have to do

# Display the user: 1 outcome

# Display how many people were recognized (ask yolo or look for how many faces there are, yolo should be more reliable): DONE


# define state Foo
class wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop_waiting'])

        self.loopRate = rospy.Rate(30)

        self.done_waiting = False
 
        #self.pub_instructions.publish("I am now waiting, please input a message to begin")

    def execute(self, userdata):
        rospy.loginfo('Executing state waiting')
        
        while rospy.is_shutdown() == False:
            # Controls speed
            self.loopRate.sleep()

            self.done_waiting = False if input() == "" else True

            if self.done_waiting:
                return 'stop_waiting'
            


# define state Bar
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

class recognize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recognized', 'failed'])        

    def execute(self, userdata):

        #rospy.loginfo('Executing state train')

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