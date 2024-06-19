import rospy
import smach
import smach_ros
from std_srvs.srv import Empty

#for now, just a sketch


# This state will listen and recognize which of the questions is
class listen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['question_understood', 'failed'])        

    def execute(self, userdata):

        rospy.loginfo('Executing state listen')

        #maybe will be another service
        rospy.wait_for_service('/utbots/voice/listen_stt')
        try:
            object_srv = rospy.ServiceProxy('/utbots/voice/listen_stt', Empty)
            resp1 = object_srv()
            return 'question_understood'
        
        except rospy.ServiceException as e:
            return 'failed'
        
# This state will convert TTS of the answer to the question detected
class answer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['question_answered', 'failed'])        

    def execute(self, userdata):

        rospy.loginfo('Executing state answer')

        #maybe will be another service
        rospy.wait_for_service('/utbots/voice/answer_tts')
        try:
            object_srv = rospy.ServiceProxy('/utbots/voice/answer_tts', Empty)
            resp1 = object_srv()
            return 'question_answered'
        
        except rospy.ServiceException as e:
            return 'failed'
        

def main():

    rospy.init_node('answer_question')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LISTEN', listen(), 
                               transitions={'question_understood':'ANSWER',
                                            'failed':'failed'})
        smach.StateMachine.add('ANSWER', listen(), 
                               transitions={'question_answered':'done',
                                            'failed':'failed'})

    sis = smach_ros.IntrospectionServer('visu_recognize_sm', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH 6 times, because are 6 questions
    i = 6
    while(i != 0):
        outcome = sm.execute()
        print(outcome)          #regardless of outcome, proceeds to next question
        i -= 1
        

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()