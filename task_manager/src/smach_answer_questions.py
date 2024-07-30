import rospy
import smach
import smach_ros
from std_srvs.srv import Empty

#for now, just a sketch
number_of_questions = 6
question_counter = 0

# This state will listen and recognize which of the questions is
class listen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['question_understood', 'all_answered','failed'])        

    def execute(self, userdata):

        rospy.loginfo('Executing state listen')

        if question_counter >= number_of_questions:
            return 'done'

        try:
            listen_srv = rospy.ServiceProxy('/utbots/voice/listen_stt', Empty)
            resp1 = listen_srv()
            return 'question_understood'
        
        except rospy.ServiceException as e:
            return 'failed'
        
# This state will convert TTS of the answer to the question detected
class answer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['question_answered', 'failed'])        

    def execute(self, userdata):

        rospy.loginfo('Executing state answer')

        question_counter += 1
        try:
            answer_srv = rospy.ServiceProxy('/utbots/voice/answer_tts', Empty)
            resp1 = answer_srv()
            return 'question_answered'
        
        except rospy.ServiceException as e:
            return 'failed'
        

def main():

    rospy.init_node('answer_question')
    #client = actionlib.SimpleActionClient('bla', blabla)
    #client.wait_for_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LISTEN', listen(), 
                               transitions={'question_understood':'ANSWER',
                                            'all_answered':'done',
                                            'failed':'failed'})
        smach.StateMachine.add('ANSWER', answer(), 
                               transitions={'question_answered':'LISTEN',
                                            'failed':'failed'})

    sis = smach_ros.IntrospectionServer('visu_recognize_sm', sm, '/SM_ROOT')
    sis.start()

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()