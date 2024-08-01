import rospy
import smach
import smach_ros
from std_srvs.srv import Empty
import actionlib
from utbots_actions.msg import InterpretNLUAction, InterpretNLUResult
from smach_ros import SimpleActionState

number_of_questions = 6

# This state will listen and recognize which of the questions is
class question_counter(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['action_understood', 'all_answered','failed'],
                            input_keys=['client_in','question_counter'],
                            output_keys=['client_out'])        

    def execute(self, userdata):

        rospy.loginfo('Executing state question_counter')

        if userdata.question_counter >= number_of_questions:
            return 'all_answered'

        try:
            #if client is question to be answered and not a task command
            return 'action_understood'
        
        except rospy.ROSInterruptException:
            return 'failed'
        
# This state will convert TTS of the answer to the question detected
class listen_answer(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            ['question_answered','failed'],
                            input_keys=['client'])        

    def execute(self, userdata):

        rospy.loginfo('Executing state listen_answer')

        question_counter += 1
        try:
            result = userdata.client.get_result()
            return 'question_answered'
        
        except rospy.ROSInterruptException:
            return 'failed'
        

def main():

    rospy.init_node('smach_answer_questions',anonymous=True)

    client = actionlib.SimpleActionClient('interpret_nlu', InterpretNLUAction)
    client.wait_for_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])
    sm.userdata.client = client
    sm.userdata.counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add ('QUESTION_COUNTER', 
                                question_counter(), 
                                transitions={'action_understood':'LISTEN_ANSWER',
                                            'all_answered':'done',
                                            'failed':'failed'},
                                remapping={ 'client_in':'client',
                                            'client_out':'client',
                                            'question_counter':'counter'})
        #not gonna use SimpleActionState() but maybe is better
        smach.StateMachine.add ('LISTEN_ANSWER', 
                                listen_answer(), 
                                transitions={'question_answered':'QUESTION_COUNTER',
                                            'failed':'failed'},
                                remapping={'client':'client'})

    sis = smach_ros.IntrospectionServer('visu_recognize_sm', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    print(outcome)

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()