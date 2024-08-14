import rospy
import smach
import actionlib
from utbots_actions.msg import InterpretNLUAction, InterpretNLUGoal
from smach_ros import SimpleActionState

number_of_questions = 6

# This state will count the questions
class question_counter(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['action_understood', 'all_answered','aborted'],
                            input_keys=['question_counter'],
                            output_keys=['question_counter_out'])        

    def execute(self, userdata):

        rospy.loginfo('Executing state question_counter')

        if userdata.question_counter >= number_of_questions:
            return 'all_answered'

        try:
            userdata.question_counter_out = userdata.question_counter + 1
            return 'action_understood'
        
        except rospy.ROSInterruptException:
            return 'aborted'
        


def main():

    rospy.init_node('smach_answer_questions',anonymous=True)

    client = actionlib.SimpleActionClient('interpret_nlu', InterpretNLUAction)
    client.wait_for_server()


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])
    sm.userdata.counter = 0

    goal = InterpretNLUGoal()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add ('QUESTION_COUNTER', 
                                question_counter(), 
                                transitions={'action_understood':'LISTEN_ANSWER',
                                            'all_answered':'done',
                                            'aborted':'failed'},
                                remapping={ 'question_counter':'counter',
                                            'question_counter_out':'counter'})
        smach.StateMachine.add ('LISTEN_ANSWER', 
                                SimpleActionState('interpret_nlu',
                                                  InterpretNLUAction, 
                                                  goal),
                                transitions={'succeeded':'QUESTION_COUNTER',
                                            'aborted':'failed',
                                            'preempted':'done'})


    outcome = sm.execute()
    print(outcome)

    rospy.spin()

if __name__ == "__main__":
    main()