import rospy
import smach
import actionlib
from utbots_actions.msg import InterpretNLUAction, InterpretNLUGoal
from smach_ros import SimpleActionState
from std_msgs.msg import String

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

# This state will log the answers
class answers_log(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['log_saved', 'aborted'],
                            input_keys=['nlu_input', 'nlu_output'])        

    def execute(self, userdata):
        rospy.loginfo('Executing state answers_log')

        try:
            question = userdata.nlu_input.data
            answer = userdata.nlu_output.data

            question = question.replace("data: ", "")
            answer = answer.replace("data: ", "")

            rospy.loginfo(f"Question: {question}")
            rospy.loginfo(f"Answer: {answer}")

            question_log = question + '\n' + answer + '\n'
            rospy.loginfo(f"Question log: {question_log}")
            
            return 'log_saved'
        
        except rospy.ROSInterruptException:
            return 'aborted'

def main():

    rospy.init_node('smach_answer_questions', anonymous=True)

    client = actionlib.SimpleActionClient('interpret_nlu', InterpretNLUAction)
    client.wait_for_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])
    sm.userdata.counter = 0

    # Define the goal for the action client
    goal = InterpretNLUGoal()

    # Open the container
    with sm:
        # State that counts the questions
        smach.StateMachine.add('QUESTION_COUNTER', 
                               question_counter(), 
                               transitions={'action_understood': 'LISTEN_ANSWER',
                                            'all_answered': 'done',
                                            'aborted': 'failed'},
                               remapping={'question_counter': 'counter',
                                          'question_counter_out': 'counter'})

        # State that executes the InterpretNLUAction
        smach.StateMachine.add('LISTEN_ANSWER', 
                               SimpleActionState('interpret_nlu',
                                                 InterpretNLUAction, 
                                                 goal=goal,
                                                 result_slots=['NLUInput', 'NLUOutput']),
                               transitions={'succeeded': 'ANSWERS_LOG',
                                            'aborted': 'failed',
                                            'preempted': 'done'},
                               remapping={'NLUInput': 'nlu_input', 
                                          'NLUOutput': 'nlu_output'})

        # State that logs the answers
        smach.StateMachine.add('ANSWERS_LOG', 
                               answers_log(), 
                               transitions={'log_saved': 'QUESTION_COUNTER',
                                            'aborted': 'failed'},
                               remapping={'nlu_input': 'nlu_input', 
                                          'nlu_output': 'nlu_output'})

    # Execute the state machine
    outcome = sm.execute()
    print(outcome)

    rospy.spin()

if __name__ == "__main__":
    main()
