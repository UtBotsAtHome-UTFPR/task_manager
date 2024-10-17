#!/usr/bin/env python3

import rospy
from rospkg import RosPack
import smach
import smach_ros
import actionlib
from utbots_actions.msg import InterpretLlamaAction, InterpretLlamaGoal
from smach_ros import SimpleActionState
from fpdf import FPDF
import os

number_of_questions = 6

# This state will count the questions
class question_counter(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['action_understood', 'all_answered','aborted'],
                            input_keys=['question_counter'],
                            output_keys=['question_counter_out','reset_va_out'])

    def execute(self, userdata):

        rospy.loginfo('Executing state question_counter')

        if userdata.question_counter >= number_of_questions:
            return 'all_answered'

        try:
            userdata.question_counter_out = userdata.question_counter + 1
            userdata.reset_va_out=False#resetar va
            return 'action_understood'
        
        except rospy.ROSInterruptException:
            return 'aborted'

# This state will log the answers
class answers_log(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['log_saved', 'aborted'],
                            input_keys=['LLMInput', 'LLMOutput'])        

    def execute(self, userdata):
        global question_number, log
        
        rospy.loginfo('Executing state answers_log')

        try:
            question = userdata.LLMInput.data
            answer = userdata.LLMOutput.data

            question = question.replace("data: ", "")
            answer = answer.replace("data: ", "")

            rospy.loginfo(f"Question: {question}")
            rospy.loginfo(f"Answer: {answer}")

            question_log =  str(question_number) + '- ' + question + '\n' + answer + '\n'
            rospy.loginfo(f"Question log: {question_log}")

            question_number += 1
            log += question_log
            
            return 'log_saved'
        
        except rospy.ROSInterruptException:
            return 'aborted'
        
class detect_va(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['to_detection','to_answer','aborted'],
                            input_keys=['activated_in','reset_va_in'],
                            output_keys=['set_goal','reset_va_out','activated_out']
                            # input_keys=['activated_in','reset_va_in'],
                            # output_keys=['set_goal','reset_va_out','activated_out']
                            )        

    def execute(self, userdata):

        rospy.loginfo('Executing state question_counter')

        if userdata.reset_va_in == True:
            userdata.set_goal=False
            userdata.reset_va_out = False
            return 'to_detection'
            

        if userdata.activated_in.data == False:
            userdata.set_goal=False
            return 'to_detection'

        try:
            userdata.set_goal=True
            userdata.reset_va_out = True
            return 'to_answer'
        
        except rospy.ROSInterruptException:
            return 'aborted'


def main():

    rospy.init_node('smach_answer_questions', anonymous=False)

    global log
    global question_number
    log = str('')
    question_number = 1

    client = actionlib.SimpleActionClient('llama_inference', InterpretLlamaAction)
    client.wait_for_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])
    sm.userdata.counter = 0
    sm.userdata.reset_va=False#resetar va
    sm.userdata.set_goal=False
    class act():
        def __init__(self):
            self.data=False
    

    sm.userdata.activation=InterpretLlamaAction().action_result.result.Activation
    sm.userdata.activation.data=False
    # Define the goal for the action client
    goal = InterpretLlamaGoal()
    goal.Answer = False


    # Open the container
    with sm:
        def goal_Llama_cb(userdata, goal):
            action_goal=InterpretLlamaGoal()
            action_goal.Answer.data=userdata.set_goal
            print(action_goal.Answer.data)
            
            return action_goal
            
        # def goal_Llama_va_cb(userdata, goal):
        #     action_goal=InterpretLlamaGoal()
        #     action_goal.Answer.data=userdata.set_goal
        #     print(action_goal.Answer.data)
            
        #     return action_goal
        
        # State that counts the questions
        smach.StateMachine.add('QUESTION_COUNTER', 
                               question_counter(), #smach.State class
                               transitions={'action_understood': 'DETECT_VA',
                                            'all_answered': 'done',
                                            'aborted': 'failed'},
                               remapping={'question_counter': 'counter',
                                        #   'activation': 'activation',
                                          'question_counter_out': 'counter'
                                          })
        # State that executes the InterpretNLUAction
        smach.StateMachine.add('REQUEST_ACTIVATION', 
                               SimpleActionState('llama_inference', #smach.State class
                                                 InterpretLlamaAction, 
                                                 goal=InterpretLlamaGoal(),

                                                #  input_keys=['set_goal'],
                                                #  goal_cb=goal_Llama_cb,
                                                 result_slots=['LLMInput', 'LLMOutput', 'Activation']),
                               transitions={'succeeded': 'QUESTION_COUNTER',
                                            'aborted': 'QUESTION_COUNTER',
                                            'preempted': 'QUESTION_COUNTER'},
                               remapping={'Activation':'activation'})
        
        smach.StateMachine.add('DETECT_VA', #to_detection','to_answer','aborted
                                detect_va(), #smach.State class
                                transitions={'to_detection': 'REQUEST_ACTIVATION',
                                            'to_answer': 'LISTEN_ANSWER',
                                            'aborted': 'failed'},
                                remapping={'activated_in': 'activation',
                                            'reset_va_out':'reset_va',
                                            'reset_va_in':'reset_va'})

        # State that executes the InterpretNLUAction
        smach.StateMachine.add('LISTEN_ANSWER', 
                               SimpleActionState('llama_inference', #smach.State class
                                                 InterpretLlamaAction,
                                                 input_keys=['set_goal'],
                                                 goal_cb=goal_Llama_cb,
                                                #  goal=InterpretLlamaGoal(),
                                                result_slots=['LLMInput', 'LLMOutput','Activation']),
                               transitions={'succeeded': 'ANSWERS_LOG',
                                            'aborted': 'ANSWERS_LOG',
                                            'preempted': 'ANSWERS_LOG'},
                               remapping={'LLMInput': 'llm_input', 
                                          'LLMOutput': 'llm_output',
                                          'Activation':'activation'})

        # State that logs the answers
        smach.StateMachine.add('ANSWERS_LOG', 
                               answers_log(), 
                               transitions={'log_saved': 'REQUEST_ACTIVATION',
                                            'aborted': 'failed'},
                               remapping={'LLMInput': 'llm_input', 
                                          'LLMOutput': 'llm_output'})

    sis = smach_ros.IntrospectionServer('answer_questions', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()
    print(outcome)
    print(log)
    log = ''.join(c for c in log if ord(c) < 256)

    # Save log to PDF
    current_time = rospy.Time.now()
    pdf = FPDF()
    pdf.add_page()
    pdf.set_font("Times", size=10)
    pdf.multi_cell(0, 5, log)
    rospack = RosPack()
    pkg_path = rospack.get_path('task_manager')
    dir=pkg_path+"/logs/"
    if not os.path.exists(dir):
        os.makedirs(dir)
    pdf.output(dir+f"answer_questions_log_{current_time}.pdf")#
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()
