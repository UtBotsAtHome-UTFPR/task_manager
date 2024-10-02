#!/usr/bin/venv_utbots_tasks/bin/python

import rospy
import smach
import smach_ros
import actionlib
from cv_bridge import CvBridge
import cv2
from utbots_actions.msg import InterpretNLUAction, InterpretNLUGoal
from smach_ros import SimpleActionState
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch

# Initialize CvBridge to convert ROS image message to OpenCV image
bridge = CvBridge()

# This state will send a nav goal to the place the objects are located
# class go_to_shelf(smach.State):
    # def __init__(self):
    #     smach.State.__init__(self, 
    #                         outcomes=['reached', 'all_answered','aborted'],
    #                         input_keys=['waypoint'])

    # def execute(self, userdata):

    #     rospy.loginfo('Executing state question_counter')

    #     if userdata.question_counter >= number_of_questions:
    #         return 'all_answered'

    #     try:
    #         userdata.question_counter_out = userdata.question_counter + 1
    #         return 'action_understood'
        
    #     except rospy.ROSInterruptException:
    #         return 'aborted'

# This state will save the labeled image with objects detected by YOLOv8
class detection_log(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['log_saved', 'aborted'],
                            input_keys=['labeled_img', 'bboxes'])        

    def execute(self, userdata):
        
        rospy.loginfo('Executing state get_objects_bbox')

        try:
            log_img = userdata.labeled_img
            bboxes = userdata.bboxes.bounding_boxes

            # Convert the ROS Image message to an OpenCV image
            cv_image = bridge.imgmsg_to_cv2(log_img, desired_encoding="bgr8")

            # Save the image as a temporary file (as reportlab requires an image file)
            image_filename = "/tmp/log_img.jpg"
            cv2.imwrite(image_filename, cv_image)

            current_time = rospy.Time.now()

            # Create a new PDF file
            pdf_file = f"/home/laser/catkin_ws/src/utbots_tasks/task_manager/logs/object_recognition_manipulation_log_{current_time}.pdf"
            c = canvas.Canvas(pdf_file, pagesize=A4)

            # Insert an image (you can adjust the image size by scaling)
            image_path = "/tmp/log_img.jpg"
            c.drawImage(image_path, x=100, y=600, width=4*inch, height=3*inch)

            # Add some text
            c.setFont("Helvetica", 12)

            text_height = 800
            for bbox in bboxes:
                text = f"- {bbox}"
                c.drawString(100, text_height, text)
                text_height += 200

            # Finalize the PDF file
            c.showPage()
            c.save()
            
            return 'log_saved'
        
        except rospy.ROSInterruptException:
            return 'aborted'

# outcomes=['detected', 'no_detected'],

def main():

    rospy.init_node('smach_answer_questions', anonymous=True)

    global log
    global question_number
    log = str('')
    question_number = 1

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
                                            'aborted': 'ANSWERS_LOG',
                                            'preempted': 'ANSWERS_LOG'},
                               remapping={'NLUInput': 'nlu_input', 
                                          'NLUOutput': 'nlu_output'})

        # State that logs the answers
        smach.StateMachine.add('ANSWERS_LOG', 
                               answers_log(), 
                               transitions={'log_saved': 'QUESTION_COUNTER',
                                            'aborted': 'failed'},
                               remapping={'nlu_input': 'nlu_input', 
                                          'nlu_output': 'nlu_output'})

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
    pdf.output(f"/home/laser/catkin_ws/src/utbots_tasks/task_manager/logs/answer_questions_log_{current_time}.pdf")

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()
