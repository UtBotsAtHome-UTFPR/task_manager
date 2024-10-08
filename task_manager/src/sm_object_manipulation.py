#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import actionlib
from cv_bridge import CvBridge
import cv2
from utbots_actions.msg import YOLODetectionAction, YOLODetectionGoal, Extract3DPointAction, Extract3DPointGoal
from smach_ros import SimpleActionState
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch

global bboxes
bboxes = None

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
            global bboxes

            log_img = userdata.labeled_img
            bboxes = userdata.bboxes.bounding_boxes

            # Convert the ROS Image message to an OpenCV image
            cv_image = bridge.imgmsg_to_cv2(log_img)

            # Save the image as a temporary file (as reportlab requires an image file)
            image_filename = "/tmp/log_img.jpg"
            cv2.imwrite(image_filename, cv_image)

            current_time = rospy.Time.now()

            # Create a new PDF file
            pdf_file = f"/home/laser/catkin_ws/src/utbots_tasks/task_manager/logs/object_recognition_manipulation_log_{current_time}.pdf"
            c = canvas.Canvas(pdf_file, pagesize=A4)

            # Insert an image (you can adjust the image size by scaling)
            image_path = "/tmp/log_img.jpg"
            c.drawImage(image_path, x=85, y=440, width=6*inch, height=4.5*inch)

            # Add some text
            c.setFont("Helvetica", 12)

            text_height = 420
            c.drawString(100, text_height, "Objects recognized:")
            text_height -= 15

            for bbox in bboxes:
                text = f"- {bbox.Class}"
                c.drawString(100, text_height, text)
                text_height -= 12

            # Finalize the PDF file
            c.showPage()
            c.save()
            
            return 'log_saved'
        
        except rospy.ROSInterruptException:
            return 'aborted'
        
# This state selects the object for manipulation
class select_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['get_object_point', 'all_objects', 'aborted'],
                            output_keys=['selected_bbox'])

    def execute(self, userdata):

        rospy.loginfo('Executing state select_object')

        global bboxes

        if bboxes == []:
            return 'all_objects'

        try:
            userdata.selected_bbox = bboxes[0]
            bboxes = bboxes[1:]
            return 'get_object_point'
        
        except rospy.ROSInterruptException:
            return 'aborted'
        
# This state exctracts the 3D point for a given object
class get_object_point(smach_ros.SimpleActionState):
    def __init__(self):
        smach_ros.SimpleActionState.__init__(self, 'extract_3d_point', Extract3DPointAction, 
                                             goal_cb=self.goal_callback, 
                                             result_slots=['Point', 'Success'],
                                             input_keys=['selected_bbox'],
                                             output_keys=['Point'])

    def goal_callback(self, userdata, goal):
        # Use the goal_data from previous state to set the goal for the action
        goal.Object_bbox = userdata.selected_bbox
        return goal

def main():

    rospy.init_node('sm_object_manipulation', anonymous=True)

    client_yolo = actionlib.SimpleActionClient('YOLO_detection', YOLODetectionAction)
    client_yolo.wait_for_server()

    rospy.loginfo("[TASK] YOLO Detection server up")

    client_ext = actionlib.SimpleActionClient('extract_3d_point', Extract3DPointAction)
    client_ext.wait_for_server()

    rospy.loginfo("[TASK] Extract 3D Point server up")

    rospy.loginfo("[TASK] Initiating Task Object Recognition and Manipulation")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    # Define the goal for the action client
    # nav_goal = 
    yolo_goal = YOLODetectionGoal()
    yolo_goal.TargetCategory.data = "bottle"

    # Open the container
    with sm:
        # State that goes to the shelf
        # smach.StateMachine.add('GO_TO_SHELF', 
        #                         SimpleActionState('go_to_shelf',
        #                                             , 
        #                                             goal=nav_goal,
        #                                             result_slots=[]),
        #                         transitions={'succeeded': 'ANSWERS_LOG',
        #                                         'aborted': 'ANSWERS_LOG',
        #                                         'preempted': 'ANSWERS_LOG'},
        #                         remapping={'NLUInput': 'nlu_input', 
        #                                   'NLUOutput': 'nlu_output'})

        # State that gets the object detections from YOLO
        smach.StateMachine.add('GET_DETECTION', 
                               SimpleActionState('YOLO_detection',
                                                 YOLODetectionAction, 
                                                 goal=yolo_goal,
                                                 result_slots=['LabeledImage', 'DetectedObjs', 'Success']),
                               transitions={'succeeded': 'DETECTION_LOG',
                                            'aborted': 'failed',
                                            'preempted': 'DETECTION_LOG'},
                               remapping={'LabeledImage': 'labeled_img', 
                                          'DetectedObjs': 'bboxes'})
        
         # State that logs the detection
        smach.StateMachine.add('DETECTION_LOG', 
                               detection_log(), 
                               transitions={'log_saved': 'SELECT_OBJECT',
                                            'aborted': 'failed'},
                               remapping={'labeled_img': 'labeled_img', 
                                          'bboxes': 'bboxes'})
        
        # State that logs that selects the current object for manipulation
        smach.StateMachine.add('SELECT_OBJECT', 
                               select_object(), 
                               transitions={'all_objects' : 'done',
                                            'get_object_point': 'GET_OBJECT_POINT',
                                            'aborted': 'failed'},
                               remapping={'selected_bbox' : 'selected_bbox'})
        
        # State that extracts the 3D point for a given object
        smach.StateMachine.add('GET_OBJECT_POINT', 
                               get_object_point(),
                               transitions={'succeeded': 'SELECT_OBJECT', #'PICK_OBJECT',
                                            'aborted': 'failed',#'PICK_OBJECT',
                                            'preempted': 'SELECT_OBJECT'},#'PICK_OBJECT'},
                               remapping={'selected_bbox': 'selected_bbox', 
                                          'Point': 'point'})

        # # State that executes the object manipulation
        # smach.StateMachine.add('PICK_OBJECT', 
        #                        answers_log(), 
        #                        transitions={'pick_up_object': 'select_object',
        #                                     'aborted': 'failed'},
        #                        remapping={'nlu_input': 'nlu_input', 
        #                                   'nlu_output': 'nlu_output'})

    sis = smach_ros.IntrospectionServer('object_manipulation', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()
    print(outcome)

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()
