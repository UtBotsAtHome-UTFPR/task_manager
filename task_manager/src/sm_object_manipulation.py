#!/usr/bin/env python

import rospy
import smach
import smach_ros
import rospkg
import actionlib
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import PoseStamped
from utbots_actions.msg import YOLODetectionAction, YOLODetectionGoal, Extract3DPointAction, Extract3DPointGoal
from smach_ros import SimpleActionState
import yaml
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch

# Initialize CvBridge to convert ROS image message to OpenCV image
bridge = CvBridge()

# Initialize the rospkg instance
rospack = rospkg.RosPack()

# Get the path to the desired package (replace 'your_package' with the actual package name)
package_path = rospack.get_path('task_manager')

def retrieve_waypoint(nametag):
    # Open and read the YAML file
    with open(f"{package_path}/waypoints.yaml", 'r') as file:
        data = yaml.safe_load(file)

    # Find the pose with the given nametag
    for pose_data in data['poses']:
        if pose_data['nametag'] == nametag:
            # Create a PoseStamped message
            pose_stamped = PoseStamped()

            # Fill in the header
            pose_stamped.header.seq = pose_data['header']['seq']
            pose_stamped.header.stamp.secs = pose_data['header']['stamp']['secs']
            pose_stamped.header.stamp.nsecs = pose_data['header']['stamp']['nsecs']
            pose_stamped.header.frame_id = pose_data['header']['frame_id']

            # Fill in the pose (position and orientation)
            pose_stamped.pose.position.x = pose_data['pose']['position']['x']
            pose_stamped.pose.position.y = pose_data['pose']['position']['y']
            pose_stamped.pose.position.z = pose_data['pose']['position']['z']

            pose_stamped.pose.orientation.x = pose_data['pose']['orientation']['x']
            pose_stamped.pose.orientation.y = pose_data['pose']['orientation']['y']
            pose_stamped.pose.orientation.z = pose_data['pose']['orientation']['z']
            pose_stamped.pose.orientation.w = pose_data['pose']['orientation']['w']

            return pose_stamped

# This state will send a nav goal to the place the objects are located
class go_to_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'aborted'],
                            input_keys=['waypoint'])
        self.pub = rospy.Publisher('/bridge_navigation_to_pose/goal', PoseStamped, queue_size=1)

    def execute(self, userdata):

        rospy.loginfo('Executing state go_to_shelf')

        try:
            self.pub.publish(retrieve_waypoint("shelf"))
            return 'succeeded'
        
        except rospy.ROSInterruptException:
            return 'aborted'

        
# This state will save the labeled image with objects detected by YOLOv8
class detection_log(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['log_saved', 'aborted'],
                            input_keys=['labeled_img', 'bboxes'],
                            output_keys=['bboxes'])        

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
        
# This state selects the object for manipulation
class select_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['get_object_point', 'all_objects', 'aborted'],
                            input_keys=['bboxes_in'],
                            output_keys=['selected_bbox', 'bboxes_out'])

    def execute(self, userdata):

        rospy.loginfo('Executing state select_object')

        bboxes = userdata.bboxes_in

        if userdata.bboxes_in != []:
            return 'all_objects'

        try:
            userdata.selected_bbox = bboxes[0]
            userdata.bboxes_out = bboxes.pop(0)
            return 'get_object_point'
        
        except rospy.ROSInterruptException:
            return 'aborted'
        
# This state exctracts the 3D point for a given object
class get_object_point(smach_ros.SimpleActionState):
    def __init__(self):
        smach_ros.SimpleActionState.__init__(self, 'action_server_name', Extract3DPointAction, 
                                             goal_cb=self.goal_callback, 
                                             result_slots=['Point', 'Success'],
                                             input_keys=['selected_bbox'],
                                             output_keys=['Point'])

    def goal_callback(self, userdata, goal):
        # Use the goal_data from previous state to set the goal for the action
        goal.some_goal_field = userdata.selected_bbox
        return goal

def main():

    rospy.init_node('sm_object_manipulation', anonymous=True)

    # client_yolo = actionlib.SimpleActionClient('yolo_detection', YOLODetectionAction)
    # client_yolo.wait_for_server()

    # client_ext = actionlib.SimpleActionClient('extract_3d_point', Extract3DPointAction)
    # client_ext.wait_for_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    # Define the goal for the action client
    # nav_goal = 
    yolo_goal = YOLODetectionGoal()

    # Open the container
    with sm:
        # State that goes to the shelf
        smach.StateMachine.add('GO_TO_SHELF', 
                                go_to_shelf(),
                                transitions={'succeeded': 'GET_DETECTION',
                                            'aborted': 'failed',
                                            'preempted': 'failed'})

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
                               remapping={'bboxes': 'bboxes_in',
                                          'selected_bbox' : 'selected_bbox',
                                          'bboxes_out': 'bboxes'})
        
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

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()