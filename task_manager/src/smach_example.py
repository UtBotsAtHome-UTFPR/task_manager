import rospy
import smach
import smach_ros
from std_srvs.srv import Empty

class recognize_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['objects_recognized', 'failed'])        

    def execute(self, userdata):

        rospy.loginfo('Executing state recognize_object')

        rospy.wait_for_service('/utbots/vision/detection_srv')
        try:
            object_srv = rospy.ServiceProxy('/utbots/vision/detection_srv', Empty)
            resp1 = object_srv()
            return 'objects_recognized'
        
        except rospy.ServiceException as e:

            return 'failed'
        

def main():

    rospy.init_node('recognize_object_example')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('RECOGNIZE_OBJECT', recognize_object(), 
                               transitions={'objects_recognized':'done',
                                            'failed':'failed'})

    sis = smach_ros.IntrospectionServer('visu_recognize_sm', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    print (outcome)

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()