import rospy
import smach
from std_msgs.msg import String
import os
import smach_ros


# Wait for someone to show up, possibly voice queue or find a face in the image: 1 outcome

# Run new face routine: if failed for whatever reason go to previous step and tell the user it failed

# Run training: If failed try again, if fail twice go back one step

# Tell the user to go to the crowd and tell the user what is the voice queue to start looking for him, and to repeat in case the robot doesn't start imediatly: 1 outcome after voice queue

# TURN AROUND EVERY NOW AND THEN i GET A LITTLE BIT LONELY: 1 outcome

# Run recognition
# Only correct when only one person is identified as something other than unknown, check for that: 1 outcome

# Save the recognized image: Not really a state but will have to do

# Display the user: 1 outcome

# Display how many people were recognized (ask yolo or look for how many faces there are, yolo should be more reliable): DONE


# define state Foo
class waiting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop_waiting'])
        
        self.pub_instructions = rospy.Publisher("/robot_speech", String, queue_size=1)

        #rospy.init_node('waiting_state', anonymous=True)

        self.loopRate = rospy.Rate(30)

        self.done_waiting = False
 
        #self.pub_instructions.publish("I am now waiting, please input a message to begin")

    def execute(self, userdata):
        rospy.loginfo('Executing state waiting')
        
        while rospy.is_shutdown() == False:
            # Controls speed
            self.loopRate.sleep()

            self.done_waiting = False if input() == "" else True

            if self.done_waiting:
                return 'stop_waiting'
            


# define state Bar
class new_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['new_face_trained', 'cancelled'])
        
        # Create a subscriber to a topic new_face makes to publish a done message

        # Use os.system("command") to launch new_face (possibly change it to launch files, check how to)

        

    def execute(self, userdata):

        # Wait for new_face to send a ok

        rospy.loginfo('Executing state new_face')
        return 'new_face_trained'
        return 'cancelled'
        



# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])
    sm.userdata

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAITING', waiting(), 
                               transitions={'stop_waiting':'NEW_FACE'})
        smach.StateMachine.add('NEW_FACE', new_face(), 
                               transitions={'new_face_trained':'finished',
                                            'cancelled':'finished'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()