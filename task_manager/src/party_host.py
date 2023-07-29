#!/usr/bin/env python3
from time import sleep
import rospy
from std_msgs.msg import String, Bool
from custom_msg.msg import set_angles
from voice_msgs.msg import NLU

# Claw control
msg_setAngle = set_angles()
msg_setAngle.set_Kp_GAR = 0.3
#

def no_task(self):
    self.pub_response.publish("Please repeat.")   

class Manager:
    def __init__(self):
        rospy.init_node('party_host_mgr', anonymous=True)

        self.sub_voice_commands = rospy.Subscriber('/utbots/voice/nlu_msg', NLU, self.callback_commands, queue_size=1)
        self.pub_manager_commands = rospy.Publisher('/utbots/task_manager/manager_commands', String, queue_size=10)
        self.pub_response = rospy.Publisher('/utbots/voice/tts/robot_speech', String, queue_size=1)
        self.sub_enable = rospy.Subscriber("/utbots/vision/faces/enable", Bool, self.callback_enable_face)

        self.msg_enable = Bool()
        self.msg_enable.data = False

        self.loop()

    def PartyHost(self):
        self.pub_manager_commands.publish("register_face")
        sleep(1)
        if self.msg_enable.data == True:
            self.pub_response.publish("Hello David!")
        

    # Maps the task function name according to the voice command received
    task_by_command = {
        "Hello":PartyHost,
        "Go":PartyHost
        }

    def callback_commands(self, data):
        if(data.database.data == "commands"):
            print(data.text)
            self.task_by_command.get(data.text.data, no_task)(self) # Collects the task name and calls the respective function

    def callback_enable_face(self, msg):
        self.msg_enable = msg

    def loop(self):
        rospy.spin()

if __name__ == '__main__':
    Manager()
