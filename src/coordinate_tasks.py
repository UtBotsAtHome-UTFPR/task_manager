#!/usr/bin/env python3
from time import sleep
import rospy
from std_msgs.msg import String
from custom_msg.msg import set_angles

# Claw control
msg_setAngle = set_angles()
msg_setAngle.set_Kp_GAR = 0.3
#

class Manager:
    def __init__(self):
        rospy.init_node('tsk_manager', anonymous=True)

        self.sub_voice_commands = rospy.Subscriber('/voice_commands', String, self.callback, queue_size=1)
        self.pub_manager_commands = rospy.Publisher('/manager_commands', String, queue_size=1)
        self.pub_resposta = rospy.Publisher('/tts', String, queue_size=1)

        self.loop()
        
    def CarryMyLuggagePt1(self):
        self.pub_manager_commands.publish("Open")
        # Claw control
        msg_setAngle.set_GAR = 180
        self.pub_setAngle.publish(msg_setAngle)
        # 
        self.pub_resposta.publish("Ok. Put the bag in my hand, i will close it in 5 seconds")
        sleep(9)
        self.pub_manager_commands.publish("Close")
        # Claw control
        msg_setAngle.set_GAR = 120
        self.pub_setAngle.publish(msg_setAngle)
        #
        self.pub_resposta.publish("Be ready, I will start following you in 5 seconds. When we reach the car, call my name, wait and say stop now")
        sleep(10)
        self.pub_resposta.publish("Following you")
        self.pub_manager_commands.publish("Follow me")

    def CarryMyLuggagePt2(self):
        self.pub_resposta.publish("I will stop now, bruh. I will open my hand, pick up your bag, please")
        self.pub_manager_commands.publish("Stop")
        sleep(5)
        self.pub_manager_commands.publish("Open")
        # 
        msg_setAngle.set_GAR = 180
        self.pub_setAngle.publish(msg_setAngle)
        # 

    # Maps the task function name according to the voice command received
    task_by_command = {
        "Go":CarryMyLuggagePt1,
        "Stop":CarryMyLuggagePt2
        }

    def callback(self, data):
        self.task_by_command.get(data.data)(self) # Collects the task name and calls the respective function
        self.pub_resposta.publish("Callback")

    def loop(self):
        rospy.spin()


if __name__ == '__main__':
    Manager()
