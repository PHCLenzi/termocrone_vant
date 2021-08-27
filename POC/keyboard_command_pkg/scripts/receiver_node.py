#!/usr/bin/env python
#fonte deste vídeo: https://www.youtube.com/watch?v=i4l03Idb-rQ
import rospy
from rospy.core import loginfo
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import PySimpleGUI as sg

def subscriber():
    layout = [[sg.Text("Hello from PySimpleGUI")], [sg.Text("OK")]]
    window = sg.Window("Demo", layout)
   
    sub = rospy.Subscriber('string_publisher_topic_name',String, callback_funtion)
    rospy.spin()

def twistSubscriber():

    sub = rospy.Subscriber('cmd_vel',Twist, twist_callback_funtion)
    twistSub =rospy.Subscriber
    rospy.spin()

def callback_funtion(message):
    layout = [[sg.Text("Hello from PySimpleGUI")], [sg.Text("I received: %s"%message.data)]]
    event, values = window.read()
    rospy.loginfo("I received: %s"%message.data)

def twist_callback_funtion(message):
    
    rospy.loginfo("Linear y =  %f"%message.linear.y)



if __name__ == "__main__":
    testeString = 0
    # layout = [[sg.Text("Hello from PySimpleGUI")], [sg.Text("OK")]]
    # window = sg.Window("Demo", layout)
    rospy.init_node("receiver_node")
    twistSubscriber()