#!/usr/bin/env python
#fonte deste vídeo: https://www.youtube.com/watch?v=uip2BbaazjU&t=103s
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def publisher():

    pub = rospy.Publisher('string_publisher_topic_name',String, queue_size= 10)
    rate = rospy.Rate(1)#frenquency of publication

    msg_to_publish = String()

    counter = 0

    while not rospy.is_shutdown():
        string_to_publish = "Publishing %d" %counter
        counter += 1

        msg_to_publish.data = string_to_publish# put the contents into the message 
        pub.publish(msg_to_publish)

        rospy.loginfo(string_to_publish)

        rate.sleep()

def move():
    #fonte http://wiki.ros.org/turtlesim/Tutorials/Moving%20in%20a%20Straight%20Line

    # Starts a new node
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twistRate = rospy.Rate(1)#frenquency of publication

    vel_msg = Twist()

   
    #Since we are moving just in x-axis
    vel_msgLinearY = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():
        vel_msg.linear.y = vel_msgLinearY
        vel_msgLinearY += 0.01

        velocity_publisher.publish(vel_msg)
        rospy.loginfo("Publisher vel_msgLinearY = %f"%vel_msg.linear.y )
        twistRate.sleep()
   

if __name__ == "__main__":
    rospy.init_node("cammand_node")
    #publisher()
    move()
