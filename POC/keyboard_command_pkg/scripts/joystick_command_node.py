#! /usr/bin/env python
# Import ROS.
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *

#from joy import 


def main():
    # Initializing ROS node.
    rospy.init_node("joystick_drone_controller", anonymous=True)

    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU(flight controller unit) connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    #drone.wait4start()
    drone.set_mode("GUIDED")

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

     # Specify some waypoints
    goals = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0],
             [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]
    i = 0

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        rate.sleep()
        if drone.check_waypoint_reached():
            i += 1

   
    # Land after all waypoints is reached.
    rospy.loginfo("Flight Mode changing!")
    drone.set_mode("AUTO")#SITL can't change to AUTO mode
    # drone.set_mode("ALT_HOLD")#was a little better, but stil falling
    # drone.set_mode("STABILIZE")#fall
    #drone.set_mode("ALTHOLD")#do not work
    # drone.set_mode("LOITER") #fall
    #(original)drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)
    rospy.spin()


def joy_subscriber():
   
    sub = rospy.Subscriber('string_publisher_topic_name',String, joy_callback_funtion)

def joy_callback_function(msg):

   


if __name__ == '__main__':
    try:
        joy_subscriber()
        main()
    except KeyboardInterrupt:
        exit()