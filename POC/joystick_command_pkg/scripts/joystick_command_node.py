#! /usr/bin/env python
# Import ROS.

import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *
import math
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import csv
from Trajectory_Manager import Trajectory_Manager


from sensor_msgs.msg import Joy # to know batter $rosmsg show sensor_msgs/Joy

class Command_Joystick:
   

    def __init__(self):
        # Create an object for the API.
        self.drone = gnc_api()
        self.set_initial_glogal_variables()

        # Initializing ROS node.
        rospy.init_node("joystick_drone_controller", anonymous=True)

        # Initializing subscribing into joy topic.
        sub = rospy.Subscriber("joy", Joy, self.joy_callback_function)
        self.trajectory_manager = Trajectory_Manager(self)
        rospy.loginfo("Trajectoy manager started")

    
        # Wait for FCU(flight controller unit) connection.
        self.drone.wait4connect()
        # Wait for the mode to be switched.
        
       

        self.drone.set_mode("GUIDED")

        # Create local reference frame.
        self.drone.initialize_local_frame()
        # Request takeoff with an altitude of 3m.
        self.drone.takeoff(3)
        #set the speed of the operation
        speed_operation = 1.5
        self.drone.set_speed(speed_operation)
        rospy.loginfo("the speed operation has been set to %s m/s"%speed_operation)

        # cr = csv.reader(open('/home/phz/catkin_ws/src/joystick_command_pkg/scripts/coordinates_list.csv',"rb"))
        # list_test = list(cr)
       

        self.current_drone_flight_mode = "GUIDED"
        # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
        self.rate = rospy.Rate(3)
        #niting the variables target_position
        self.current_drone_position_heading = self.drone.get_current_heading()
        self.current_drone_position.x = self.drone.get_current_location().x 
        self.current_drone_position.y = self.drone.get_current_location().y
        self.current_drone_position.z = self.drone.get_current_location().z
        self.target_drone_position_heading = self.current_drone_position.z
        
        rospy.spin()
        

    def joy_callback_function(self,msg):
        
        self.current_drone_position_heading = self.drone.get_current_heading()
        self.current_drone_position.x = self.drone.get_current_location().x 
        self.current_drone_position.y = self.drone.get_current_location().y
        self.current_drone_position.z = self.drone.get_current_location().z
        
        
       # condition to percept de axes changes
        if(self.last_joystick_state_axes != msg.axes):
            self.last_joystick_state_axes = msg.axes
            

        #command forward and backward # right stick front and back axes
            if((msg.axes[1]>0.9 or msg.axes[1]<-0.9)or(msg.axes[0]>0.9 or msg.axes[0]<-0.9)):
                self.go_forward_and_backward(msg.axes)
        #command to go to next/previous coordinates saved # right stick with analog button on simple usb controller
            if(msg.axes[3]>0.9 or msg.axes[3]<-0.9):
                self.go_next_previous_coord_saved(msg.axes)
            

        elif(self.last_joystick_state_buttons != msg.buttons):
            self.last_joystick_state_buttons = msg.buttons

            #command to land # "2" button on simple usb controller # "B" button on xbox controller
            if(msg.buttons[1] == 1):
                if(self.current_drone_flight_mode == "GUIDED"):
                    rospy.loginfo("Land command!")
                    self.drone.set_mode("LAND")
                    self.current_drone_flight_mode = "LAND"
                elif(self.current_drone_flight_mode == "LAND"):
                    rospy.loginfo("takeoff command!")
                    self.drone.set_mode("GUIDED")
                    # Request takeoff with an altitude of 3m.
                    self.drone.takeoff(3)
                    self.current_drone_flight_mode = "GUIDED"
            #  print current informations position # button "4" on simple usb controller 
            if(msg.buttons[3] == 1):
                rospy.loginfo("Curretn location = %s " %self.current_drone_position )
                rospy.loginfo("Curretn heading = %s"  %self.current_drone_position_heading)

            #command go home "set_pos(0,0,0,0)"# button "1" on simple usb controller
            if(msg.buttons[0] == 1):
                rospy.loginfo("Go home command!")
                self.set_destination(0.0, 0.0, 3.0, 0.0)
            #command turn left or right # "bunttons" L1 and "R1" on simple usb controller
            if(msg.buttons[4] == 1 or msg.buttons[5] == 1 ):
                if(msg.buttons[4] == 1):
                    rospy.loginfo("Turn left command")
                    self.change_yaw("LEFT")
                if(msg.buttons[5] == 1):
                    rospy.loginfo("Turn right command")
                    self.change_yaw("RIGHT")
            #Command a test square mission # button "4" on simple usb controller
            if(msg.buttons[2] == 1):
               
                self.square()
            #command to change the level of operation # buttons "L2" and "R2" on simple usb controller
            if(msg.buttons[6] == 1 or msg.buttons[7] == 1 ):
                rospy.loginfo("change the level of operation")
                if(msg.buttons[6] == 1):
                    self.change_level_operation("UP")
                elif(msg.buttons[7] == 1):
                    self.change_level_operation("DOWN")
            #Command to add a new coordinate to the list of interest coordinates # button "select" on simple usb controller
            if(msg.buttons[8] == 1):
                new_coorditane = [self.drone.get_current_location().x ,self.drone.get_current_location().y ,self.drone.get_current_location().z ,self.drone.get_current_heading()]
                self.trajectory_manager.save_new_coordinate(new_coorditane)
                
        
  
    def go_next_previous_coord_saved(self,axes):
        #go to the next
        # this function sees if the straight stick went forward and if the current 
        # position in the list of saved coordinates is the last
        current_list_pos = self.trajectory_manager.current_list_coordinate_position_destiny
        coord_list = self.trajectory_manager.list_coordinates_interest
        if (axes[3]>0 and current_list_pos != len(coord_list)):
            self.trajectory_manager.current_list_coordinate_position_destiny+=1
            rospy.loginfo("Going to coordinate {} of {}".format(current_list_pos+1,len(current_list_pos)))
            self.set_destination(coord_list[current_list_pos][0], coord_list[current_list_pos][1],coord_list[current_list_pos][2],coord_list[current_list_pos][3])
        #go to the previous
        if (axes[3]<0 and current_list_pos != 0):
            self.trajectory_manager.current_list_coordinate_position_destiny-=1
            rospy.loginfo("Going to coordinate {} of {}".format(current_list_pos-1,len(current_list_pos)))
            self.set_destination(coord_list[current_list_pos][0], coord_list[current_list_pos][1],coord_list[current_list_pos][2],coord_list[current_list_pos][3])

    def change_yaw(self,side):
        if(side == "LEFT"):
            self.set_destination(self.target_drone_position.x, self.target_drone_position.y, self.target_drone_position.z, self.target_drone_position_heading+5)
        else:
            self.set_destination(self.target_drone_position.x, self.target_drone_position.y, self.target_drone_position.z, self.target_drone_position_heading-5)

    def change_level_operation(self, direction):
        if(direction == "UP"):
            self.set_destination(self.target_drone_position.x, self.target_drone_position.y, self.target_drone_position.z+0.25, self.target_drone_position_heading)
        elif(direction == "DOWN"):
            self.set_destination(self.target_drone_position.x, self.target_drone_position.y, self.target_drone_position.z-0.25, self.target_drone_position_heading)
    
    def go_forward_and_backward(self,joy_msgs_axes):
        #these fuction send the drone forwarad relativel to the position of that
        
        forward_backward_y = self.target_drone_position.y+ 0.5*joy_msgs_axes[1]*math.cos(math.radians(self.target_drone_position_heading))-0.5*joy_msgs_axes[0]*math.sin(math.radians(self.target_drone_position_heading))
        right_lef_x = self.target_drone_position.x - 0.5*joy_msgs_axes[0]*math.cos(math.radians(self.target_drone_position_heading))-0.5*joy_msgs_axes[1]*math.sin(math.radians(self.target_drone_position_heading))
        
        self.set_destination(right_lef_x,forward_backward_y,self.target_drone_position.z,self.target_drone_position_heading)
        #rospy.Rate(3).sleep()

    # this function make the drone stay static on air
    def hold_drone_position(self):
        rospy.loginfo("Hold position")
        self.set_destination(self.current_drone_position.x,self.current_drone_position.y,self.current_drone_position.z,self.current_drone_position_heading)
    #function to initialize global variables
    def set_initial_glogal_variables(self):
        

        self.last_joystick_state_axes = []
        self.last_joystick_state_buttons = []
        self.current_drone_position = Point()
        self.target_drone_position = Point()
        self.current_drone_flight_mode = ""

        self.current_drone_position.x = 0.0
        self.current_drone_position.y = 0.0
        self.current_drone_position.z = 0.0
        self.current_drone_position_heading = 0.0

        self.target_drone_position.x = 0.0
        self.target_drone_position.y = 0.0
        self.target_drone_position.z = 3.0
        self.target_drone_position_heading = 0.0

        self.list_coordinates_interest_csv_path = '/home/phz/catkin_ws/src/joystick_command_pkg/scripts/coordinates_list.csv'

    # this function is used to ensure that all drone position change orders update target_drone_position variables.
    def set_destination(self,x,y,z,h):
        self.target_drone_position.x = x
        self.target_drone_position.y = y
        self.target_drone_position.z = z
        self.target_drone_position_heading = h


        self.drone.set_destination(x,y,z,h)

    #a squere mission for test
    def square(self):
        rospy.loginfo("Start de square mission")
        goals = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0],
             [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]
        i = 0

        while i < len(goals):
            self.drone.set_destination(
                x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
            rospy.Rate(3).sleep()
            if self.drone.check_waypoint_reached():
                i += 1
        

def main():
    command_joystick = Command_Joystick()# init the subscriber, the publisher and the global variables
    #command_joystick.start()
    return


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()