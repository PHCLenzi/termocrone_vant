import rospy
import math
import csv


class Trajectory_Manager():
    #'/home/phz/catkin_ws/src/joystick_command_pkg/scripts/coordinates_list.csv'

    def __init__(self,coordinates_list_address) -> None:
        self.coordinates_list_address = coordinates_list_address

    def write_new_coordinate(self,new_coordinate):
        pass

    def print_all_saved_coordinates(self):
         with open(self.coordinates_list_address, newline='') as f:
            reader = csv.reader(f)
            for row in reader:
                for coord in row:
                    rospy.loginfo("print of one row csv = %s"%coord)


    def delete_last_coordinate():
        pass
    
    def main(): 
        return
    ''' this function is for when I run this code without being 
     called by another code, that is, the function the main 
     function will execute.'''
if __name__ == '__main__':
    try:
        trajectory_manger = Trajectory_Manager("")
    except KeyboardInterrupt:
        exit()