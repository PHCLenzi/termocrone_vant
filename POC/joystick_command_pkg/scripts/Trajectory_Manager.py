import rospy
import math
import csv


class Trajectory_Manager():
    #'/home/phz/catkin_ws/src/joystick_command_pkg/scripts/coordinates_list.csv'

    def __init__(self,controller):
        self.controller = controller

        self.set_initial_global_variables()
        self.get_saved_coordinates_from_csv()
       
       
    '''this function add a new coordinate in the global variable list,
     after that, the csv file,with previous coordinate are updated adding these new coordinate.'''
    def save_new_coordinate(self,new_coordinate):
        self.list_coordinates_interest.append(new_coordinate)
        self.update_interested_coordinates_csv_list()
        
        
    '''this function prints all coordinates saved on csv file '''
    def print_all_saved_coordinates_csv_file(self):
         with open(self.coordinates_list_path, newline='') as f:
            reader = csv.reader(f)
            for row in reader:
                for coord in row:
                    rospy.loginfo("print of one row csv = %s"%coord)

    '''this function prints all coordinates saved on global variable list'''
    def print_coordinates_list(self):
        for coord in self.list_coordinates_interest:
                print("coord = {}".format(coord))
                rospy.loginfo("print of one row csv = %s"%coord)

    '''this function delete the last coordinate of the global variables, after that, the 
    last coordinate of the csv file are deleted too.'''
    def delete_last_coordinate():
        #procura ultima coordenada e deleta ela
        self.update_interested_coordinates_csv_list()

    '''this function initiates some global variables'''
    def set_initial_global_variables(self):
        if(self.controller == ""): 
            self.coordinates_list_path = '/home/phz/catkin_ws/src/joystick_command_pkg/scripts/coordinates_list.csv'
        else:
            self.coordinates_list_path = self.controller.list_coordinates_interest_csv_path
        self.list_coordinates_interest = []
        #this variable defines which of the coordinates in the list of saved coordinates 
        #the drone should go to in case the order to go to the "next coordinate"
        self.current_list_coordinate_position_destiny = 0
    ''' this function read the csv external file, which contains the saved coordinates,
     and load these coordinates into global variables'''
    def get_saved_coordinates_from_csv(self):
        with open(self.coordinates_list_path, newline='') as f:
            reader = csv.reader(f)
            n=1
            for row in reader:
                coordinate = []
                for coord in row:
                    coordinate.append(float(coord))
                self.list_coordinates_interest.append(coordinate)
                print("trajectory_manager/The coordinate {} = {}".format(n,coordinate))
                n+=1
    '''' this function rewrite all the coordinates of the global variable, which contains 
    the current interest coordinates list, into the external csv file'''
    def update_interested_coordinates_csv_list(self):
        with open(self.coordinates_list_path, mode='w') as f:
            writer = csv.writer(f)
            for row in self.list_coordinates_interest:
                writer.writerow(row)

            # writer.writerow([8.4, 7.4, 6.4, 20.4])
            # writer.writerow([9.4, 10.4, 11.4, 12.4])
        # atualizar o arquivo CSV com os elementos da self.list_coordinates_interest
        pass


    def test_function(self):
        # with open(self.coordinates_list_path, newline='') as f:
        #     reader = csv.reader(f)
        #     for row in reader:
        #         print("print of one row csv = %s"%str(row))
        self.save_new_coordinate([4,4,5,0])
        self.print_coordinates_list()
    def main(): 
        return
    ''' this function is for when I run this code without being 
     called by another code, that is, the function the main 
     function will execute.'''

if __name__ == '__main__':
    try:
        trajectory_manger = Trajectory_Manager("")
        trajectory_manger.test_function()

    except KeyboardInterrupt:
        exit()