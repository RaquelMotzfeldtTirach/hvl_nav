#!/usr/bin/env python3

import rospy
import actionlib

from my_husky_messages.msg import MovePathAction, MovePathActionFeedback, MovePathActionResult, MovePathActionGoal
from std_msgs.msg import String

#Importing classes from custom library
from lib import FileHandler, PathCreator, MoveBaseClient, PathVizualiser, PathPlanner



# Class for the action server
class ActionServer:
    
    _feedback = MovePathActionFeedback
    _result = MovePathActionResult
    
    def __init__(self, package_name: str, file_name: str, publisher: rospy.Publisher=rospy.Publisher('/action_server', String, queue_size=1)):
        
        # Saves the necessary information for reading from file
        self.package_name = package_name
        self.file_name = file_name
        
        # Initialize the action server
        self.server = actionlib.SimpleActionServer('start_path', MovePathAction, execute_cb=self.start_action, auto_start=False)
        self.server.start()
        
    # The main function of the action server
    # Handles the input from the user, and acts accordingly
    # Goal 1: Move to the GPS coordinates supplied by the user
    # Goal 2: Follow the path planned by the PathPlanner
    def start_action(self, goal: MovePathActionGoal):
        
        
        
        if goal.input == 1:
            self.move_to_gps_coordinates()
        elif goal.input == 2:
            self.sweeping_coverage_path_follower()
            
        self._result.returned_result = "Success"
        self.server.set_succeeded(self._result)

    # This function moves the robot to the supplied GPS coordinates.
    def move_to_gps_coordinates(self):
        
        # 1. Read the GPS coordinates from file
        file_handler = FileHandler()
        
        gps_coordinates = file_handler.read_csv_file(self.package_name, self.file_name)
        
        # 2. Generate the path based on the GPS coordinates
        path_creator = PathCreator(target_frame='utm', segment_length=2.0)
        
        goal = path_creator.generate_path(gps_coordinates)
        
        # 3. Send the path to the action client
        move_base_client = MoveBaseClient()
        
        # vizualise the path
        # Uncomment the following lines to vizualise the path
        # This is mostly only for debugging purposes
        # path_vizualiser = PathVizualiser()
        # path_vizualiser.vizualise(goal)
        
        if len(goal) == 1:
            move_base_client.move_to_point(goal[0])
        else:
            move_base_client.follow_path(goal)
        
    # This function makes the robot follow the path planned by the PathPlanner    
    def sweeping_coverage_path_follower(self):
        
        # 1. Read the GPS coordinates from file
        file_handler = FileHandler()
        
        gps_coordinates = file_handler.read_csv_file(self.package_name, self.file_name)
        
        # 2. Generate the path
        path_creator = PathCreator(target_frame="utm", segment_length=2.0)
        
        goals = path_creator.generate_path_sweeping_coverage(gps_coordinates)
        
        
        # 3. Send the goals to the action client
        move_base_client = MoveBaseClient()
        move_base_client.follow_path(goals)
        
        
        # Vizualise the path
        # Uncomment the following lines to vizualise the path
        #path_vizualiser = PathVizualiser()
        #path_vizualiser.vizualise(goals)
        
        
        


if __name__ == '__main__':
    rospy.init_node('web_navigation_control', anonymous=True)
    
    # Names the package and file name needed for this application
    package_name = 'gps_user_service'
    file_name = '/gps_coordinates.txt'
    
    action_server = ActionServer(package_name=package_name, file_name=file_name)
    
    rospy.spin()