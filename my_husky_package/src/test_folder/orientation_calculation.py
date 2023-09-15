#!/usr/bin/env python3
import copy
import time

import rospy
import rospkg
import actionlib
import utm

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler 
import tf.transformations
import math
from math import pi
import numpy as np
import matplotlib.pyplot as plt

class orientatation_calculation:
    
    def __init__(self):
        
        self.path_no_orientation = self.get_path_from_file()
        self.path_with_orientation = list()
        self.calculate_orientations_of_path()
        
        #Creating a publisher, to vizualise the data
        self.pub = rospy.Publisher('orientation_vizualiser', MoveBaseGoal, queue_size=10)
        
        
        
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        print("Initializing client")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print("Client Initializesd")
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        print("Connected to server")
        
        
    
    def get_path_from_file(self):
        
        package_name = "my_husky_package"
        file_name = "/src/test_folder/sweeping_path.txt"
        
        try:
            rp = rospkg.RosPack()
            package_path = rp.get_path(package_name)
        except rospkg.common.ResourceNotFound as e:
            rospy.logerr(f"Package not found: {e}")
            return None
        
        file_path = package_path + file_name
        
        path_point_list = []
        
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    values = line.strip().split(',')
                    path_points_tuple = (float(values[0]), float(values[1]))
                    path_point_list.append(path_points_tuple)
        except FileNotFoundError as e:
            rospy.logerr(f"File not found: {e}")
            return None
        
        return path_point_list
      
    def get_quaternions(self, start_point: list, end_point: list):
        #Take two points and get the quaternions from that.
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        
        angle_rad = math.atan2(dy, dx)
        orientation_rpy = (0, 0, angle_rad)
        
        angle_quat = quaternion_from_euler(*orientation_rpy)
        
        return angle_quat
    
    def calculate_orientations_of_path(self):
        
        for i in range(len(self.path_no_orientation) - 1):
            
            #Extracts the cartesian coordinates from the path.
            x_1 = self.path_no_orientation[i][0]
            y_1 = self.path_no_orientation[i][1]
            
            x_2 = self.path_no_orientation[i + 1][0]
            y_2 = self.path_no_orientation[i + 1][1]
            
            
            start_point = [x_1, y_1]
            end_point = [x_2, y_2]
            
            quaternions = self.get_quaternions(start_point, end_point)
            
            #This is now obsolete, i think.
            
            # if(i == 0):
            #     #For the first goal, we want to navigate to the first starting position
            #     self.path_with_orientation.append(self.create_move_base_goal(quaternions, start_point))
                
            #     #Creates a goal with the starting position and new orientation
            #     self.path_with_orientation.append(self.create_move_base_goal(quaternions, start_point))
                
            #     #Creates a goal with the ending position and new orientation
            #     self.path_with_orientation.append(self.create_move_base_goal(quaternions, end_point))           
            # else:
            #     #Creates a goal with the starting position and new orientation
            #     self.path_with_orientation.append(self.create_move_base_goal(quaternions, start_point))
                
            #     #Creates a goal with the ending position and new orientation
            #     self.path_with_orientation.append(self.create_move_base_goal(quaternions, end_point))
                
            #Create the goal with the position the robot is standing n
            self.add_segment_to_path(quaternions, start_point, end_point)


    def create_move_base_goal(self, quaternions : list, position : list):
        
        move_base_goal = MoveBaseGoal()
        
        move_base_goal.target_pose.header.frame_id = 'utm'
        
        move_base_goal.target_pose.pose.position.x = position[0]
        move_base_goal.target_pose.pose.position.y = position[1]
        
        move_base_goal.target_pose.pose.orientation.x = quaternions[0]
        move_base_goal.target_pose.pose.orientation.y = quaternions[1]
        move_base_goal.target_pose.pose.orientation.z = quaternions[2]
        move_base_goal.target_pose.pose.orientation.w = quaternions[3]
        
        return move_base_goal
    
    def add_segment_to_path(self, quaternions : list, start_point : list, end_point : list):
        
        x_1, y_1 = start_point
        x_2, y_2 = end_point
        
        #Finding the length of this particular segment
        length_of_segment = math.dist(start_point, end_point)
        
        # Equals zero when only rotation is wanted.
        if(length_of_segment > 1):
            
            #The number of segments should equal the lenght of the segment in meters.
            number_of_segments = int(length_of_segment)
            
            #Finding the travel distane along each axis per segmnt
            dx = (x_2 - x_1) / number_of_segments
            dy = (y_2 - y_1) / number_of_segments
            
            
            #Using intepolation, create the segments
            for i in range(number_of_segments):
                x = x_1 + i * dx
                y = y_1 + i * dy
                segment_point = [x, y]
                self.path_with_orientation.append(self.create_move_base_goal(quaternions, segment_point))
                
        self.path_with_orientation.append(self.create_move_base_goal(quaternions, end_point))

        
        
        
        
                
    def execute_path(self):
        for goal in self.path_with_orientation:
            print(f"Sending goal: \n")
            print(f"X: {goal.target_pose.pose.position.x}")
            print(f"Y: {goal.target_pose.pose.position.y} \n")


            # Publishing the goal for visualization.
            self.pub.publish(goal)
            

            # Sending the goal to the move_base action server            
            self.client.send_goal(goal)
            print("Goal sent \n")
            self.client.wait_for_result()
            print("Goal reached")
                
            
                        

            
            
    
    def go_to_position(self, quaternions: list, positions: list):
        quaternions_ = quaternion_from_euler(*quaternions)
        
        move_base_goal_temp = MoveBaseGoal()
        
        move_base_goal_temp.target_pose.header.frame_id = 'utm'
        move_base_goal_temp.target_pose.header.stamp = rospy.Time.now()
        
        move_base_goal_temp.target_pose.pose.orientation.x = quaternions_[0]
        move_base_goal_temp.target_pose.pose.orientation.y = quaternions_[1]
        move_base_goal_temp.target_pose.pose.orientation.z = quaternions_[2]
        move_base_goal_temp.target_pose.pose.orientation.w = quaternions_[3]

        move_base_goal_temp.target_pose.pose.position.x = positions[0]
        move_base_goal_temp.target_pose.pose.position.y = positions[1]
        print(f"X: {positions[0]} \nY: {positions[1]}")
        self.client.send_goal(move_base_goal_temp)
        print("Goal sent \n")
        self.client.wait_for_result()
        print("Goal reached\n")
        
        
    def vizualise_path(self):
    
        path_points = self.path_with_orientation
        
        x_values = list()
        y_values = list()
        
        orientations = list()
        orientations_from_quaternions = list()            
        
        for i in range(len(path_points)):
            x_values.append(self.path_with_orientation[i].target_pose.pose.position.x)
            y_values.append(self.path_with_orientation[i].target_pose.pose.position.y)
            
            x = self.path_with_orientation[i].target_pose.pose.orientation.x
            y = self.path_with_orientation[i].target_pose.pose.orientation.y
            z = self.path_with_orientation[i].target_pose.pose.orientation.z
            w = self.path_with_orientation[i].target_pose.pose.orientation.w
            
            rpy = tf.transformations.euler_from_quaternion([x, y, z, w])
            yaw = rpy[2]
            
            orientations_from_quaternions.append(math.degrees(yaw))   
            
        for i in range(len(x_values) - 1):
            start_point = [x_values[i], y_values[i]]
            end_point = [x_values[i+1], y_values[i+1]]
            
            orientations.append(self.calculate_orientation_in_degs(start_point, end_point))
        
        fig, ax = plt.subplots()
        ax.plot(x_values, y_values, '-o')
        
        
        
        # for i in range(len(orientations)):
        #     if(orientations[i] != orientations_from_quaternions[i]):
        #         print(f"orientations: {orientations[i]}\norientation Quat: {orientations_from_quaternions[i]} \n\n\n")
        
        for i in range(len(path_points) - 1):
            x = x_values[i]
            y = y_values[i]
            orientation = orientations[i]
            dx = math.cos(math.radians(orientation))
            dy = math.sin(math.radians(orientation))
            ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1, color='r')
        
        # Adjust aspect ratio
        ax.set_aspect('equal')
        
        # Set limits of the plot based on UTM coordinates
        x_min, x_max = min(x_values), max(x_values)
        y_min, y_max = min(y_values), max(y_values)
        ax.set_xlim(x_min - 10, x_max + 10)
        ax.set_ylim(y_min - 10, y_max + 10)
        
        ax.set_xlabel('UTM X')
        ax.set_ylabel('UTM Y')
        plt.show()
    
    def calculate_orientation_in_degs(self, start_point, end_point):
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        
        return angle_deg
        
    
    
    
if __name__ == '__main__':
    
    print("Initializing the node")
    rospy.init_node('orientation_calculation_test')
    print("Node initialized")
    
    orientation_calculator = orientatation_calculation()
    
    
    #orientation_calculator.vizualise_path()
    
    orientation_calculator.execute_path()
    
    #print("here")
    
    
        