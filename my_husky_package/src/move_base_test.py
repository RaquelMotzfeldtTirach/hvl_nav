#!/usr/bin/env python3
import copy

import rospy
import rospkg
import actionlib
import os
import utm
import time
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from my_husky_messages.msg import MovePathActionGoal, MovePathActionResult, MovePathActionFeedback, MovePathAction
from my_husky_messages.srv import GPSToCartesian, GPSToCartesianRequest, GPSToCartesianResponse
from std_msgs.msg import String 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import tf
import math
import tf.transformations

from lib import MoveBaseClient

class ActionServer:
    #Creating the necessary variables for the action server
    _feedback = MovePathActionFeedback
    _result = MovePathActionResult

    def __init__(self):
        #INitializes the action server
        self.server = actionlib.SimpleActionServer('start_path', MovePathAction, execute_cb=self.start_action, auto_start=False)
        self.pub_bug_talker = rospy.Publisher('bug_reporter', String, queue_size=10)

        #Starts the action server
        self.server.start()
        rospy.loginfo("Server is up and running")
        self.current_position = list()


    def start_action(self, goal: MovePathActionGoal):
        #This function starts when the action server is being called
        #In the goal there is an int, this will be used for controll.
        
        self.pub_bug_talker.publish(f"Starting the action server with goal {goal.input}")

        # Goal = 1: Move to the GPS coordinates found in the GPS file
        # Goal = 2: Move the path found in the pathplanner file
        package_name = 'gps_user_service'
        file_name = '/gps_coordinates.txt'
        self.gps_points_list = self.get_data_from_file(package_name, file_name)
        self.pub_bug_talker.publish(f"The list of gps points contains {len(self.gps_points_list)} entries")
        if goal.input == 1:
            self.pub_bug_talker.publish(f"Starting to move to gps points")
            self.move_to_gps_points()
        elif goal.input == 2:
            self.follow_path_planner()
        elif goal.input == 3:
            self.save_data_to_file()
        elif goal.input == 4:
            self.move_to_gps_points_using_cartesian()
        else:
            rospy.loginfo("Action not found")


        self.server.set_succeeded(self._result) #Sets the result of the action.

    
    #This function takes the gps points from the file and moves the robot along the path in straight lines
    def move_to_gps_points(self):

        # Deprecated solution, no need for cartesian coordinates
        #cartesian_coordinates = rospy.ServiceProxy('gps_to_cartesian_service', GPSToCartesian)

        #The order of operations:
        #1. Get the gps points from the file
        #2. Convert the gps points to UTM Coordinates
        #3. Calculate the required orientations
        #4. Move the robot to the points
        
        #1. Get the gps points from the file
        #The gps points are already gathered, and belongs in self.gps_points_list
        
        #2. Convert the gps points to UTM Coordinates
        utm_coordinates = self.get_utm_coordinates(self.gps_points_list)
        
        #3. Calculate the required orientations
        #The orientation is calculated by using the t_1 and t_2 points in the list.
        
        #This publishes the status of the action server, mostly for debugging purposes
        self.pub_bug_talker.publish(f"Path contains {len(utm_coordinates)} entries")
        
        #If there is only one point in the list, another algorithm is used
        if(len(utm_coordinates) == 1):
            path = self.generate_path_single_point(utm_coordinates)
            
        else:
            path = self.generate_path_multiple_points(utm_coordinates)
        
        #4. Move the robot to the points
        rospy.loginfo("Starting to move")
        self.pub_bug_talker.publish(f"Sending the path to the movebase client")
        self.movebase_client(path)
        
        
    def generate_path_single_point(self, end_point: list()):
        #This function generates a path from the current position to the end point
        
        #Order of operations:
        #1. Get the current position
        end_point = [end_point[0][0], end_point[0][1]]
        
        self.pub_bug_talker.publish(f"Getting the current position...")
        
        get_position = GetPosition()
        
        lat, lon = get_position.get_gps_point()
        
        current_position = utm.from_latlon(lat, lon)

        
        #This publishes the status of the action servr, mostly for debugging purposes
        self.pub_bug_talker.publish(f"Current position is {current_position}")
        
        #2. Calculate the orientation
        orientation = self.get_orientation(current_position, end_point)
        
        #This publishes the status of the action server, mostly for debugging purposes
        self.pub_bug_talker.publish(f"Orientation is {orientation}")
        
        #3. Generate the action goal
        action_goal = self.generate_action_goal(end_point, orientation)
        
        return action_goal
        
    def generate_action_goal(self, end_point: list(), orientation: list()):
        #This function generates the action goal for the movebase client
        goal = MoveBaseGoal()
        
        goal.target_pose.header.frame_id = "utm"
        
        goal.target_pose.pose.position.x = end_point[0]
        goal.target_pose.pose.position.y = end_point[1]
        
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        
        return goal        
    
    def generate_path_multiple_points(self, utm_coordinates: list()):
        # This function generates a path from the starting position to the end position
        # The starting position is not the same as the robot position, but the first point in the list
        
        #Order of operations:
        #1. Define the first point in the list as the starting point with orientation
        #2. Generate the path from the starting point to the end point
        #3. Move the robot through the path.
        
        path = list()
        
        self.pub_bug_talker.publish(f"Generating path...")
        
        #1. Define the first point in the list as the starting point with orientation
        start_point = (utm_coordinates[0][0], utm_coordinates[0][1])
        next_point = (utm_coordinates[1][0], utm_coordinates[1][1])
        orientation = self.get_orientation(start_point, next_point)
        goal = self.generate_action_goal(start_point, orientation)
        path.append(goal)
        
        #2. Generate the path from the starting point to the end point
        for i in range(len(utm_coordinates) - 1):
            start_point = (utm_coordinates[i][0], utm_coordinates[i][1])
            end_point = (utm_coordinates[i + 1][0], utm_coordinates[i + 1][1])
            orientation = self.get_orientation(start_point, end_point)
            
            # This should only update the orientation
            path.append(self.generate_action_goal(start_point, orientation))
            
            
            path.extend(self.add_segment_to_path(orientation, start_point, end_point))
        
        self.pub_bug_talker.publish(f"Path generated")
        
        return path
    
    def add_segment_to_path(self, quaternions : list, start_point : list, end_point : list): #Want to make a separate class for this
        
        x_1, y_1 = start_point
        x_2, y_2 = end_point
        
        #Finding the length of this particular segment
        length_of_segment = math.dist(start_point, end_point)
        
        path = list()
        
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
                path.append(self.generate_action_goal(segment_point, quaternions))
                
        path.append(self.generate_action_goal(end_point, quaternions))
        return path

    
    def get_utm_coordinates(self, gps_coordinates: list()): #Want to make a separate class for this
        
        utm_coordinates = list()
        
        for i in range(len(gps_coordinates)):
            lat = gps_coordinates[i][0]
            lon = gps_coordinates[i][1]
            
            #This function converts the gps coordinates to utm coordinates in zone 32
            #The northern parameter is set to true, because we are in the northern hemisphere
            #UTM zone 32 is used in Norway south of Nordland
            utm_coordinates.append(utm.from_latlon(lat,lon, force_zone_number=32))
        
        return utm_coordinates

    def move_to_gps_points_using_cartesian(self):   #Possibly depricated
        cartesian_coordinates = rospy.ServiceProxy('gps_to_cartesian_service', GPSToCartesian)

        move_base_goal_list = list()

        move_base_goal_temp = MoveBaseGoal()
        move_base_goal_temp.target_pose.header.frame_id = "map"
        move_base_goal_temp.target_pose.pose.orientation.x = 0
        move_base_goal_temp.target_pose.pose.orientation.y = 0
        move_base_goal_temp.target_pose.pose.orientation.z = 0
        move_base_goal_temp.target_pose.pose.orientation.w = 1




        cartesian_coordinates_list = list()
        for i in range(len(self.gps_points_list)):
            rospy.loginfo(f"Latitude: {self.gps_points_list[i][0]}")
            rospy.loginfo(f"Longitude: {self.gps_points_list[i][1]}")
            respond = cartesian_coordinates(self.gps_points_list[i][1], self.gps_points_list[i][0])
            cartesian_coordinates.wait_for_service()
            rospy.loginfo(f"x = {respond.x_axis}")
            rospy.loginfo(f"y = {respond.y_axis}")

            # Copies the object, so we can use the same template
            move_base_goal_temp = copy.deepcopy(move_base_goal_temp)
            move_base_goal_temp.target_pose.pose.position.x = respond.x_axis
            move_base_goal_temp.target_pose.pose.position.x = respond.y_axis
            cartesian_coordinates_list.append((respond.x_axis, respond.y_axis))
            move_base_goal_list.append(move_base_goal_temp)



        rospy.loginfo("Starting to move")
        self.movebase_client(move_base_goal_list)


    def follow_path_planner(self):
        package_name = 'my_husky_package'
        file_name = '/include/sweeping_path.txt'
        path_from_file = self.get_data_from_file(package_name, file_name)

        move_base_goal_list = list()

        move_base_goal_temp = MoveBaseGoal()
        move_base_goal_temp.target_pose.header.frame_id = 'utm'
        move_base_goal_temp.target_pose.pose.orientation.x = 0
        move_base_goal_temp.target_pose.pose.orientation.y = 0
        move_base_goal_temp.target_pose.pose.orientation.z = 0
        move_base_goal_temp.target_pose.pose.orientation.w = 1

        for i in range(len(path_from_file)):
            # Copies the object, so we can use the same template
            move_base_goal_temp = copy.deepcopy(move_base_goal_temp)
            move_base_goal_temp.target_pose.pose.position.x = path_from_file[i][0]
            move_base_goal_temp.target_pose.pose.position.y = path_from_file[i][1]

            rospy.loginfo(f"Next Point: ")
            rospy.loginfo(f"X: {path_from_file[i][0]}")
            rospy.loginfo(f"Y: {path_from_file[i][1]}")

            move_base_goal_list.append(move_base_goal_temp)

        rospy.loginfo("Starting to move")
        self.movebase_client(move_base_goal_list)



    def get_data_from_file(self, package_name: str, file_name: str): #Want to make a separate class for this
        
        try:
            rp = rospkg.RosPack()
            package_path = rp.get_path(package_name)
        except rospkg.common.ResourceNotFound as e:
            rospy.logerr(f"Package not found: {e}")
            return None

        file_path = package_path + file_name

        
        gps_point_list = []

        try:
            with open(file_path, 'r') as file:
                for line in file:
                    values = line.strip().split(',')
                    gps_point_tuple = (float(values[0]), float(values[1]))
                    gps_point_list.append(gps_point_tuple)
        except FileNotFoundError as e:
            rospy.logerr(f"File not found: {e}")
            return None

        return gps_point_list


    def save_data_to_file(self): #Want to make a separate class for this
        #Declare the service client
        cartesian_coordinates = rospy.ServiceProxy('gps_to_cartesian_service', GPSToCartesian)
        self.pub_bug_talker.publish("Saving data to file")
        #Saves the cartesian coordinates to a file.
        dir_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(dir_path, 'cartesian_cordinates.txt'), 'w')as file:
            for i in range(len(self.gps_points_list)):
                lat = self.gps_points_list[i][0]
                lon = self.gps_points_list[i][1]

                temp = utm.from_latlon(lat, lon)
                #respond = cartesian_coordinates(self.gps_points_list[i][1], self.gps_points_list[i][0])
                file.write(f"{temp[0]} , {temp[1]}\n")
        self.pub_bug_talker.publish("Data saved to file")




    def movebase_client(self, path: list):
        #Sends a list of MoveBaseGoals that we got from the pathplanner.

        self.pub_bug_talker.publish("Starting movebase client")
        self.pub_bug_talker.publish("Connecting to move_base server")
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        self.pub_bug_talker.publish("Client connected to the move_base server")
        
        self.pub_bug_talker.publish(f"The movebase list has {len(path)} entries")
        #client.send_goal(path[0])
        # Sends the goal to the action server.
        #publisher_hack = rospy.Publisher("/move_base/goal", MoveBaseGoal, queue=10)
        #publisher_hack.publish(path[0])
        for goal in path:
            self.pub_bug_talker.publish("Publishing goal")
            self.pub_bug_talker.publish(str(goal.target_pose.pose.position.x))
            self.pub_bug_talker.publish(str(goal.target_pose.pose.position.y))
            client.send_goal(goal)
            rospy.loginfo(f"Moving to position:")
            rospy.loginfo(f"X: {goal.target_pose.pose.position.x}")
            rospy.loginfo(f"Y: {goal.target_pose.pose.position.y}")


            #rospy.loginfo(goal.target_pose.pose.position.x)
            wait = client.wait_for_result()
            client.wait_for_result()
            time.sleep(0.1)
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                # Result of executing the action
                rospy.loginfo(client.get_result())
            rospy.loginfo("Point Reached")

        self.pub_bug_talker.publish("Movement finished")


        # #client.send_goal(goal)
        # # Waits for the server to finish performing the action.
        # wait = client.wait_for_result()
        # # If the result doesn't arrive, assume the Server is not available
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     # Result of executing the action
        #     return client.get_result()

        # If the python node is executed as main process (sourced directly)
        
        
    def get_orientation(self, start_point: list, end_point: list):
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        
        angle_rad = math.atan2(dy, dx)
        orientation_rpy = [0, 0, angle_rad]
        orientation_quat = tf.transformations.quaternion_from_euler(*orientation_rpy)      
        
        return orientation_quat
    
    
class GetPosition:
    
    def __init__(self):
        self.sub = rospy.Subscriber("/emlid/fix", NavSatFix, self.gps_callback)
        self.gps_point = None
        self.recieved_data = False
        #rospy.init_node('get_position', anonymous=True)
        
    def gps_callback(self, data: NavSatFix):
        self.gps_point = (data.latitude, data.longitude)
        self.recieved_data = True
        self.sub.unregister()
        
    #Blocking function
    def get_gps_point(self):
        #rospy.spin() #The node runs until it recieves data
        while not self.recieved_data:
            #Wait for data
            time.sleep(0.1)
        return self.gps_point
    
class Rotator:
    
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.orientation = None

        self.sub = rospy.Subscriber("/global_ekf/odometry/filtered", Odometry, self.odom_callback)
    
    def rotate_in_place(self, angle: float):
        #Rotates the robot in place
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        
        while(self.orientation == None):
            #Wait for data
            time.sleep(0.1)
            
        while(self.orientation[2] < angle):
            #Rotate
            twist.angular.z = 0.5
            self.pub.publish(twist)

        
        

        
    def odom_callback(self, data: Odometry):
        #Gets the current orientation
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        
        self.orientation = tf.transformations.euler_from_quaternion([x, y, z, w])
        

if __name__ == '__main__':
    rospy.init_node('follow_path')

    server = ActionServer()
    
    rotator = Rotator()
    
    #rotator.rotate_in_place(3.14)
    
    rospy.spin()

    # try:
    #     # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    #     rospy.init_node('movebase_client_py')
    #     result = movebase_client()
    #     if result:
    #         rospy.loginfo("Goal execution done!")
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Navigation test finished.")