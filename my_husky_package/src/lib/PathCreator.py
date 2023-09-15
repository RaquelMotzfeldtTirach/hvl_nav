import utm
import math
import tf.transformations
import rospy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from .GetPosition import GetPosition
from .PathPlanner import PathPlanner
from std_msgs.msg import String


# Class for creating a path based on a list of gps coordinates
class PathCreator:

    def __init__(self, target_frame: str, segment_length: float=1.0, publisher: rospy.Publisher=rospy.Publisher('/path_creator', String, queue_size=1)):
        self.target_frame = target_frame
        self.segment_length = segment_length
        
    # The main function, called from outside, to generate the path
    def generate_path(self, gps_coordinates: list()):

        #Checks the number of points in the list, and calls the appropriate function
        if(len(gps_coordinates) == 1):
            return self.generate_path_single_point(gps_coordinates[0])
        elif (len(gps_coordinates) > 1):
            return self.generate_path_multiple_points(gps_coordinates)
        else:
            raise ValueError("The list of gps coordinates is empty")
        
        
    # Generates a path based on a single GPS coordinate
    def generate_path_single_point(self, end_point: list()):
        # This function generates a path from the current posisiton to the end point
        
        action_goal = list()
        
        # Order of operations:
        # 1. Get current position
        
        get_position = GetPosition()
        
        lat, lon = get_position.get_gps_point()
        
        current_position = utm.from_latlon(lat, lon)
        
        # 2. Get the UTM coordinates from the end point
        end_point = utm.from_latlon(end_point[0], end_point[1])
        
        # 3. Get the orientation between the robot and the end point
        orientation = self.get_orientation(current_position, end_point)
        
        # 4. Generate the action goal
        action_goal.append(self.generate_action_goal(end_point, orientation))
        
        
        
        return action_goal
    
    # Generates a path based on a list of GPS coordinates
    def generate_path_multiple_points(self, gps_coordinates: list()):
        
        path = list()
        
        # 1. Convert the GPS coordinates to UTM coordinates
        utm_coordinates = self.get_utm_coordinates(gps_coordinates)
        
        # 2. Define the first point in the list as the starting point of the path with corresponding orientation
        # start_point = (utm_coordinates[0][0], utm_coordinates[0][1])
        # next_point = (utm_coordinates[1][0], utm_coordinates[1][1])
        # orientation = self.get_orientation(start_point, next_point)
        # goal = self.generate_action_goal(start_point, orientation)
        # path.append(goal)
        
        # 3. Generate the path from the starting point to the end point
        
        for i in range(len(utm_coordinates) - 1):
            # Define the working points
            start_point = (utm_coordinates[i][0], utm_coordinates[i][1])
            end_point = (utm_coordinates[i + 1][0], utm_coordinates[i + 1][1])
            orientation = self.get_orientation(start_point, end_point)
            
            # Add a rotation before movement
            path.append(self.generate_action_goal(start_point, orientation))
            
            # Adds the movement, in segments, to the path
            path.extend(self.add_segment_to_path(orientation, start_point, end_point, self.segment_length))
            
        return path
    
    def generate_path_sweeping_coverage(self, gps_coordinates: list()):
        
        path = list()
        
        # 1. Convert the GPS coordinates to UTM coordinates
        utm_coordinates = self.get_utm_coordinates(gps_coordinates)
        utm_coordinates_ = list()
        
        # 1.1. Clean up the UTM coordinates.
        for coordinates in utm_coordinates:
            utm_coordinates_.append((coordinates[0], coordinates[1]))
            coordinates = (coordinates[0], coordinates[1])
            
        utm_coordinates = utm_coordinates_
        
        # 2. Generate the path
        # 2.1 Determine the starting line of the sweeping coverage path
        # For future implementation - Shamos' Algorithm for determining antipodal pairs-
        # Using the antipodal pairs with the closest distance for most efficient path
        
        # Current implementation: Using the two first points as the starting line
        # Defined by two endpoints
        vertex_1 = utm_coordinates[0]
        vertex_2 = utm_coordinates[1]
        
        # 2.2 Define the path planner
        path_width = 2.0 # The width of the path in meters
        path_planner = PathPlanner(path_width=path_width, vertex_1=vertex_1, vertex_2=vertex_2, bounding_vertices=utm_coordinates)
        
        # 2.3 Generate the coverage path
        planned_path = path_planner.get_path()
        
        # 3 Generate the complete path with the corresponding orientation
        
        for i in range(len(planned_path) - 1):
            # Define the working points
            start_point = (planned_path[i][0], planned_path[i][1])
            end_point = (planned_path[i + 1][0], planned_path[i + 1][1])
            orientation = self.get_orientation(start_point, end_point)
            
            # Add a rotation before movement
            path.append(self.generate_action_goal(start_point, orientation))
            
            # Adds the movement, in segments, to the path
            path.extend(self.add_segment_to_path(orientation, start_point, end_point, self.segment_length))
            
        return path
        
        
        
        
        
    # This function converts a list of gps coordinates to utm coordinates
    def get_utm_coordinates(self, gps_coordinates: list()) -> list():
        
        utm_coordinates = list()
        
        for i in range(len(gps_coordinates)):
            # Gets the latitude and longitude from the list
            lat = float(gps_coordinates[i][0])
            lon = float(gps_coordinates[i][1])
            
            # Converts the latitude and longitude to utm coordinates in zone 32
            utm_coordinates.append(utm.from_latlon(lat,lon, force_zone_number=32))
    
        return utm_coordinates
    
    # This function generates an action goal based on the end point and the orientation    
    def generate_action_goal(self, end_point: list(), orientation: list()):
        # Create the action goal of type MoveBaseGoal
        goal = MoveBaseGoal()
        
        # Define the frame of the goal, as defined in the init function
        goal.target_pose.header.frame_id = self.target_frame
        
        # Add the end point and orientation to the goal
        goal.target_pose.pose.position.x = end_point[0]
        goal.target_pose.pose.position.y = end_point[1]
        
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]
        
        return goal   
        
    
    # This function gets the orientation between two points
    def get_orientation(self, start_point: list, end_point: list):
        dx = float(end_point[0]) - float(start_point[0])
        dy = float(end_point[1]) - float(start_point[1])
        
        angle_rad = math.atan2(dy, dx)
        orientation_rpy = [0, 0, angle_rad]
        orientation_quat = tf.transformations.quaternion_from_euler(*orientation_rpy)      
        
        return orientation_quat
    
    # This function adds a segment to the path for straight movement.
    def add_segment_to_path(self, quaternions : list, start_point : list, end_point : list, segmented_lengths : float=1.0) -> list(): 
        
        x_1, y_1 = start_point
        x_2, y_2 = end_point
        
        #Finding the length of this particular segment
        length_of_segment = math.dist(start_point, end_point)
        
        path = list()
        
        # Equals zero when only rotation is wanted.
        if(length_of_segment > segmented_lengths):
            
            #The number of segments should equal the lenght of the segment in meters.
            number_of_segments = int(length_of_segment)/segmented_lengths
            
            #Finding the travel distane along each axis per segmnt
            dx = (x_2 - x_1) / number_of_segments
            dy = (y_2 - y_1) / number_of_segments
            
            
            #Using intepolation, create the segments
            for i in range(int(number_of_segments)):
                x = x_1 + i * dx
                y = y_1 + i * dy
                segment_point = [x, y]
                path.append(self.generate_action_goal(segment_point, quaternions))
        
        # Adds the end point to the path        
        path.append(self.generate_action_goal(end_point, quaternions))
        
        return path
    
    