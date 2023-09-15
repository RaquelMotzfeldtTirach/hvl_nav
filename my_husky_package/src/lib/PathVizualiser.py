import tf
import math
import matplotlib.pyplot as plt


# This class is used to visualize the path that the robot is following
class PathVizualiser:
    
    def __init__(self):
        # Declare lists where the plotted values will be stored
        self.x_values = list()
        self.y_values = list()
        
        self.orientations = list()
        self.orientations_from_quaternions = list()
    
    # This function takes a path, and vizualises it. Using MoveBaseGoal from ROS
    def vizualise(self, path: list()):
        
        self.fill_lists(path)
        
        fig, ax = plt.subplots()
        ax.plot(self.x_values, self.y_values, '-o')
        
        for i in range(len(path) - 1):
            x = self.x_values[i]
            y = self.y_values[i]
            orientation = self.orientations[i]
            dx = math.cos(math.radians(orientation))
            dy = math.sin(math.radians(orientation))
            ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1, color='r')
            
        # Adsjust aspect ratio
        ax.set_aspect('equal', 'box')
        
        # Set limits of the plot based on UTM coordinates
        x_min, x_max = min(self.x_values), max(self.x_values)
        y_min, y_max = min(self.y_values), max(self.y_values)
        ax.set_xlim([x_min - 10, x_max + 10])
        ax.set_ylim([y_min - 10, y_max + 10])
        
        ax.set_xlabel('UTM X')
        ax.set_ylabel('UTM Y')
        
        # Show the plot
        plt.show()
        
    def fill_lists(self, path:list()):
        
        for point in path:
            self.x_values.append(point.target_pose.pose.position.x)
            self.y_values.append(point.target_pose.pose.position.y)
            
            x = point.target_pose.pose.orientation.x
            y = point.target_pose.pose.orientation.y
            z = point.target_pose.pose.orientation.z
            w = point.target_pose.pose.orientation.w
            
            rpy = tf.transformations.euler_from_quaternion([x,y,z,w])
            yaw = rpy[2]
            
            #self.orientations_from_quaternions.append(math.degrees(yaw))
            self.orientations.append(math.degrees(yaw))
            
        for i in range(len(self.x_values) - 1):
            start_point = [self.x_values[i], self.y_values[i]]
            end_point = [self.x_values[i+1], self.y_values[i+1]]
            
            #self.orientations.append(self.calculate_orientation_in_degs(start_point, end_point))
            
    def calculate_orientation_in_degs(self, start_point, end_point):
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        
        return angle_deg         

    