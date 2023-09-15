from tf.transformations import quaternion_from_euler
import tf.transformations
from math import pi
import utm
import math
import matplotlib.pyplot as plt

def test1():
    orientation = (0, 0, pi/2)
    quaternions = quaternion_from_euler(*orientation)
    
    rpy_degs = tf.transformations.euler_from_quaternion(quaternions)
    roll, pitch, yaw = rpy_degs
    
    print(rpy_degs)
    print(quaternions)

    lat = 61.18064481779069
    lon = 6.862252499306305

    temp_ = utm.from_latlon(lat, lon)


    temp = [temp_[0], temp_[1]]

    numbers = [1, 2]

    x_1, y_1 = numbers

    # print(x_1)
    # print(y_1)

    print(int(7.2))


    for i in range(3):
        print(i)
    
    # for i in range(3):
    #     print(temp[0])
    #     temp[0] = temp[0] + 1
    
def test_orientation_plotter():
    # Example usage
    path_points = [(0, 0), (1, 1), (3, 1), (3, 4), (6, 4)]
    path_orientations = calculate_path_orientations(path_points)
    visualize_path(path_points, path_orientations)


def calculate_orientation(start_point, end_point):
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    
    return angle_deg

def calculate_path_orientations(path_points):
    path_orientations = []
    
    for i in range(len(path_points) - 1):
        start_point = path_points[i]
        end_point = path_points[i + 1]
        orientation = calculate_orientation(start_point, end_point)
        path_orientations.append(orientation)
    
    return path_orientations

def visualize_path(path_points, path_orientations):
    x_values = [point[0] for point in path_points]
    y_values = [point[1] for point in path_points]
    
    fig, ax = plt.subplots()
    ax.plot(x_values, y_values, '-o')
    
    for i in range(len(path_points) - 1):
        x = path_points[i][0]
        y = path_points[i][1]
        orientation = path_orientations[i]
        dx = math.cos(math.radians(orientation))
        dy = math.sin(math.radians(orientation))
        ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1, color='r')
    
    ax.set_aspect('equal', 'box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    plt.show()



if __name__ == '__main__':
    test_program = 0
    
    if(test_program == 0):
        test_orientation_plotter()
    elif(test_program == 1):
        test1()
