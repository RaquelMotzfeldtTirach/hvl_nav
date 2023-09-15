from lib import FileHandler, PathCreator, MoveBaseClient, PathVizualiser
import tf
import math

if __name__ == "__main__":
    
    #Testing the orientation given from the path.
    
    # Creating a path with different orientations
    path = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
    
    path_creator = PathCreator('utm')
    
    #path_calculated = path_creator.generate_path(path)
    
    for i in range(len(path) - 1):
        orientation_in_quat = path_creator.get_orientation(path[i], path[i + 1])
        rpy = tf.transformations.euler_from_quaternion(orientation_in_quat)
        
        print(math.degrees(rpy[2]))
    
    
    
    
    # calculate the angle between point t=0 and t=1 from the path
    
    
          
    #path_vizualiser = PathVizualiser()
    #path_vizualiser.vizualise(path)
    
    