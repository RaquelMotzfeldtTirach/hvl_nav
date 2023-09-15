
import rospkg
import rospy
import os

# Class for handling reading and writing to files, specifically csv files with gps data in a this ros project
class FileHandler:
    
    def __init__(self):
        pass
    
    # Reads a csv file and returns a list of gps coordinates.
    # Arguments:
    #   package_name: The name of the package where the file is located
    #   file_name: The name of the file to read, including the path from the package root
    # Returns:
    # List of lists of gps coordinates
    def read_csv_file(self, package_name: str, file_name: str) -> list:
        try:
            rp = rospkg.RosPack()
            package_path = rp.get_path(package_name)
        except rospkg.common.ResourceNotFound as e:
            rospy.logerr(f"Package not found: {e}")
            return None
    
        file_path = package_path + file_name
        
        data_list = []
        
        try:
            with open(file_path, 'r') as file:
                data_list = self.extract_data(file.readlines())
        except FileNotFoundError as e:
            rospy.logerr(f"File not found: {e}")
            return None
        
        return data_list

    # Extracts data from a list of strings and returns a list of lists of the data
    def extract_data(self, data: list) -> list:
        data_list = []
        
        for line in data:
            values = line.strip().split(',')
            values = [float(i) for i in values]
            data_list.append(values)
        
        return data_list
    
    # Saves a list of gps coordinates to a csv file in the current directory
    def save_to_file_current_dir(self, data: str, filename: str):
        
        # Get the path of the current file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        
        # Create the file
        with open(os.path.join(dir_path, filename), 'w') as file:
            
            for line in data:
                
                for i in range(len(line)): # Write all but the last element with a comma after
                    
                    if i < (len(line) -1 ):
                        file.write(str(line[i]) + ',')
                    else:
                        file.write(str(line[i]))                
                file.write('\n')
    
    # Saves a list of gps coordinates to a csv file in a custom directory    
    def save_to_file_custom_dir(self, data: str, package_name: str, filename: str):
            
            # Get the path of the current file
            rp = rospkg.RosPack()
            package_path = rp.get_path(package_name)
            
            # Create the file
            with open(os.path.join(package_path, filename), 'w') as file:
                for line in data:
                
                    for i in range(len(line)): # Write all but the last element with a comma after
                        
                        if i < len(line):
                            file.write(str(line[i]) + ',')
                        else:
                            file.write(str(line[i]))                
                    file.write('\n')