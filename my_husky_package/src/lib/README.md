### FileHandler.py
Class for reading and writing to files, specifically csv files with gps data.

#### .read_csv_files(package_name: str, file_name: str) -> list
- package_name: the name of the ROS package.
- file_name: the name of the file to be read, including the path from the package root

Reads a csv file and returns a list containing all the data.

#### .save_to_file_current_dir(data: str, filename: str)
- data: The data to be saved, sent as a string.
- filename: name of the file to be saved as.

Saves the data string as comma separated value in the ROS package this method is called from. The filename argument is the filename of the file saved

#### .save_to_file_custom_dir(data: str, package_name: str, filename: str)
- data: The data to be stored, sent as a string.
- package_name: the ROS package the file will be saved.
- filename: name of the file to be saved as.

Saves the data string as a comma separated value in the given ROS package.


### GetPosition.py
Class for getting the current position of the robot. 

Works by creating a ROS subscriber to the GPS publisher and returns the last given position before unregistering. Class is getting only one position.

#### .get_gps_:point()
Returns the first GPS point after the class is initialized. 

### MoveBaseClient.py
Class for sending goals to the move base action client from the navigation stack from ROS. Capable of either sending a single goal, or to send several goals sequentially.

#### .follow_path(path: list)
- path: a list of ROS MoveBaseGoal.

Sends sequentially the MoveBaseGoals to the move_base action server.

#### .move_to_point(point: MoveBaseGoal)
- point: MoveBaseGoal

Sends the point to the move_base action server.


### PathCreator.py
Class for creating a path for a mobile robot. Can generate a path either containing a single point, multiple points or calculate a path for covering a given area. The path generated is defined as a list of ROS MoveBaseGoal with corresponding position and orientation. 
To obtain straight movement the path is created with multiple points per straight line, with a set length between the points.

#### init(target_frame: str, segment_length: float)
- target_frame: World frame to navigate in.
- segment_langth: Distance between the path goals.

Constructor of the class, used only for defining the nature of the path to be created.

#### .generate_path(gps_coordinates: list) -> path
- gps_coordinates: GPS points to navigate to/through

Generates the path that moves through all the given gps_coordinates.

#### .generate_path_sweeping_coverage(gps_coordinates: list) -> path
- gps_coordinates: The bounding coordinates of the area to be covered.

Generates a path that covers an entire area using PathPlanner.py. This path starts with the first two coordinates given.



### PathPlanner.py
Class for planning a path that covers an entire area. Takes a starting line and creates parallell lines that covers the area with a given width.

#### init(path_width: float, vertex_1: tuple, vertex_2: tuple, bounding vertices: list)
- path_width: the width of the path to be taken.
- vertex_1: One point of the starting line.
- vertex_2: Other point of the starting line.
- bounding_vertices: List of points defining the area to be covered.

Constructor only initializes variables used for creating a path.
#### .get_path() -> path
Creates a path to cover the area defined by bounding_vertices with a given width. Generates a path originating from the line defined by vertex_1 and vertex_2.
Shrinks the polygon defined by bounding_vertices by the width of the path to be taken. 

### PathVizualiser.py
A tool for vizualising a path that a robot will take. It takes in a list of ROS MoveBaseGoals and plots it with arrows indicating the orientation each goal has. Great for making sure the path calculated is the desired path.

#### .vizualise(path)
- path: the path to be vizualised.

Vizualising the given path.