More information on the working of each node can be found in the corresponding src folder.


### custom_gps_navigation.launch
Launches all the nodes for navigating using the webapplication.

#### Websocket
Launches the websocket for communicating between the webapplication and ROS.

#### web_navigation_control_action_server.py
The action server responsible for controlling the robot.

#### relay_node_odometry_ekf.py
A relay node designed to make the localization more accurate if the GPS reciever looses signel.

#### gps_user_service_server.py
A ROS service designed to receive and save GPS positions from the webapplication so that the action server can utilize them.

### run_simulation.launch
Starts the Gazebo simulation and spawns the husky robot.

There has been developed a way of using a satellite image as the ground plane. 
- Navigate to my_husky_package/worlds
- In the file "static_map_plugin_test.world", insert a Google API key between the tags "<api_key>"
- In the terminal type: ``` gazebo --verbose static_map_plugin_test.world ```
- Launch run_simulation.launch
Now the ground plane should match the coordinates.

### tunnel_world.launch
Starts the Gazebo simulation with the world "map_test.world "


### empty_world.launch
Starts gazebo with an empty world.
### spawn_husky.launch
Spawns the husky in a running Gazebo world.


### description.launch
Used for spawning in the husky in spawn_husky.launch

### navigation_stack_for_simulation.launch
Starts the necessary nodes for achieving navigation through a Gazebo simulation.

This launch does not simulate loss of GPS signal

### simulating_navigation_no_relay_node.launch
Launches the necessary nodes for obtaining navigation through a Gazebo simulation without the relay node for the kalman filter.

This launch is simulating loss of GPS signal

### simulating_navigation_with_relay_node.launch
Launches the necessary nodes for obtaining navigation through a Gazebo simulation with the relay node for the kalman filter.

This launch is simulating loss of GPS signal
### navigation_clone_from_mbs.launch
A copy of the gps_navigation launch file from MBS without the LIDAR node.