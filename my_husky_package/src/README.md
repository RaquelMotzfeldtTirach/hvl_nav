#### Lib
Library containing classes that are used in the following scripts. More information on the use of these classes in the directory itself.

#### gps_simulate_loss_of_signal.py
Node for simulating a loss of gps signal. Works by having a subscriber to the ROS Topic coming from the GPS receiver, here it is /emlid/fix, and a publisher to a ROS topic, here /emlid/fix/simulated. 

The node relays the signal for 30 seconds, and then for 10 seconds moves the signal up to 1 meter in a random direction around the GPS position. This behavior is continuous.

### relay_node_odometry_ekf.py

This node tries to maintain localization accuracy when the GPS either looses RTK correction or signal to the satellites. The localization is done by using the Robot Localization ROS package, a Navsat transform node for transforming the GPS position to a relative position to the robot frame and a extended kalman filter for fusing sensor data between the GPS signal and the onboard odometry.

The relay node works by placing itself between the sensor data and the kalman filter. If there is proper RTK correction on the GPS signal, the sensor data is passed through. If there is no RTK correction, the relay node does not send through any GPS data, but instead uses the localization approximated by the kalman filter and adds the difference in the last two onboard odometry readings. This will work in the short term, but as the onboard odometry has a tendency to drift and rotate, it is not a sustainable solution in the long term. 

#### Custom topics

For a more custom implementation, ROS parameters have been used to define the necessary topics. The names of the parameters is also the default topics if no parameters are defined.

- /odometry/gps
	- GPS odometry input to the relay node.
- /odometry/gps/filtered
	- GPS odometry output of the relay node.
- /gps/fix
	- Topic from the GPS driver.
- /odometry
	- Robots onboard odometry
	- Default topic here is "/husky_velocity_controller/odom"
- /odometry/flitered
	- Relay node output of odometry data
- /global_ekf/odometry/filtered
	- Output of Kalman filter for global localization

#### Kalman filter parameters

When using the relay node, it is important making sure the input of the kalman filter corresponds to the output of the relay node.

#### ROS publisher

A ROS publisher, "/debug_relay_node", publishes runtime messages.


### web_navigation_control_action_server.py
A ROS action server used for navigation. There are two types of navigation implemented, either following a set of GPS coordinates or generating, and following, a path designed to completely cover an area defined by GPS coordinates. The gps coordinates comes from the ROS package husky-gps-service  https://github.com/Axo-xD/husky-gps-service.

#### Action Server
This node initializes the action server "start_path", which takes a custom MovePathActionGoal message as an input. This message is defined in the ROS package my_husky_messages and contains a single string.

All navigation is done using a combination of GPS points, and converting them to UTM coordinates.

To use the action server, send an integer in the input section of the MovePathActionGoal message. The different actions implemented are:
- Input: "1" - Follow the GPS-coordinates sequentially.
- Input: "2" - Calculate a path for coverage, and execute it.

The design of this action server aims to making it easy to expand.

It is also possible to use a vizualiser tool. To do that, uncomment the lines of code in the corresponding function.

#### Waypoint following
Gets the GPS coordinates from the husky-gps-service package and moves the robot to these points one after another. 

#### Covering path following
Uses the PathCreator.py and the PathPlanner.py to generate a path that would cover an area defined by GPS coordinates. There are some limitations on this, the area must be defined as a convex polygon. 

The path that will start with the two first points defined.





