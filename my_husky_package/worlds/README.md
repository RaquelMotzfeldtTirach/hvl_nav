Directory for saving the worlds to be used in simulation. 

### map_test.world
Has a world based on a real-life berry farm with an area of trees used to represent the berrybushes.

There has been developed a way of using a satellite image as the ground plane. 
- Navigate to my_husky_package/worlds
- In the file "static_map_plugin_test.world", insert a Google API key between the tags "<api_key>"
- In the terminal type: ``` gazebo --verbose static_map_plugin_test.world ```
- Launch run_simulation.launch
Now the ground plane should match the coordinates.