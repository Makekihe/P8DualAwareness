# P8DualAwareness


YOLO setup: https://github.com/tom13133/darknet_ros/tree/yolov4
Note, when git cloning, DO NOT use their link in the ReadMe, it is outdated. Instead, use the link you get when clicking 'Clone' -> 'SSH'


# Park test
## Plugin
The plugin should be installed when downloading the repository. To tell Gazebo where to find the plugin, insert this line to the terminal (with your own workspace path instead of `{ws_path}`) 

```c
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:{ws_path}/actor_collisions/build' >> ~/.bashrc
``` 

Restart the terminal or run this command `. ~/.bashrc` 

<br>

## Launch
Add to bashrc, run:
`roslaunch park_test park.launch

# picture_to_coordinates

This is the package that deprojects pixels into the world coordinates of the CAMERA frame. To run it with the realsense and YOLO, you will need extra packages. 

My publisher is a simple publisher that publishes simulated YOLOv4 detection in the darknet_ros_msg/BoundingBoxes format.  

# turtlebot3_navigation
Copied the turtlebot3 nav package and modified it to work with our nav_stack from ROB6. There is a launch file that launches TEB nav stack. A lot of work here to still be done BUT we will continue with this on the robot. Consider this package to be a backup of work done so far. 
