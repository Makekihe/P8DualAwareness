# P8DualAwareness


YOLO setup: https://github.com/tom13133/darknet_ros/tree/yolov4
Note, when git cloning, DO NOT use their link in the ReadMe, it is outdated. Instead, use the link you get when clicking 'Clone' -> 'SSH'


#Park test
## Plugin
The plugin should be installed when downloading the repository. To tell Gazebo where to find the plugin, insert this line to the terminal (with your own workspace path instead of `{ws_path}`) 

```c
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:{ws_path}/actor_collisions/build' >> ~/.bashrc
``` 

Restart the terminal or run this command `. ~/.bashrc` 

<br>

##Launch
Add to bashrc, run:
`roslaunch park_test park.launch
