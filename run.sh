#!/bin/bash

# Open new tab and launch the ROS demo
gnome-terminal --tab -- bash -c "roslaunch arctos_config demo.launch; exec bash"

# Open new tab and run the Moveo MoveIt interface
gnome-terminal --tab -- bash -c "rosrun moveo_moveit interface.py; exec bash"

# Open new tab and run the Moveo MoveIt interface
gnome-terminal --tab -- bash -c "rosrun moveo_moveit transform.py; exec bash"

# Open new tab, change directory to ~/Desktop/devv, and run the UI script
gnome-terminal --tab -- bash -c "python3 ui.py; exec bash"

