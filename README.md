# Arctos GUI v0.1.1

Arctos GUI is a graphical user interface for controlling a robotic arm using ROS (Robot Operating System). It supports connections through Arduino Mega or CANable adapter, allowing users to control the arm's joints, Cartesian coordinates, and gripper movements, as well as send custom CAN messages.

![Arctos GUI Screenshot](/arctosgui.png)

## Installation Options

You have two options to install Arctos GUI:
1. **Step-by-step installation**: Install ROS, dependencies, and Arctos GUI manually.
2. **Preconfigured Virtual Machine**: Use a preconfigured Virtual Machine image with everything set up for you.

## Table of Contents

- [Step-by-Step Installation](#step-by-step-installation)
- [Preconfigured Virtual Machine](#preconfigured-virtual-machine)
- [Gear Ratios](#gear-ratios)
- [Arduino MEGA Open Loop Support](#arduino-mega-open-loop-support)
- [Useful Links](#useful-links)

# Step-by-Step Installation


To start, you need to install ROS and its dependencies:

```
su root
nano /etc/sudoers
<username> ALL=(ALL) ALL
 ```
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
Next, install additional ROS dependencies:
 ```
```
sudo apt-get install ros-melodic-catkin python-catkin-tools
sudo apt install ros-melodic-moveit ros-melodic-moveit-visual-tools 
sudo apt-get install git
git clone https://github.com/ArctosRobotics/ROS
bash
  ```
```
catkin build 
cd ROS
source devel/setup.bash
 ```

Clone and Build the Repository
Clone the Arctos GUI repository and install dependencies:

 
 
```
git clone https://github.com/ArctosRobotics/arctosgui
sudo apt update
sudo apt install python3-pip
pip3 install python-can[serial] ttkthemes sv-ttk

```

Add ROS environment to your bash profile:



```
nano ~/.bashrc
```

Add this line:

```
source /home/<your-username>/ROS/devel/setup.bash
```

Save with Ctrl+S and exit with Ctrl+X.

Running the GUI
After installation, navigate to the arctosgui directory and start the GUI:


 ```
cd arctosgui
chmod +x run.sh
./run.sh
```

Alternatively, you can manually open each tab:


 ```
roslaunch arctos_config demo.launch 
rosrun moveo_moveit interface.py 
rosrun moveo_moveit transform.py 
python3 ui.py
Connecting the Robot
```
In MoveIt RViz, go to File > Open Config or press Ctrl+O and open the arctosgui_config.rviz file. Once the configuration is loaded, connect the robot via Arduino or CANable, and use the GUI to plan and execute movements.

# Preconfigured Virtual Machine
If you prefer a faster setup, you can use a preconfigured Virtual Machine:


- Download and install [VirtualBox](https://www.virtualbox.org/wiki/Downloads).
- Download the Arctos preconfigured VM image from this [Google Drive link](https://drive.google.com/drive/folders/1R-wapvf-ZU6bWU-n6ExmzQ6M_z44bq0r?usp=sharing
).
- Open VirtualBox and create a new machine (Ctrl+N).
- Name the machine and set the type to Linux (Debian 64-bit).
- Use iso image zz-disk001.iso
- Set at least 4 GB of RAM.
- Use the existing virtual hard disk file (zz-disk002.vmdk).
- Start the machine and log in with the password: zz.
- In the left toolbar, launch Arctos GUI, connect the robot via Arduino or CANable, and start using the interface.

# Gear Ratios
Set the gear ratios in convert.py and roscan.py as needed for your robot:

 
gear_ratios = [1, 1, 1, 1, 1, 1]  # Replace with your actual gearbox ratios
For raw gear ratios:

X: 13.5
Y: 150
Z: 150
A: 48
B: 67.82
C: 67.82
These raw gear ratios can be multiplied by 0.5 for estimated values:

  ```
gear_ratios = [6.75, 75, 75, 24, 33.91, 33.91]
 ```

# Arduino MEGA Open Loop Support
To use the open loop version with Arduino MEGA, adapt the serial port configuration in convert.py:

 ```
serial_port = "/dev/ttyUSB0"
```

Before running, you need to give permissions to the USB port:

 ```
sudo chmod a+rw /dev/ttyUSB0
```

# Useful Links

- [Documentation](https://arctosrobotics.com/docs/)
- [Manuals](https://arctosrobotics.com/#Assembly)
- [CAD Files](https://arctosrobotics.com/#Assembly)
