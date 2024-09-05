# Arctos GUI v0.1.1 
This is a GUI application for controlling a robotic arm via ROS. It supports connections through either an Arduino Mega or a CANable adapter. The interface allows for controlling the arm's joints, Cartesian coordinates, and gripper movements. Additionally, it includes functionality to send custom CAN messages to the robotic arm.

![arctosgui.png](/arctosgui.png)

```
su root 
nano /etc/sudoers

<username> user_name ALL=(ALL)  ALL
```
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update

sudo apt install ros-melodic-desktop 
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo su 
apt install python-rosdep2 
apt-get update
apt-get dist-upgrade
```
in new terminal: 
```
sudo apt-get install ros-melodic-catkin python-catkin-tools 
sudo apt install ros-melodic-moveit
source /opt/ros/melodic/setup.bash 
sudo apt-get install ros-melodic-moveit ros-melodic-moveit-visual-tools 

sudo apt-get install git
git clone https://github.com/ArctosRobotics/ROS

catkin build 
cd ROS 
source devel/setup.bash 
```
```
sudo apt update
sudo apt install python3-pip
pip3 install python-can[serial]
sudo apt-get install python3-tk
pip3 install sv-ttk
 ```
```
git clone https://github.com/ArctosRobotics/arctosgui


pip3 install ttkthemes
nano ~/.bashrc
```
add this line with your username: 
```
source /home/<your username>/ROS/devel/setup.bash 
```
Ctrl+S
Ctrl+X
```
sudo apt install python3-rosdep python3-rosinstall-generator python3-wstool build-essential 
sudo apt install python3-rosinstall python3-catkin-tools python3-osrf-pycommon
sudo apt-get install ros-melodic-robot-state-publisher 
sudo apt-get install ros-melodic-joint-state-publisher 
sudo chmod a+rw /dev/tty<YOUR PORT> 
```

```
sudo rosdep init
rosdep update
```
```
cd arctosgui 
ls 
Ctrl+C
chmod +x run.sh 
ls # should be green 
./run.sh 
```
4 tabs will open 
you can manually open them by: 
```
roslaunch arctos_config demo launch 
rosrun moveo_moveit interface.py 
rosrun moveo_moveit transform.py 
python3 ui.py 
```
Wait for the gui and rviz to show 

In moveit rviz go File>Open config or Ctrl+O and open 
arctosgui_config.rviz

Connect the robot 

Plan new toolpath by moving joints or tool
Run ROS button will send CAN messages from new pose 

Run RoboDK will send gcode.tap file to robot 
Make sure that you copy the gcode from RoboDK post processor to gcode.tap or adapt it to export code to arctos gui location under the name gcode.tap and replace it. 

Set gear ratios in convert.py and roscan.py 
gear_ratios = [1, 1, 1, 1, 1, 1]  # Replace with your actual gearbox ratios

Raw gear ratios. 
X  13.5
Y  150
Z  150
A  48
B  67.82
C  67.82

In theory raw gear ratios should be multiplied to 0.5, so gear_ratios would be
[6.75, 75, 75, 24, 33.91, 33.91]
They are not tested! 

# Change this according to your folder 
```
user_path = "/home/ee/arctosgui"
```


# Now supports open loop version with Arduino MEGA. 
just adapt serial port to your specific port 

```
serial_port = "/dev/ttyUSB0"
```

Before using you need to give permissions to ttyUSB0
```
sudo chmod a+rw /dev/ttyUSB0
```
