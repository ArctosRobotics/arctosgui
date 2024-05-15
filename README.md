
### Arctos CAN controller GUI 
![gui-light.jpg](/img/gui-light.jpg)

## 


Copy scripts interface.py and transform.py to:

/your-path/arctos_ros/src/arctos_moveit/scripts

make sure they are executable with: 
```
sudo chmod +x interface.py 
sudo chmod +x transform.py 
```
```
cd ~/your-path/arctos_ros 
catkin build 
```
Install requirements:
 ```
pip install -r requirements.txt
```
Now you can go to 
```
cd ~/your-path/arctosgui 
```
and run 
```
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

If you have other port than /dev/ttyACM0 edit files 
send.py and ros.py to adress your specific port 

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


 
