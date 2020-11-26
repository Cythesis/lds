## LDS - Learning and Developments
This is the base package for testing with other packages such as:
- [Smach State Machine](http://wiki.ros.org/smach)
- [URSim](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial)
- [Moveit](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html)
- [Usb_cam](http://wiki.ros.org/usb_cam)
- [Ar_track_alvar](http://wiki.ros.org/ar_track_alvar) 

## Requirements
Packages:
```
cd ~/catkin_ws/src
sudo apt install ros-melodic-moveit
sudo apt install ros-melodic-ar-track-alvar
git clone https://github.com/ros-drivers/usb_cam.git
rosdep install -y --from-paths . --ignore-src --rosdistro melodic
cd ..
catkin_make
```

Non-packages (Optional)(Already included in package):

[URSimulator](https://www.universal-robots.com/download/?option=91610#section41511)


## Installation
Package:
```
git clone https://github.com/Cythesis/lds.git
```
URSimulator set up:
[Link](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

## Usage
To run the basic demo:
```
rosrun lds start-ursim.sh
roslaunch lds all.launch
rosrun lds automas.py
```
Press enter to proceed with the demo.

## Extra Info
Service:
```
/move_pose
/move_cartesian
```
Service callback functions available in [controller.cpp](src/controller.cpp)

## Issues
![Issues](/images/Issues.png)



