## LDS - Learning and Developments
This is the base package for testing with other packages such as:
- [Smach State Machine](http://wiki.ros.org/smach)
- [URSim](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial)
- [Moveit](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html)
- [Usb_cam](http://wiki.ros.org/usb_cam)

## Requirements
Packages:
`sudo apt install ros-melodic-moveit
sudo apt install ros-melodic-smach`

Non-packages (Optional)(Already included in package):
[URSimulator](https://www.universal-robots.com/download/?option=91610#section41511)


## Installation
`git clone https://github.com/Cythesis/lds.git`

## Usage
To run the basic demo:
`rosrun lds start-ursim.sh
roslaunch lds all.launch
rosrun lds automas.py`
Press enter to proceed with the demo.

## Topics and services

## Issues


To create functional system and movement for the UR5 robot
