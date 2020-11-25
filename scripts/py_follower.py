#!/usr/bin/env python

import roslib
import rospy
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from math import radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from ar_track_alvar_msgs.msg import AlvarMarkers
import time 

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class Controller(object):
	def __init__(self):
		super(Controller, self).__init__()
		moveit_commander.roscpp_initialize(sys.argv)
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group_name = "manipulator"
		move_group = moveit_commander.MoveGroupCommander(group_name)
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
														moveit_msgs.msg.DisplayTrajectory,
														queue_size=20)
		planning_frame = move_group.get_planning_frame()
		eef_link = move_group.get_end_effector_link()
		group_names = robot.get_group_names()
		move_group.set_goal_joint_tolerance(0.1)
		move_group.set_goal_orientation_tolerance(0.1)
		r = rospy.Rate(100)

		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.move_group = move_group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.eef_link = eef_link
		self.group_names = group_names

	def movec(self,data):
		move_group = self.move_group
		q = move_group.get_current_joint_values()
		z = move_group.get_goal_orientation_tolerance()
		print(z)
		q[0] = data[0]
		q[1] = data[1]
		q[2] = data[2]
		q[3] = data[3]
		q[4] = data[4]
		q[5] = data[5]
		move_group.go(q, wait = True)
		move_group.stop()

	def movej(self,data):
		move_group = self.move_group
		pose_goal = self.move_group.get_current_pose().pose
		pose_goal.position.x = self.coordinates[0]
		pose_goal.position.y = self.coordinates[1]
		pose_goal.position.z = self.coordinates[2]
		# pose_goal.orientation.x = self.coordinates[3]
		# pose_goal.orientation.y = self.coordinates[4]
		# pose_goal.orientation.z = self.coordinates[5]
		move_group.set_pose_target(pose_goal)
		move_group.go(wait=True)
		move_group.stop()
		move_group.clear_pose_targets()

	def arCallback(self,data):
		# try:
		x = data.markers[0].pose.pose.position.x
		y = data.markers[0].pose.pose.position.y
		z = data.markers[0].pose.pose.position.z
		rx = data.markers[0].pose.pose.orientation.x
		ry = data.markers[0].pose.pose.orientation.y
		rz = data.markers[0].pose.pose.orientation.z
		self.coordinates = [x,y,z,rx,ry,rz]
		self.arFound = 1
		print("AR Detected")
		self.movej(1)
		# except:
		# 	self.arFound = 0
		# 	print("No AR Detected")


if __name__ == '__main__':
	rospy.init_node('py_follower')
	robot = Controller()
	rospy.Subscriber('ar_pose_marker', AlvarMarkers, robot.arCallback, queue_size=1)
	q = [0,-3.14/2,0,0,0,0]
	robot.movec(q)
	rospy.spin()










# class MoveGroupPythonInterfaceTutorial(object):
#   def __init__(self):
#     super(MoveGroupPythonInterfaceTutorial, self).__init__()
#     moveit_commander.roscpp_initialize(sys.argv)
#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     group_name = "manipulator"
#     move_group = moveit_commander.MoveGroupCommander(group_name)
#     display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
#                                                    moveit_msgs.msg.DisplayTrajectory,
#                                                    queue_size=20)

#     planning_frame = move_group.get_planning_frame()
#     eef_link = move_group.get_end_effector_link()
#     group_names = robot.get_group_names()
#     r = rospy.Rate(10)
#     # Misc variables
#     self.box_name = ''
#     self.robot = robot
#     self.scene = scene
#     self.move_group = move_group
#     self.display_trajectory_publisher = display_trajectory_publisher
#     self.planning_frame = planning_frame
#     self.eef_link = eef_link
#     self.group_names = group_names


#   def go_to_joint_state(self,msg):
#     move_group = self.move_group
#     joint_goal = move_group.get_current_joint_values()
#     print(q)
#     if msg == 0:
# 		joint_goal[0] = 0
# 		joint_goal[1] = radians(-90)
# 		joint_goal[2] = 0
#     elif msg == 1:
# 		joint_goal[0] = radians(49.48)
# 		joint_goal[1] = radians(-98.66)
# 		joint_goal[2] = radians(132.56)
#     elif msg == 2:
#     	joint_goal[0] = radians(-45.77)
#     	joint_goal[1] = radians(-70.59)
#     	joint_goal[2] = radians(95.41)
#     elif msg == 3:
#     	joint_goal[0] = 0
#     	joint_goal[1] = radians(-90)
#     	joint_goal[2] = 0

#     move_group.go(joint_goal, wait=True)
#     move_group.stop()
#     current_joints = move_group.get_current_joint_values()
#     return all_close(joint_goal, current_joints, 0.01)


  # def go_to_pose_goal(self, msg):
  #   move_group = self.move_group
  #   pose_goal = self.move_group.get_current_pose().pose
  #   # pose_goal = geometry_msgs.msg.Pose()
  #   # pose_goal.orientation.w = 1.0
  #   # pose_goal.position.x = msg.markers[0].pose.pose.position.x
  #   # pose_goal.position.y = msg.markers[0].pose.pose.position.y
  #   try:
  #       pose_goal.position.z = msg.markers[0].pose.pose.position.z
  #   except:
  #       print("No markers found!")
  #   move_group.set_pose_target(pose_goal)

  #   plan = move_group.go(wait=True)
  #   move_group.stop()
  #   move_group.clear_pose_targets()

  #   current_pose = self.move_group.get_current_pose().pose
  #   print("Current z pose is: " + str(current_pose.position.z))
  #   try:
  #       print("Target z pose is: " + str(msg.markers[0].pose.pose.position.z))
  #   except:
  #       print("")
  #   print("")
  #   return all_close(pose_goal, current_pose, 0.01)


