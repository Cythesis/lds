#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
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
from lds.srv import pose, poseResponse, request, requestResponse, move_pose, move_poseResponse
arFound = 0

def attachObject(attach):
	rospy.wait_for_service('attach_object')
	try:
		att_obj = rospy.ServiceProxy('attach_object', request)
		resp1 = att_obj(attach)
		return resp1.rt
	except:
		print("service call failed")

def createEnvironment(create):
	rospy.wait_for_service('create_environment')
	try:
		environment = rospy.ServiceProxy('create_environment', request)
		resp1 = environment(create)
		return resp1.rt
	except:
		print("service call failed")

def moveCartesian(x, y, z, qw, qx, qy, qz):
	rospy.wait_for_service('move_cartesian')
	try:
		motion = rospy.ServiceProxy('move_cartesian', pose)
		resp1 = motion(x,y,z,qw,qx,qy,qz)
		if (resp1.rt != 1):
			raw_input("ERROR . . .")
		return resp1.rt
	except:
		print("service call failed")
		raw_input("ERROR . . .")

def movePose(x, y, z, qw, qx, qy, qz, velocity, x_tolerance, y_tolerance, z_tolerance):
	rospy.wait_for_service('move_pose')
	try:
		motion = rospy.ServiceProxy('move_pose', move_pose)
		resp1 = motion(x,y,z,qw,qx,qy,qz,'',velocity,x_tolerance,y_tolerance,z_tolerance)
		return resp1.rt
	except:
		print("service call failed")


class inactive(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[1])

	def execute(self, userdata):
		createEnvironment(1)
		moveCartesian(0,0.5,0.25,0.707,0,0,0.707)
		
		raw_input("Press enter to activate motion . . .")
		return 1


class active(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[0,1])

	def execute(self, userdata):
		print("Active state entered. Robot will actively look for ARtags")
		return 1

class move_to_ar(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0,0.8,0.15,0.707,0,0,0.707)
		return 1

class move_closer(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0,0.9,0.15,0.707,0,0,0.707)
		return 1

class close_gripper(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		attachObject(1)
		return 1

class raise_cup(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0,0.9,0.2,0.707,0,0,0.707)
		return 1

class move_to_tap(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0.5,0,0.2,1,0,0,0)
		return 1

class lower_cup(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0.5,0,0.15,1,0,0,0)
		return 1

class open_gripper(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		attachObject(0)
		return 1

class back_away(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0.4,0,0.15,1,0,0,0)
		return 1

class move_to_handle(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0.4,0,0.6,1,0,0,0)
		return 1

class push_handle(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0.6,0,0.6,1,0,0,0)
		time.sleep(3)
		return 1

class return_handle(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0.45,0,0.6,1,0,0,0)
		return 1

class move_to_cup(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0.4,0,0.15,1,0,0,0)
		return 1

class engage_cup(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0.5,0,0.15,1,0,0,0)
		return 1

class close_gripper_1(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		attachObject(1)
		return 1

class pick_cup_up(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0.5,0,0.2,1,0,0,0)
		return 1

class move_cup_back(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		movePose(0,0.9,0.2,0.707,0,0,0.707,0.1,0,0,0)
		time.sleep(2)
		return 1

class lower_the_cup(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0,0.9,0.15,0.707,0,0,0.707)
		return 1

class release_cup(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		attachObject(0)
		return 1

class Home(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=[1])

	def execute(self, userdata):
		moveCartesian(0,0.5,0.25,0.707,0,0,0.707)
		return 1




if __name__ == '__main__':
	rospy.init_node('automas')
	sm = smach.StateMachine(outcomes=['Success','Failed'])

	with sm:
		smach.StateMachine.add('Inactive', inactive(), transitions={1:'Active'})
		smach.StateMachine.add('Active', active(), transitions={1:'move_to_ar',0:'Failed'})
		smach.StateMachine.add('move_to_ar', move_to_ar(), transitions={1:'move_closer'})
		smach.StateMachine.add('move_closer', move_closer(), transitions={1:'close_gripper'})
		smach.StateMachine.add('close_gripper', close_gripper(), transitions={1:'raise_cup'})
		smach.StateMachine.add('raise_cup', raise_cup(), transitions={1:'move_to_tap'})
		smach.StateMachine.add('move_to_tap', move_to_tap(), transitions={1:'lower_cup'})
		smach.StateMachine.add('lower_cup', lower_cup(), transitions={1:'open_gripper'})
		smach.StateMachine.add('open_gripper', open_gripper(), transitions={1:'back_away'})
		smach.StateMachine.add('back_away', back_away(), transitions={1:'move_to_handle'})
		smach.StateMachine.add('move_to_handle', move_to_handle(), transitions={1:'push_handle'})
		smach.StateMachine.add('push_handle', push_handle(), transitions={1:'return_handle'})
		smach.StateMachine.add('return_handle', return_handle(), transitions={1:'move_to_cup'})
		smach.StateMachine.add('move_to_cup', move_to_cup(), transitions={1:'engage_cup'})
		smach.StateMachine.add('engage_cup', engage_cup(), transitions={1:'close_gripper_1'})
		smach.StateMachine.add('close_gripper_1', close_gripper_1(), transitions={1:'pick_cup_up'})
		smach.StateMachine.add('pick_cup_up', pick_cup_up(), transitions={1:'move_cup_back'})
		smach.StateMachine.add('move_cup_back', move_cup_back(), transitions={1:'lower_the_cup'})
		smach.StateMachine.add('lower_the_cup', lower_the_cup(), transitions={1:'release_cup'})
		smach.StateMachine.add('release_cup', release_cup(), transitions={1:'Home'})
		smach.StateMachine.add('Home', Home(), transitions={1:'Success'})

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	outcome = sm.execute()
