#!/usr/bin/env python

import sys
import pdb
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


# Determine whether actual and goal state is within tolerance
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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    self.box_name = ''
    self.ground_name = ''
    self.barrier_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self,j0,j1,j2,j3,j4,j5):
    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = j0
    joint_goal[1] = j1
    joint_goal[2] = j2
    joint_goal[3] = j3
    joint_goal[4] = j4
    joint_goal[5] = j5
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self,x,y,z,qw,qx,qy,qz):
    move_group = self.move_group
    joint_coefficient = 1.5
    points = []
    attempts = 0
    move_group.set_planning_time(0.1)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = qw
    pose_goal.orientation.x = qx
    pose_goal.orientation.y = qy
    pose_goal.orientation.z = qz
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    move_group.set_pose_target(pose_goal)

    while (len(points) == 0):
        start_joint = []
        end_joint = []
        delta_joint = []
        attempts += 1
        plan = move_group.plan()
        points = plan.joint_trajectory.points
        if (len(points) != 0):
            for i in range(5):
                start_joint.append(points[0].positions[i])
                end_joint.append(points[len(points)-1].positions[i])
                delta_joint.append(start_joint[i]-end_joint[i])
                if (delta_joint[i] > joint_coefficient):
                    points = []
                    joint_coefficient += 0.05
                    break
    

    print("Planning phase finished with:")
    print("Attempts: " + str(attempts))
    print("Max joint change: " + str(max(delta_joint)))
    print("^>^>^>^>^>^>^>^>^>^>^>^>")
    # pdb.set_trace()
    # raw_input()
    move_group.execute(plan)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def go_to_cartesian_path(self,x,y,z,qw,qx,qy,qz):
    move_group = self.move_group
    waypoints = []
    fraction = 0.0
    attempts = 0
    max_attempts = 20

    while (fraction < 0.98):
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        wpose.orientation.w = qw
        wpose.orientation.x = qx
        wpose.orientation.y = qy
        wpose.orientation.z = qz
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
                                           waypoints,   
                                           0.01,        
                                           2.0)         
        attempts += 1
        if (attempts > max_attempts):
            sys.exit()
        if (fraction >= 0.98):
            move_group.execute(plan,wait=True)

    return plan, fraction


  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory);


  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan, wait=True)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False


  def create_environment(self, timeout=4):
    scene = self.scene

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.orientation.x = 0
    box_pose.pose.orientation.y = 0
    box_pose.pose.orientation.z = 0
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = -0.005
    box_name = "ground"
    scene.add_box(box_name, box_pose, size=(2, 2, 0.01))

    self.ground_name = box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def add_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 1
    box_pose.pose.position.z = 0.15
    box_name = "cup"
    scene.add_box(box_name, box_pose, size=(0.12, 0.12, 0.3))

    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    scene.attach_box(eef_link, box_name)
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object(box_name)
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    ur10 = MoveGroupPythonIntefaceTutorial()
    ur10.create_environment()
    ur10.detach_box()
    ur10.add_box()
    

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    ur10.go_to_pose_goal(0,0.5,0.25,0.707,0,0,0.707)
    ur10.go_to_pose_goal(0,0.8,0.15,0.707,0,0,0.707)
    ur10.go_to_pose_goal(0,0.9,0.15,0.707,0,0,0.707)
    ur10.attach_box()
    ur10.go_to_cartesian_path(0,0.9,0.2,0.707,0,0,0.707)
    ur10.go_to_cartesian_path(0.5,0,0.2,1,0,0,0)
    ur10.go_to_pose_goal(0.5,0,0.150,1,0,0,0)
    ur10.detach_box()
    ur10.go_to_pose_goal(0.4,0,0.15,1,0,0,0)
    ur10.go_to_pose_goal(0.4,0,0.6,1,0,0,0)
    ur10.go_to_pose_goal(0.6,0,0.6,1,0,0,0)
    ur10.go_to_pose_goal(0.45,0,0.6,1,0,0,0)
    ur10.go_to_pose_goal(0.4,0,0.15,1,0,0,0)
    ur10.go_to_pose_goal(0.5,0,0.15,1,0,0,0)
    ur10.attach_box()
    ur10.go_to_cartesian_path(0.5,0,0.2,1,0,0,0)
    ur10.go_to_cartesian_path(0,0.9,0.2,0.707,0,0,0.707)
    ur10.go_to_pose_goal(0,0.9,0.16,0.707,0,0,0.707)
    ur10.detach_box()
    ur10.go_to_pose_goal(0,0.5,0.25,0.707,0,0,0.707)







    # print "============ Press `Enter` to execute a movement using a pose goal ..."
    # raw_input()
    # tutorial.go_to_pose_goal()

    # print "============ Press `Enter` to plan and display a Cartesian path ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path()

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # tutorial.display_trajectory(cartesian_plan)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    # tutorial.execute_plan(cartesian_plan)

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_box()

    # print "============ Press `Enter` to attach a Box to the Panda robot ..."
    # raw_input()
    # tutorial.attach_box()

    # print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    # tutorial.execute_plan(cartesian_plan)

    # print "============ Press `Enter` to detach the box from the Panda robot ..."
    # raw_input()
    # tutorial.detach_box()

    # print "============ Press `Enter` to remove the box from the planning scene ..."
    # raw_input()
    # tutorial.remove_box()

    # print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/melodic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
