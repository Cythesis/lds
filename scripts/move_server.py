#!/usr/bin/env python

from __future__ import print_function

from lds.srv import pose,poseResponse
import rospy
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

class MoveGroupPythonInteface(object):
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_server')
    rospy.Service('move_robot', pose, self.service)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    self.move_group = move_group

  def check_plan(self,points,threshold):
    delta_joint = [0.0,0.0,0.0,0.0,0.0,0.0]
    if (len(points) != 0):
        for k in range(len(points)-1):
            for i in range(6):
                delta_joint[i] += abs(points[k+1].positions[i] - points[k].positions[i])
                if (delta_joint[i] > threshold):
                    return 0, delta_joint
    if (len(points) == 0):
        return 0, delta_joint
    return 1, delta_joint

  def go_to_pose_goal(self,x,y,z,qw,qx,qy,qz,planning_time,threshold):
    move_group = self.move_group
    plan_flag = 0
    attempts = 0
    move_group.set_planning_time(planning_time)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = qw
    pose_goal.orientation.x = qx
    pose_goal.orientation.y = qy
    pose_goal.orientation.z = qz
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    move_group.set_pose_target(pose_goal)

    while (plan_flag == 0):
        plan = move_group.plan()
        points = plan.joint_trajectory.points
        (plan_flag, delta_joint) = self.check_plan(points,threshold)
        threshold += 0.05
        attempts += 1    
        if (attempts > 100):
            return 0

    print("Planning phase finished with:")
    print("Type: Pose")
    print("Target: " + str(x) + ", " + str(y) + ", " + str(z))
    print("Attempts: " + str(attempts))
    print("Max joint change: " + str(max(delta_joint)))
    print("^>^>^>^>^>^>^>^>^>^>^>^>")
    move_group.execute(plan)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    return 1

  def go_to_cartesian_path(self,x,y,z,qw,qx,qy,qz):
    move_group = self.move_group
    waypoints = []
    fraction = 0.0
    attempts = 0
    max_attempts = 10
    threshold = 1.5

    while (fraction < 0.98):
        plan_flag = 0
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
        while (plan_flag == 0):
            (plan, fraction) = move_group.compute_cartesian_path(
                                           waypoints,   
                                           0.01,        
                                           2.0)   
            points = plan.joint_trajectory.points
            (plan_flag, delta_joint) = self.check_plan(points,threshold)
            threshold += 0.05
            attempts += 1  
            if (attempts > 100):
                sys.exit()
        

    print("Planning phase finished with:")
    print("Type: Cartesian")
    print("Attempts: " + str(attempts))
    print("Max joint change: " + str(max(delta_joint)))
    print("^>^>^>^>^>^>^>^>^>^>^>^>")
    move_group.execute(plan,wait=True)

    return plan, fraction

  def service(self,req):
      # set path constraint
      if ((req.x_tolerance != 0) and (req.y_tolerance != 0) and (req.z_tolerance != 0)):
          constraint = moveit_msgs.msg.Constraints()
          orientation_constraints = moveit_msgs.msg.OrientationConstraint()
          orientation_constraints.orientation.w = req.qw
          orientation_constraints.orientation.x = req.qx
          orientation_constraints.orientation.y = req.qy
          orientation_constraints.orientation.z = req.qz
          orientation_constraints.header.frame_id = "base_link"
          orientation_constraints.link_name = self.move_group.get_end_effector_link()
          orientation_constraints.absolute_x_axis_tolerance = req.x_tolerance
          orientation_constraints.absolute_y_axis_tolerance = req.y_tolerance
          orientation_constraints.absolute_z_axis_tolerance = req.z_tolerance
          orientation_constraints.weight = 1
          constraint.orientation_constraints.append(orientation_constraints)
          self.move_group.set_path_constraints(constraint)

      if (req.planning_time == 0):
        req.planning_time = 0.1

      if (req.joint_threshold == 0):
        req.joint_threshold = 1.5

      if (req.velocity_scaling == 0):
        self.move_group.set_max_velocity_scaling_factor(1)
      else:
        self.move_group.set_max_velocity_scaling_factor(req.velocity_scaling)

      if (req.acceleration_scaling == 0):
        self.move_group.set_max_acceleration_scaling_factor(1)
      else:
        self.move_group.set_max_acceleration_scaling_factor(req.acceleration_scaling)

      if (req.type == "pose"):
        self.go_to_pose_goal(req.x,req.y,req.z,req.qw,req.qx,req.qy,req.qz,req.planning_time,req.joint_threshold)
      elif (req.type == "cartesian"):
        self.go_to_cartesian_path(req.x,req.y,req.z,req.qw,req.qx,req.qy,req.qz)
      else:
        self.go_to_pose_goal(req.x,req.y,req.z,req.qw,req.qx,req.qy,req.qz,req.planning_time,req.joint_threshold)

      print("Successful service call")
      self.move_group.clear_path_constraints()
      self.move_group.set_max_velocity_scaling_factor(1.0)
      self.move_group.set_max_acceleration_scaling_factor(1.0)

      return 1

def main():
  try:
    ur10 = MoveGroupPythonInteface()
    rospy.spin()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
