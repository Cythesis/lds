#!/usr/bin/env python

# UR libraries
import sys
import pdb
import copy
import rospy
import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from lds.srv import pose,poseResponse

# Email libraries
import time
from itertools import chain
import email
import imaplib
import pdb


def move(x, y, z, qw, qx, qy, qz, moveType='', vel_scaling = 0.0, acc_scaling = 0.0, x_tol = 0.0, y_tol = 0.0, z_tol = 0.0, planning_time = 0.0, threshold = 0.0):
    rospy.wait_for_service('move_robot')
    try:
        motion = rospy.ServiceProxy('move_robot', pose)
        resp1 = motion(moveType,x,y,z,qw,qx,qy,qz,vel_scaling,acc_scaling,x_tol,y_tol,z_tol,planning_time,threshold)
        if (resp1.rt != 1):
            raw_input("ERROR . . .")
        return resp1.rt
    except:
        print("service call failed")
        raw_input("ERROR . . .")


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


class MoveGroupPythonInteface(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()
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

  def go_to_pose_goal(self,x,y,z,qw,qx,qy,qz):
    move_group = self.move_group
    threshold = 1.5
    increase = 0.1
    plan_flag = 0
    t_attempts = 0
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

    while (plan_flag == 0):
        plan = move_group.plan()
        points = plan.joint_trajectory.points
        (plan_flag, delta_joint) = self.check_plan(points,threshold)
        threshold += 0.1
        attempts += 1

    print("Planning phase finished with:")
    print("Type: Pose")
    print("Target: " + str(x) + ", " + str(y) + ", " + str(z))
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
    scene.add_box(box_name, box_pose, size=(4, 4, 0.01))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.orientation.x = 0
    box_pose.pose.orientation.y = 0
    box_pose.pose.orientation.z = 0
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.5
    box_pose.pose.position.z = 0.5
    box_name = "obstacle"
    # scene.add_box(box_name, box_pose, size=(0.3, 0.3, 0.3))

    self.ground_name = box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def create_environment2(self, timeout=4):
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
    scene.add_box(box_name, box_pose, size=(4, 4, 0.01))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.orientation.x = 0
    box_pose.pose.orientation.y = 0
    box_pose.pose.orientation.z = 0
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.5
    box_pose.pose.position.z = 0.5
    box_name = "obstacle"
    scene.add_box(box_name, box_pose, size=(0.3, 0.3, 0.3))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.orientation.x = 0
    box_pose.pose.orientation.y = 0
    box_pose.pose.orientation.z = 0
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.5
    box_pose.pose.position.z = 1.2
    box_name = "obstacle2"
    scene.add_box(box_name, box_pose, size=(0.3, 0.3, 0.5))

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
  
  def remove_all(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name='obstacle')
    scene.remove_attached_object(eef_link, name='obstacle2')
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
  

  def remove_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object(box_name)
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def search_string(uid_max, criteria):
    c = list(map(lambda t: (t[0], '"'+str(t[1])+'"'), criteria.items())) + [('UID', '%d:*' % (uid_max+1))]
    return '(%s)' % ' '.join(chain(*c))
    # Produce search string in IMAP format:
    #   e.g. (FROM "me@gmail.com" SUBJECT "abcde" BODY "123456789" UID 9999:*)


def get_first_text_block(msg):
    type = msg.get_content_maintype()

    if type == 'multipart':
        for part in msg.get_payload():
            if part.get_content_maintype() == 'text':
                return part.get_payload()
    elif type == 'text':
        return msg.get_payload()



def demo1(): # Standard movements
    ur10 = MoveGroupPythonInteface()
    ur10.create_environment()
    # pdb.set_trace()
    ur10.detach_box()
    ur10.add_box()
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
    ur10.go_to_pose_goal(0,0.9,0.151,0.707,0,0,0.707)
    ur10.detach_box()
    ur10.go_to_pose_goal(0,0.5,0.25,0.707,0,0,0.707)

def demo2(): # Random movement 
    ur10 = MoveGroupPythonInteface()
    ur10.remove_all()
    ur10.create_environment()
    while not rospy.is_shutdown():
        ranx = random.randint(-79,79)/100.0
        ranz = random.randint(10,79)/100.0
        print("<"+str(ranx)+","+str(ranz)+">")

        ur10.go_to_pose_goal(ranx,0.5,ranz,0.707,0,0,0.707)
        rospy.sleep(0.3)

def demo3(): # Random movement 
    ur10 = MoveGroupPythonInteface()
    ur10.create_environment()
    while not rospy.is_shutdown():
        ranx = random.randint(-79,79)/100.0
        ranz = random.randint(10,79)/100.0
        print("<"+str(ranx)+","+str(ranz)+">")

        ur10.go_to_cartesian_path(ranx,0.5,ranz,0.707,0,0,0.707)
        rospy.sleep(0.3)


def demo4(): # Random movement 
    ur10 = MoveGroupPythonInteface()
    ur10.create_environment2()
    ur10.go_to_pose_goal(0.48,0.5,0.55,0.707,0,0,0.707)
    ur10.go_to_pose_goal(-0.36,0.5,0.6,0.707,0,0,0.707)

def demo0():
    demo = 0
    ur10 = MoveGroupPythonInteface()
    ur10.create_environment()
    imap_ssl_host = 'imap.gmail.com'  # imap.mail.yahoo.com
    imap_ssl_port = 993
    username = 'command.service.ros@gmail.com'
    password = 'ubuntu20'

    criteria = {
        'FROM':    'command.service.ros@gmail.com'
        # 'SUBJECT': 'Your First Message',
        # 'BODY':    'Sincerely',
    }
    uid_max = 0
    server = imaplib.IMAP4_SSL(imap_ssl_host, imap_ssl_port)
    server.login(username, password)
    server.select('INBOX')

    result, data = server.uid('search', None, search_string(uid_max, criteria))

    uids = [int(s) for s in data[0].split()]
    if uids:
        uid_max = max(uids)
        # Initialize `uid_max`. Any UID less than or equal to `uid_max` will be ignored subsequently.

    server.logout()
    while 1:
        # Have to login/logout each time because that's the only way to get fresh results.

        server = imaplib.IMAP4_SSL(imap_ssl_host, imap_ssl_port)
        server.login(username, password)
        server.select('Inbox')

        result, data = server.uid('search', None, search_string(uid_max, criteria))

        uids = [int(s) for s in data[0].split()]
        for uid in uids:
            # Have to check again because Gmail sometimes does not obey UID criterion.
            if uid > uid_max:
                result, data = server.uid('fetch', uid, '(RFC822)')  # fetch entire message
                msg = email.message_from_string(data[0][1])
                uid_max = uid
            
                text = get_first_text_block(msg)
                print ':::::::::::: New message ::::::::::::'
                demo = int(msg['subject'])
        server.logout()
        if (demo == 1):
            demo1()
            demo = 0
        elif (demo == 2):
            demo2()
            demo = 0
        elif (demo == 3):
            demo3()
            demo = 0
        rospy.sleep(1)

def main():
  try:
    demo = int(raw_input("Which demo to perform: "))
    if (demo == 1):
        demo1() # Beer path
    elif (demo == 2):
        demo2() # Random pose
    elif (demo == 3):
        demo3() # Random cartesian
    elif (demo == 4):
        demo4() # Random pose with obstacles
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
