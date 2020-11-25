// Libraries 
#include <iostream>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
using namespace std;

class Controller {
  string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  const robot_state::JointModelGroup* joint_model_group;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  geometry_msgs::Pose target_pose1;


public:
  Controller():move_group(PLANNING_GROUP) {  };
  void demo1();
  void getpos();
  void movej();
  void movec();
  void callback(ar_track_alvar_msgs::AlvarMarkers data);
};


void Controller::demo1() {
  target_pose1.position.x = 0.1;
  target_pose1.position.y = 0.3;
  target_pose1.position.z = 0.5;
  // Move to pos1 
  while (1) {
    target_pose1.position.x += 0.02;
    move_group.setPoseTarget(target_pose1);
    move_group.move();
  }
}


void Controller::callback(ar_track_alvar_msgs::AlvarMarkers data) {
  ROS_INFO("Callback called");
  try {
    target_pose1.position.x = data.markers[0].pose.pose.position.z;
    target_pose1.position.y = 0.3;
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);
    move_group.move();
  }
  catch (const exception& e) { cout << data; }
  
}


// Main code
int main(int argc,char** argv) {
  ros::init(argc, argv, "ar_follower");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  Controller robot;
  ros::Subscriber sub = node_handle.subscribe("ar_pose_marker", 1, &Controller::callback, &robot);

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // Move Group set up
  // static const std::string PLANNING_GROUP = "manipulator";
  // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // // Robot Model set up
  // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  // ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  // robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  // kinematic_state->setToDefaultValues();
  // const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get robot to move to specified position in C++

  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.x = 1.0;
  // // target_pose1.orientation.y = 1.0;
  // // target_pose1.orientation.z = 1.0;
  // // target_pose1.orientation.w = 1.0;
  // target_pose1.position.x = 0.28;
  // target_pose1.position.y = -0.2;
  // target_pose1.position.z = 0.7;
  // move_group.setPoseTarget(target_pose1);
  // move_group.move();
  // ros::spin();
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  ros::waitForShutdown();
  return 0;
}

