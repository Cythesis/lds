#ifndef CONTROLLER
#define CONTROLLER

#include <iostream>
#include <unistd.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "lds/move_joint.h"
#include "lds/move_pose.h"
#include "lds/move_beer.h"
#include "lds/pose.h"
#include "lds/request.h"
#include <math.h>




using namespace std;

class Controller {
public: 

	Controller():move_group(PLANNING_GROUP),robot_model_loader("robot_description"),kinematic_model(robot_model_loader.getModel()) {
		target_pose.orientation.x = 0.1;
		target_pose.orientation.y = 0.1;
		target_pose.orientation.z = 0.5;
		target_pose.orientation.w = 1;
		target_pose.position.x = 0;
		target_pose.position.y = 0;
		target_pose.position.z = 0;
		// move_group.setPlannerId("PRMkConfigDefault");
		move_group.setPlanningTime(3);
	};

	bool arMarkerFound = 0;
	

	void animate(std::vector<double> joints);
	void move(geometry_msgs::Pose pose);
	vector<double> getpos();
	Eigen::MatrixXd jacob0();

	int moveIK(geometry_msgs::Pose pose);
	int moveCartesian(geometry_msgs::Pose start_pose,geometry_msgs::Pose end_pose);
	void moveOCM(geometry_msgs::Pose pose,moveit_msgs::Constraints constraint);
	void demo1();
	void addObject();
	void addObject2();
	void attachCup();
	void demo_stage_1();
	void moveBase(double x);
	void detachCup();
	geometry_msgs::Pose modifyArTransform(geometry_msgs::Transform transform);

	void arCallback(ar_track_alvar_msgs::AlvarMarkers data);
	bool moveJointCallback(lds::move_joint::Request &req, lds::move_joint::Response &res);
	bool movePoseCallback(lds::move_pose::Request &req, lds::move_pose::Response &res);
	bool moveBeerCallback(lds::move_beer::Request &req, lds::move_beer::Response &res);
	bool moveCartesianCallback(lds::pose::Request &req, lds::pose::Response &res);
	bool attachObjectCallback(lds::request::Request &req, lds::request::Response &res);
	bool createEnvironmentCallback(lds::request::Request &req, lds::request::Response &res);
	void moveRobotCartesianPath();


private:
	string PLANNING_GROUP = "manipulator";
	
	
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	geometry_msgs::Pose target_pose;

	moveit::planning_interface::PlanningSceneInterface planning_scene;
	moveit::planning_interface::MoveGroupInterface move_group;

	robot_model_loader::RobotModelLoader robot_model_loader;
	robot_model::RobotModelPtr kinematic_model;
	moveit::core::RobotStatePtr robot_state;
	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

	std::vector<moveit_msgs::CollisionObject> collision_objects;
};

#endif