#include <ros/ros.h>
#include <actionlib/client/simple_action_client>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/MoveGroupActionGoal.h>

int main (int argc, char **argv) {
	ros::init(argc,argv, "test_move");
	actionlib::SimpleActionClient<moveit_msgs::MoveGroupActionGoal> ac("move_group", true);
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	ROS_INFO("Action server started, sending goal.");
	moveit_msgs::MoveGroupActionGoal goal;
	goal.request.start_state.multi_dof_joint_state.transforms.translation.x = 0;
	goal.request.start_state.multi_dof_joint_state.transforms.translation.y = 1;
	goal.request.start_state.multi_dof_joint_state.transforms.translation.z = 0.15;
	ac.sendGoal(goal);
	bool finished_before_timeout = ac.waitForResult(ros::Duration(5.0));
	if (finished_before_timeout)
	{
	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	ROS_INFO("Action did not finish before the time out.");

	//exit
	return 0;
}
