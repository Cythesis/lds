#include "ros/ros.h"
#include "controller.cpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>



int state = 1;

void printVector(std::vector <double> const &a) {
   std::cout << "The vector elements are : ";
   for(int i=0; i < a.size(); i++)
   std::cout << a.at(i) << ' ';
}

geometry_msgs::Pose modifyArTransform(geometry_msgs::Transform transform) {
	geometry_msgs::Pose pose;
	tf2::Quaternion q_original, q_mod, q_modified;

  	tf2::convert(transform.rotation, q_original);
	double r=0,p=0,y=-M_PI/2;
	q_mod.setRPY(r,p,y);
	q_modified = q_mod * q_original;
	q_modified.normalize();
	tf2::convert(q_modified, transform.rotation);

	pose.position.x = transform.translation.x;
	pose.position.y = transform.translation.y-0.15;
	pose.position.z = transform.translation.z;
	pose.orientation = transform.rotation;
	return pose;
}



int main(int argc, char **argv)
{
  // Initialization
  ros::init(argc, argv, "main");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  Controller ur5;
  ros::Rate rate(10.0);
  
  // Nodes initialization
  ros::Subscriber sub = node.subscribe("ar_pose_marker", 1, &Controller::arCallback, &ur5);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Variable initialization
  geometry_msgs::Transform transform;
  geometry_msgs::Pose modifiedPose;
  vector<double> home, joint0;
  home = {-1.6, -1.727, -2.203, -0.808, 1.595, -0.031};
  joint0 = {1.2317, -1.4238, 2.06088, -0.6379, 1.232, -3.141};

  geometry_msgs::Pose pose0,pose1,pose2,pose3,pose4,pose5,pose6,pose7,pose8;
  moveit_msgs::OrientationConstraint ocm4;
  moveit_msgs::Constraints constraint4;
  
  // Move to home
  pose0.position.x = 0;
  pose0.position.y = 0.5;
  pose0.position.z = 0.25;
  pose0.orientation.w = 0.707;
  pose0.orientation.x = 0;
  pose0.orientation.y = 0;
  pose0.orientation.z = 0.707;


  // Move in dront of ARtag
  pose1.position.x = 0;
  pose1.position.y = 0.8;
  pose1.position.z = 0.15;
  pose1.orientation.w = 0.707;
  pose1.orientation.y = 0.0;
  pose1.orientation.x = 0.0;
  pose1.orientation.z = 0.707;

  // Move close to the drink
  pose2.position.x = 0;
  pose2.position.y = 0.90;
  pose2.position.z = 0.15;
  pose2.orientation.w = 0.707;
  pose2.orientation.y = 0.0;
  pose2.orientation.x = 0.0;
  pose2.orientation.z = 0.707;

  // Bring the drink up
  pose3.position.x = 0;
  pose3.position.y = 0.8;
  pose3.position.z = 0.2;
  pose3.orientation.w = 0.707;
  pose3.orientation.y = 0.0;
  pose3.orientation.x = 0.0;
  pose3.orientation.z = 0.707;

  // Move to drop off position
  pose4.position.x = 0.7;
  pose4.position.y = 0;
  pose4.position.z = 0.2;
  pose4.orientation.w = 1;
  ocm4.link_name = "ee_link";
  ocm4.header.frame_id = "base_link";
  ocm4.orientation.w = 1;
  ocm4.absolute_x_axis_tolerance = 0.1;
  ocm4.absolute_y_axis_tolerance = 0.1;
  ocm4.absolute_z_axis_tolerance = 2*M_PI;
  ocm4.weight = 1.0;
  constraint4.orientation_constraints.push_back(ocm4);


  // Lower the drink down
  pose5.position.x = 0.5;
  pose5.position.y = 0;
  pose5.position.z = 0.15;
  pose5.orientation.w = 1;
  // pose5.orientation.x = -0.707;

  // Move up to tap
  pose6.position.x = 0.5;
  pose6.position.y = 0;
  pose6.position.z = 0.4;
  pose6.orientation.w = 1;
  // pose6.orientation.x = -0.707;

  // Turn on tap
  pose7.position.x = 0.55;
  pose7.position.y = 0;
  pose7.position.z = 0.4;
  pose7.orientation.w = 1;
  // pose7.orientation.x = -0.707;

  // Turn on tap
  pose8.position.x = 0.8;
  pose8.position.y = 0;
  pose8.position.z = 0.2;
  pose8.orientation.w = 1;
  // pose8.orientation.x = -0.707;

  




  // Add collision objects; n
  ur5.addObject2();
  int status;
  // Main Loop
  while (node.ok()) {
  	switch(state) {
  		case 1:
  			ROS_INFO("Attempting to move to home position");
  			ur5.animate(joint0);
  			state = 2;
  			break;
  		case 2:
  			ROS_INFO("Attempting to move to move to AR tag");
        sleep(1);
  			status = ur5.moveCartesian(pose0, pose1);
        while(1) {
          status = ur5.moveCartesian(pose0, pose1);
          cout << "------------------";
          cout << "Status message is:";
          cout << status << endl;
        }
  			state = 3;
  			break;
  		case 3:
  			ROS_INFO("Attempting to move to gripping position");
        sleep(1);
  			cout << ur5.moveCartesian(pose1, pose2);
        while(status!=1) {
          status = ur5.moveCartesian(pose1, pose2);
          cout << status;
        }
  			ur5.attachCup();
  			state = 4;
  			break;
  		case 4:
  			ROS_INFO("Attempting to move to pick the cup up");
  			cout << ur5.moveCartesian(pose2, pose3);
  			state = 5;
  			break;
  		case 5:
  			ROS_INFO("Attempting to move cup to tap");
  			ur5.moveOCM(pose4,constraint4);
  			ROS_INFO("OCM attempted");
  			state = 6;
  			break;
  		case 6:
  			ROS_INFO("Attempting to move to lower and detach cup");
  			ur5.moveCartesian(pose4,pose5);
  			ur5.detachCup();
  			state = 7;
  			break;
    	case 7:
    		ROS_INFO("Attempting to move to tap");
    		ur5.moveCartesian(pose5,pose6);
    		state = 8;
    		break;
    	case 8:
    		ROS_INFO("Attempting to push tap");
    		ur5.moveCartesian(pose6,pose7);
    		state = 9;
    		break;
    	case 9:
    		ROS_INFO("Attempting to back away from tap");
    		ur5.moveCartesian(pose7,pose6);
    		state = 10;
    		break;
    	case 10:
    		ROS_INFO("Attempting to move back to cup and attach");
    		ur5.moveCartesian(pose6,pose5);
    		ur5.attachCup();
    		state = 11;
    		break;
    	case 11:
    		ROS_INFO("Attempting to move cup to original spot");
    		ur5.moveCartesian(pose5,pose4);
    		state = 12;
    		break;
    	case 12:
    		ROS_INFO("Attempting to move cup to original spot");
    		ur5.moveOCM(pose3, constraint4);
    		state = 13;
    		break;
    	case 13:
    		ROS_INFO("Attempting to lower cup to original spot and detach");
    		ur5.moveCartesian(pose3,pose2);
    		ur5.detachCup();
    		state = 14;
    		break;
    	case 14:
    		ROS_INFO("Attempting to go home");
    		ur5.animate(joint0);
    		state = 14;
    		break;


  	}
  			//   	if (ur5.arMarkerFound) {
		  	// 	transform = tfBuffer.lookupTransform("world", "ar_marker_0", ros::Time(0)).transform;
		  	// 	modifiedPose = modifyArTransform(transform);
		  	// 	ur5.moveIK(modifiedPose);
		  	// } else {
		  	// 	ur5.moveIK(pose_s1);
		  	// }

	  // 	case 2:
	  // 		// Move into grabbing position
	  // 		break;
	  // 	case 3:
	  // 		// Close the gripper
	  // 		// May include checks here to see whether robot is gripping somehting or not
	  // 		break;
	  // 	case 4:
	  // 		// Move the cup under the tap
	  // 		break;
	  // 	case 5: 
	  // 		// Turn on (and off) the tap
	  // 		break;
	  // 	case 6: 
	  // 		// Return to gripping the cup
	  // 		break;
	  // 	case 7: 
	  // 		// Move to original positon while keeping end effector leveled
	  // 		break;
	  // 	default:
	  // 		// Code block
  	// }
  }


  // // Settings
  // ROS_INFO("Checkpoint 1");

  // ros::AsyncSpinner spinner(0);
  // spinner.start();
  // Controller ur5;

  // ros::Subscriber sub = n.subscribe("ar_pose_marker", 1, &Controller::arCallback, &ur5);

  // // Main Code
  // // ur5.demo1();

  ros::waitForShutdown();
  return 0;
}

