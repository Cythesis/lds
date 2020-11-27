#include "controller.h"

void Controller::animate(std::vector<double> joints) {
	move_group.setJointValueTarget(joints);
	move_group.move();
	robot_state = move_group.getCurrentState();
}

vector<double> Controller::getpos() {
	vector<double> joint_values;
	robot_state->copyJointGroupPositions(joint_model_group, joint_values);
	return joint_values;
}

Eigen::MatrixXd Controller::jacob0() {
	Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
	Eigen::MatrixXd jacobian;
	robot_state->getJacobian(joint_model_group,
	                           robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
	                           reference_point_position, jacobian);
	return jacobian;
}

int Controller::moveIK(geometry_msgs::Pose pose) {
	move_group.setPoseTarget(pose);
	moveit::planning_interface::MoveItErrorCode status = move_group.move(); 
	return status.val;
}

void Controller::moveBase(double x) {
	std::vector<double> q = move_group.getCurrentJointValues();
	q[0] = q[0] + x;
	animate(q);
}

void Controller::moveOCM(geometry_msgs::Pose pose,moveit_msgs::Constraints constraint) {
	move_group.setPathConstraints(constraint);
	move_group.setPoseTarget(pose);
	move_group.move();
	move_group.clearPathConstraints(); 	
}

void Controller::demo1() {
	geometry_msgs::Pose pose;
  	pose.position.x = 0.2;
  	pose.position.y = 0.2;
  	pose.position.z = 0.55;

  	while(1) {
  		pose.position.x += 0.03;
  		moveIK(pose);
  	}
}

void Controller::addObject() {
  moveit_msgs::CollisionObject collision_object1;
  moveit_msgs::CollisionObject collision_object2;
  moveit_msgs::CollisionObject collision_object3;
  collision_object1.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object1.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 4;
  primitive.dimensions[1] = 4;
  primitive.dimensions[2] = 0.1;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = -0.05;

  collision_object1.primitives.push_back(primitive);
  collision_object1.primitive_poses.push_back(box_pose);
  collision_object1.operation = collision_object1.ADD;
  collision_objects.push_back(collision_object1);

  // Add another Object
  collision_object2.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object2.id = "box2";

  // Define a box to add to the world.
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 1;
  primitive.dimensions[2] = 2;

  // Define a pose for the box (specified relative to frame_id)
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = -1.2;
  box_pose.position.z = 0.5;

  collision_object2.primitives.push_back(primitive);
  collision_object2.primitive_poses.push_back(box_pose);
  collision_object2.operation = collision_object2.ADD;
  collision_objects.push_back(collision_object2);


  // Add another Object
  collision_object3.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.

  collision_object3.id = "recyclable_bottle";

  // Define a box to add to the world.
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.12;
  primitive.dimensions[1] = 0.3;
  primitive.dimensions[2] = 0.12;

  // Define a pose for the box (specified relative to frame_id)
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.2;
  box_pose.position.y = 0.7;
  box_pose.position.z = 0.2;

  collision_object3.primitives.push_back(primitive);
  collision_object3.primitive_poses.push_back(box_pose);
  collision_object3.operation = collision_object3.ADD;
  collision_objects.push_back(collision_object3);


  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene.addCollisionObjects(collision_objects);

  // move_group.attachObject("recyclable_bottle");
}

void Controller::addObject2() {
  // Initiate temporary variables
  moveit_msgs::CollisionObject collision_object1;
  moveit_msgs::CollisionObject collision_object2;
  moveit_msgs::CollisionObject collision_object3;

  // Create object to represent ground  
  collision_object1.header.frame_id = move_group.getPlanningFrame();
  collision_object1.id = "ground";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 4;
  primitive.dimensions[1] = 4;
  primitive.dimensions[2] = 0.1;
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = -0.05;
  collision_object1.primitives.push_back(primitive);
  collision_object1.primitive_poses.push_back(box_pose);
  collision_object1.operation = collision_object1.ADD;
  collision_objects.push_back(collision_object1);

  // Create object to represent tap
  collision_object2.header.frame_id = move_group.getPlanningFrame();
  collision_object2.id = "virtualBlock";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 2;
  primitive.dimensions[2] = 1;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = -1.2;
  box_pose.position.z = 0.5;
  collision_object2.primitives.push_back(primitive);
  collision_object2.primitive_poses.push_back(box_pose);
  collision_object2.operation = collision_object2.ADD;
  collision_objects.push_back(collision_object2);


  // Create object to represent cup
  collision_object3.header.frame_id = move_group.getPlanningFrame();
  collision_object3.id = "recyclable_bottle";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.12;
  primitive.dimensions[1] = 0.12;
  primitive.dimensions[2] = 0.3;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 1;
  box_pose.position.z = 0.15;
  collision_object3.primitives.push_back(primitive);
  collision_object3.primitive_poses.push_back(box_pose);
  collision_object3.operation = collision_object3.ADD;
  collision_objects.push_back(collision_object3);


  // Add created object to world
  ROS_INFO_NAMED("Demo 1", "Added an object into the world");
  // planning_scene.removeCollisionObjects("recyclable_bottle");
  planning_scene.addCollisionObjects(collision_objects);
}

void Controller::attachCup() {
	move_group.attachObject("recyclable_bottle");
}

void Controller::detachCup() {
	move_group.detachObject("recyclable_bottle");
}

int Controller::moveCartesian(geometry_msgs::Pose start_pose,geometry_msgs::Pose end_pose) {
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose);
  waypoints.push_back(end_pose);
  move_group.setMaxVelocityScalingFactor(0.1);
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0, trajectory);
  sleep(1);
  moveit::planning_interface::MoveItErrorCode status = move_group.execute(trajectory);
  return status.val;
}

geometry_msgs::Pose Controller::modifyArTransform(geometry_msgs::Transform transform) {
	geometry_msgs::Pose pose;
	tf2::Quaternion q_original, q_mod, q_modified;

  	tf2::convert(transform.rotation, q_original);
	double r=0,p=0,y=-M_PI;
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

/////////////////////////
/* Services Callbacks */
///////////////////////

void Controller::arCallback (ar_track_alvar_msgs::AlvarMarkers data) {
	if (!data.markers.empty()) {
		arMarkerFound = 1;
	} else {
		arMarkerFound = 0;
	}
}

bool Controller::moveJointCallback(lds::move_joint::Request &req, lds::move_joint::Response &res) {
	vector<double> q;
	if (req.rad) {
		q = {req.a, req.b, req.c, req.d, req.e, req.f};
	} else {
		q = {req.a * M_PI / 180.0, req.b* M_PI / 180.0, req.c* M_PI / 180.0, req.d* M_PI / 180.0, req.e* M_PI / 180.0, req.f* M_PI / 180.0};
	}
	res.rt = 1;
	animate(q);
	ROS_INFO("Service working correctly");
	return 1;
}

bool Controller::movePoseCallback(lds::move_pose::Request &req, lds::move_pose::Response &res) {
	geometry_msgs::Pose pose;
	pose.position.x = req.x;
	pose.position.y = req.y;
	pose.position.z = req.z;
	pose.orientation.w = req.qw;
	pose.orientation.x = req.qx;
	pose.orientation.y = req.qy;
	pose.orientation.z = req.qz;

	if (req.planning_method == "SBL") {
		move_group.setPlannerId("SBLkConfigDefault");
	} else if (req.planning_method == "EST"){
		move_group.setPlannerId("ESTkConfigDefault");
	} else if (req.planning_method == "LBKPIECE"){
		move_group.setPlannerId("LBKPIECEkConfigDefault");
	} else if (req.planning_method == "BKPIECE"){
		move_group.setPlannerId("BKPIECEkConfigDefault");
	} else if (req.planning_method == "KPIECE"){
		move_group.setPlannerId("KPIECEkConfigDefault");
	} else if (req.planning_method == "RRT"){
		move_group.setPlannerId("RRTkConfigDefault");
	} else if (req.planning_method == "RRTConnect"){
		move_group.setPlannerId("RRTConnectkConfigDefault");
	} else if (req.planning_method == "RRTstar"){
		move_group.setPlannerId("RRTstarkConfigDefault");
	} else if (req.planning_method == "TRRT"){
		move_group.setPlannerId("TRRTkConfigDefault");
	} else if (req.planning_method == "PRM"){
		move_group.setPlannerId("PRMkConfigDefault");
	} else if (req.planning_method == "PRMstar"){
		move_group.setPlannerId("PRMstarkConfigDefault");
	} else {
		move_group.setPlannerId("RRTConnectkConfigDefault");
	}

	moveit_msgs::OrientationConstraint ocm;
  	moveit_msgs::Constraints constraint;

  	ocm.link_name = "ee_link";
	ocm.header.frame_id = "base_link";
	ocm.orientation = pose.orientation;
	ocm.absolute_x_axis_tolerance = req.x_tolerance;
	ocm.absolute_y_axis_tolerance = req.y_tolerance;
	ocm.absolute_z_axis_tolerance = req.z_tolerance;
	double min_tolerance = 0.00001;

	if (req.x_tolerance < min_tolerance) {
		ocm.absolute_x_axis_tolerance = 3.14;
	}

	if (req.y_tolerance < min_tolerance) {
		ocm.absolute_y_axis_tolerance = 3.14;
	} 

	if (req.z_tolerance < min_tolerance) {
		ocm.absolute_z_axis_tolerance = 3.14;
	}

	ocm.weight = 1.0;
	constraint.orientation_constraints.push_back(ocm);


	move_group.setPathConstraints(constraint);	
	move_group.setPoseTarget(pose);
	move_group.setMaxVelocityScalingFactor(req.velocity_scaling);
	moveit::planning_interface::MoveItErrorCode status = move_group.move(); 
	move_group.clearPathConstraints(); 
	move_group.setMaxVelocityScalingFactor(1.0);

	res.rt = 1;
	ROS_INFO("Service working correctly");
	return 1;
}

bool Controller::moveBeerCallback(lds::move_beer::Request &req, lds::move_beer::Response &res) {
	geometry_msgs::Pose pose0, pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8, pose9; 
	geometry_msgs::Transform ArPose;
	moveit_msgs::OrientationConstraint ocm4;
  	moveit_msgs::Constraints constraint4;

	ArPose.translation.x = req.x;
	ArPose.translation.y = req.y;
	ArPose.translation.z = req.z;
	ArPose.rotation.w = req.qw;
	ArPose.rotation.x = req.qx;
	ArPose.rotation.y = req.qy;
	ArPose.rotation.z = req.qz;
	vector<double> home = {1.2317, -1.4238, 2.06088, -0.6379, 1.232, -3.141};

	// Modify the ar pose to ee pose
	pose1 = modifyArTransform(ArPose);

	// Calculate all the pose ee has to move to.
	pose0.position.x = 0;
	pose0.position.y = 0.5;
	pose0.position.z = 0.25;
	pose0.orientation.w = 0.707;
	pose0.orientation.x = 0;
	pose0.orientation.y = 0;
	pose0.orientation.z = 0.707;

	pose2 = pose1;
	pose2.position.y += 0.1;
	pose3 = pose1;
	pose3.position.z += 0.05;

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

	pose9.position.x = 0.5;
	pose9.position.y = 0;
	pose9.position.z = 0.2;
	pose9.orientation.w = 1;
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

	addObject2();

	

	// Execute the poses down here.
	ROS_INFO("Attempting to move to home position");
  	animate(home);
  	ROS_INFO("Attempting to move to move to AR tag");
  	moveCartesian(pose0, pose1);
  	ROS_INFO("Attempting to move to gripping position");
	moveCartesian(pose1, pose2);
	attachCup();
	ROS_INFO("Attempting to move to pick the cup up");
	moveCartesian(pose2, pose3);
	ROS_INFO("Attempting to move cup to tap");
	moveOCM(pose4,constraint4);
	ROS_INFO("OCM attempted");
	ROS_INFO("Attempting to move to lower and detach cup");
	moveCartesian(pose4,pose5);
	moveCartesian(pose5,pose5);
	detachCup();
	ROS_INFO("Attempting to move to tap");
	moveCartesian(pose5,pose6);
	ROS_INFO("Attempting to push tap");
	moveCartesian(pose6,pose7);
	ROS_INFO("Attempting to back away from tap");
	moveCartesian(pose7,pose6);
	ROS_INFO("Attempting to move back to cup and attach");
	moveCartesian(pose6,pose5);
	attachCup();
	ROS_INFO("Attempting to move cup to original spot");
	moveCartesian(pose5,pose9);
	ROS_INFO("Attempting to move cup to original spot");
	moveOCM(pose3, constraint4);
	ROS_INFO("Attempting to lower cup to original spot and detach");
	moveCartesian(pose3,pose2);
	detachCup();
	ROS_INFO("Attempting to go home");
	animate(home);

	return true;
}

bool Controller::moveCartesianCallback(lds::pose::Request &req, lds::pose::Response &res) {
	target_pose.position.x = req.x;
	target_pose.position.y = req.y;
	target_pose.position.z = req.z;
	target_pose.orientation.w = req.qw;
	target_pose.orientation.x = req.qx;
	target_pose.orientation.y = req.qy;
	target_pose.orientation.z = req.qz;
	moveit::planning_interface::MoveItErrorCode status;
	status.val = 0;
	int attempts = 5;
	int c_attemps = 0;

	while (status.val != 1) {
		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(target_pose);
		moveit_msgs::RobotTrajectory trajectory;
		double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0, trajectory);
		status = move_group.execute(trajectory);
		cout << "--------------------" << endl;
		cout << "|Execution complete|" << endl;
		cout << "--------------------" << endl;
		cout << "Exited with status: " << status.val << endl;
		cout << "Exited with attempts: " << c_attemps << endl;
		c_attemps += 1;
		if (c_attemps >= attempts) {
			res.rt = 0;
			return false;
		}
	}
  	res.rt = 1;
	return true;
}

bool Controller::attachObjectCallback(lds::request::Request &req, lds::request::Response &res) {
	if (req.request == 1) {
		attachCup();
	} else if (req.request == 0) {
		detachCup();
	}
	return 1;
}

bool Controller::createEnvironmentCallback(lds::request::Request &req, lds::request::Response &res) {
	if (req.request == 1) {
		addObject2();
	}
	return 1;
}