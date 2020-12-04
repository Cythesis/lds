cmake_minimum_required(VERSION 3.0.2)
project(lds)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  geometry_msgs
  roscpp
  rospy
  actionlib
  actionlib_msgs
    ar_track_alvar_msgs
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
  message_generation
)


find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)

add_service_files(
  FILES
  move_joint.srv
  move_pose.srv
  pose.srv
  request.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    message_runtime
  DEPENDS
    EIGEN3
)


###########
## Build ##
###########
add_executable(main src/main.cpp)
add_executable(ar_follower src/ar_follower.cpp)
#add_executable(move_test src/move_client.cpp)

target_link_libraries(main ${catkin_LIBRARIES})
target_link_libraries(ar_follower ${catkin_LIBRARIES})
#target_link_libraries(move_test  ${catkin_LIBRARIES})

add_executable(move_joint_server src/move_joint_server.cpp)
target_link_libraries(move_joint_server ${catkin_LIBRARIES})
add_dependencies(move_joint_server lds_gencpp)

add_executable(move_pose_server src/move_pose_server.cpp)
target_link_libraries(move_pose_server ${catkin_LIBRARIES})
add_dependencies(move_pose_server lds_gencpp)

add_executable(move_cartesian_server src/moveCartesianServer.cpp)
target_link_libraries(move_cartesian_server ${catkin_LIBRARIES})
add_dependencies(move_cartesian_server lds_gencpp)

add_executable(attach_object_server src/attach_object_server.cpp)
target_link_libraries(attach_object_server ${catkin_LIBRARIES})
add_dependencies(attach_object_server lds_gencpp)

add_executable(create_environment_server src/create_environment_server.cpp)
target_link_libraries(create_environment_server ${catkin_LIBRARIES})
add_dependencies(create_environment_server lds_gencpp)


include_directories(include ${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})


