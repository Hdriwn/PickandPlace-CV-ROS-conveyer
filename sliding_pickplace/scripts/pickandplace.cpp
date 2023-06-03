#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>

// Function to turn on the vacuum gripper
void turnOnGripper(ros::NodeHandle& nh, const std::string& gripper_service)
{
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>(gripper_service + "/on");
  std_srvs::Empty srv;
  client.call(srv);
}

// Function to turn off the vacuum gripper
void turnOffGripper(ros::NodeHandle& nh, const std::string& gripper_service)
{
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>(gripper_service + "/off");
  std_srvs::Empty srv;
  client.call(srv);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


//adding collision object conveyer
moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();

    collision_object.id = "conveyer";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5;
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 1;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.75;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.applyCollisionObjects(collision_objects);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 // go the the default config
move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("pick"));

 move_group_interface_arm.move();

  ros::Duration(0.1).sleep();


ROS_INFO_NAMED("tutorial", "place position reached");

// Wait for 10 seconds
ros::Duration(0.0).sleep();


// 2. Move to home position
ROS_INFO_NAMED("tutorial", "attempting pick");
move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("pick"));
 move_group_interface_arm.move();


ROS_INFO_NAMED("tutorial", "pick position reached");

//    move_group_interface_arm.setPoseTarget(target_pose1);

    // 3. Open the gripper

ROS_INFO_NAMED("tutorial","gripperoff");
  
  turnOffGripper(nh, "/ur5/vacuum_gripper");
turnOffGripper(nh, "/ur5/vacuum_gripper1");
turnOffGripper(nh, "/ur5/vacuum_gripper2");
turnOffGripper(nh, "/ur5/vacuum_gripper3");
turnOffGripper(nh, "/ur5/vacuum_gripper4");
turnOffGripper(nh, "/ur5/vacuum_gripper5");
turnOffGripper(nh, "/ur5/vacuum_gripper6");
turnOffGripper(nh, "/ur5/vacuum_gripper7");
turnOffGripper(nh, "/ur5/vacuum_gripper8");

ros::Duration(0.0).sleep(); 

    // 5. Close the  gripper
ROS_INFO_NAMED("tutorial","vacuume onbegin");
  turnOnGripper(nh, "/ur5/vacuum_gripper");
turnOnGripper(nh, "/ur5/vacuum_gripper1");
turnOnGripper(nh, "/ur5/vacuum_gripper2");
turnOnGripper(nh, "/ur5/vacuum_gripper3");
turnOnGripper(nh, "/ur5/vacuum_gripper4");
turnOnGripper(nh, "/ur5/vacuum_gripper5");
turnOnGripper(nh, "/ur5/vacuum_gripper6");
turnOnGripper(nh, "/ur5/vacuum_gripper7");
turnOnGripper(nh, "/ur5/vacuum_gripper8");
ROS_INFO_NAMED("tutorial","vacuume onend");
ros::Duration(0.1).sleep();

 
//move cloose
ROS_INFO_NAMED("tutorial","mved closer");
  
// 2. Move to home position
ROS_INFO_NAMED("tutorial", "attempting plaaace");
move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("place"));
move_group_interface_arm.move();
  // Wait for a small duration before checking the state again
  ros::Duration(0).sleep();

ROS_INFO_NAMED("tutorial", "plaace position reached");


ROS_INFO_NAMED("tutorial","gripper open");
    // 8. Open the gripper
      turnOffGripper(nh, "/ur5/vacuum_gripper");
turnOffGripper(nh, "/ur5/vacuum_gripper1");
turnOffGripper(nh, "/ur5/vacuum_gripper2");
turnOffGripper(nh, "/ur5/vacuum_gripper3");
turnOffGripper(nh, "/ur5/vacuum_gripper4");
turnOffGripper(nh, "/ur5/vacuum_gripper5");
turnOffGripper(nh, "/ur5/vacuum_gripper6");
turnOffGripper(nh, "/ur5/vacuum_gripper7");
turnOffGripper(nh, "/ur5/vacuum_gripper8");

ros::Duration(0.0).sleep(); 
 // go the the default config
move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("pick"));

 move_group_interface_arm.move();

  ros::Duration(0.1).sleep();

  ros::shutdown();
  return 0;
}
