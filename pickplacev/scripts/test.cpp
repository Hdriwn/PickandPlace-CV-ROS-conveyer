#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP_ARM = "ur5_arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Adding conveyer belt as a collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();
    collision_object.id = "conveyer";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5;
    primitive.dimensions[1] = 2.0;
    primitive.dimensions[2] = 1.0;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.85;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.applyCollisionObjects(collision_objects);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // 1. Move to home position called "place"
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    move_group_interface_arm.move();
    ros::Duration(0.5).sleep();

    // Default picking position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("pick"));
    move_group_interface_arm.move();

    // Move the end effector closer
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = current_pose.pose.position.x;
    target_pose1.position.y = current_pose.pose.position.y;
    target_pose1.position.z = current_pose.pose.position.z - 0.124171; // Subtract the desired value

    move_group_interface_arm.setPoseTarget(target_pose1);
    move_group_interface_arm.move();

    ros::Duration(0.5).sleep();

    return 0;
}

