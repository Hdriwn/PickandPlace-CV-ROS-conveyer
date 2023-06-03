#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "prismatic_joint_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/moving_base_controller/command", 1);

    // Create a JointTrajectory message
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back("world_joint");
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(3); 
    point.time_from_start = ros::Duration(1.0);  
    // Add the JointTrajectoryPoint to the trajectory
    traj.points.push_back(point);
while(ros::ok()){
    pub.publish(traj);

    ROS_INFO("Published prismatic joint trajectory.");
}
ros::spinOnce();
    return 0;
}

