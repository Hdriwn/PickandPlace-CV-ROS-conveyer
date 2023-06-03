#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <sliding_pickplace/UserRequest.h>
#include <geometry_msgs/Point.h>

int userrequest = 0;
std::string targetBlockName;
double xCoordinate = 0;
double yCoordinate = 0;
double zCoordinate = 0;
bool blockPositionUpdated = false; 
int targetIndex=0;
void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg, ros::NodeHandle& nh)
{
    std::vector<std::string> modelNames = msg->name;
    std::vector<geometry_msgs::Pose> modelPoses = msg->pose;

    // Iterate over the model states
    for (size_t i = 0; i < modelNames.size(); i++)
    {
        // Check if the model starts with "marker"
        if (modelNames[i].find("marker") == 0)
        {
            // Check if the current model name matches the target block name
            if (modelNames[i] == targetBlockName)
            {
                ROS_INFO("Found block: %s", modelNames[i].c_str());
                ROS_INFO("Block pose: x=%f, y=%f, z=%f", modelPoses[i].position.x, modelPoses[i].position.y, modelPoses[i].position.z);
                yCoordinate = modelPoses[i].position.y;
                xCoordinate = modelPoses[i].position.x;
                zCoordinate = modelPoses[i].position.z;
                
                break;
            }
blockPositionUpdated = true;
        }
    }
}

bool userRequestCallback(sliding_pickplace::UserRequest::Request& req,
                         sliding_pickplace::UserRequest::Response& res)
{
    ROS_INFO("ENTERED USECALLBACK");

    // Update the userrequest variable with the requested value
    userrequest = req.requested_value;
    ROS_INFO("User request: %d", userrequest);

    targetBlockName = "marker_" + std::to_string(userrequest);
    ROS_INFO("Target Block Name: %s", targetBlockName.c_str());

    // Wait until values get updated
    while (!blockPositionUpdated)
    {
        ROS_INFO("Waiting for block position update...");
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Block position updated!");

    geometry_msgs::Point position;
    position.x = xCoordinate;
    position.y = yCoordinate;
    position.z = zCoordinate;

    res.success = true;
    res.position = position;

    return true;
}


int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "prismatic_joint_controller");
    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroupInterface move_group("moving_base");
    static const std::string PLANNING_GROUP_ARM = "ur5_arm";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));

 move_group_interface_arm.move();

  ros::Duration(0.1).sleep();

    ROS_INFO("before MODELSTATE");
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/moving_base_controller/command", 1);
    ros::Subscriber modelStateSub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(modelStateCallback, _1, nh));
    ROS_INFO("after MODELSTATE");

    // Create a service server for user requests
   ros::ServiceServer userRequestServer = nh.advertiseService("user_request", &userRequestCallback);
    ROS_INFO("after userrequest");

    // Add the JointTrajectoryPoint to the trajectory
    const char* command2 = "rosrun sliding_pickplace pickup";
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // Create a JointTrajectory message
        trajectory_msgs::JointTrajectory traj;
        traj.joint_names.push_back("world_joint");
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(yCoordinate);
        point.time_from_start = ros::Duration(1.0);
        // Add the JointTrajectoryPoint to the trajectory
        traj.points.push_back(point);

        pub.publish(traj);
        ROS_INFO("Published prismatic joint trajectory.");

    system(command2);

        ros::spinOnce();
        loop_rate.sleep();
    }

// ros::spin(); 


    return 0;
}

