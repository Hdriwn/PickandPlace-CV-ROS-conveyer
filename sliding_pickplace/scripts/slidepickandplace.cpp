#include <ros/ros.h>
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
void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    std::vector<std::string> modelNames = msg->name;
    std::vector<geometry_msgs::Pose> modelPoses = msg->pose;

    // Find the index of the target block
    int targetIndex = -1;
    for (size_t i = 0; i < modelNames.size(); ++i)
    {
        if (modelNames[i] == targetBlockName)
        {
            targetIndex = i;
            break;
        }
    }

    if (targetIndex != -1)
    {
        // Access the position of the target block
        yCoordinate = modelPoses[targetIndex].position.y;// using for the prismatic joint
	xCoordinate = modelPoses[targetIndex].position.x;
	zCoordinate = modelPoses[targetIndex].position.z;	

        blockPositionUpdated = true; 


    }
    else
    {
        ROS_WARN("Block %s not found in the model states", targetBlockName.c_str());
    }
}

bool userRequestCallback(sliding_pickplace::UserRequest::Request& req,
                         sliding_pickplace::UserRequest::Response& res)
{
    // Update the userrequest variable with the requested value
    userrequest = req.requested_value;
    targetBlockName = "marker_" + std::to_string(userrequest);

    // wait till values get updated:
while (!blockPositionUpdated)
    {
        ros::Duration(0.1).sleep();
    }

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
    ros::Subscriber modelStateSub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, modelStateCallback);

    // Create a service server for user requests
    ros::ServiceServer userRequestServer = nh.advertiseService("user_request", userRequestCallback);
 	//begin the spawning of blocks
	const char* command1 = "rosrun spawn_urdf_sdf spawn";
    system(command1);
   ros::Rate loop_rate(10);
   while(ros::ok()){
   // Set the prismatic joint's target position
    double desired_position = yCoordinate; 
    move_group.setJointValueTarget("world_joint", desired_position);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
	// launches the pickup program to pick up the target
    const char* command2 = "rosrun slider_pickplace pickup";
    system(command2);
}
ros::spinOnce();
      loop_rate.sleep();
}


    return 0;
}

