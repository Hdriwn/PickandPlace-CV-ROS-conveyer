#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <fstream>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/Int32.h>

std::string getFile(std::string filename)
{
  std::string buffer;
  char c;

  std::ifstream in(filename);
  if (!in)
  {
    std::cout << filename << " not found";
    exit(1);
  }
  while (in.get(c))
    buffer += c;
  in.close();

  return buffer;
}

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
            // Access the position of the model
            double modelZ = modelPoses[i].position.z;

            // Check the condition for despawning
            if (modelZ < 0.6)
            {
                // Delete the model
                gazebo_msgs::DeleteModel deleteModel;
                deleteModel.request.model_name = modelNames[i];

                // Create a service client for the delete model service
                ros::ServiceClient deleteModelClient = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

                // Call the delete model service
                if (deleteModelClient.call(deleteModel))
                {
                    if (deleteModel.response.success)
                    {
                       // ROS_INFO("Deleted model: %s", modelNames[i].c_str());
                    }
                    else
                    {
                        // Handle deletion failure
                    }
                }
                else
                {
                    // Handle service call failure
                }
            }
        }
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sdf_spawner");
  ros::NodeHandle nh;

  std::string path = ros::package::getPath("spawn_urdf_sdf");  // add roslib to cmakelists in find_package
ros::Publisher counterPub = nh.advertise<std_msgs::Int32>("model_counter", 10);

  std::vector<std::string> file_paths = {
    path + "/models/red_box.urdf",
    path + "/models/blue_box.urdf",
    path + "/models/yellow_box.urdf",
    path + "/models/green_box.urdf"
  };

  ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  ros::Subscriber modelStateSub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1000, boost::bind(modelStateCallback, _1, nh));
  
  // Create a service client for the delete model service
  ros::ServiceClient deleteModelClient = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

  gazebo_msgs::SpawnModel::Request spawn_model_req;
  gazebo_msgs::SpawnModel::Response spawn_model_resp;

  // Set initial block index
  int block_index = 0;
  int model_counter = 0;

  while (ros::ok())
  {
    std::string urdf = getFile(file_paths[block_index]);

    // Spawn the block
    spawn_model_req.initial_pose.position.x = 0.33;
    spawn_model_req.initial_pose.position.y = -0.7;
    spawn_model_req.initial_pose.position.z = 0.76;
    spawn_model_req.initial_pose.orientation.x = 0.0;
    spawn_model_req.initial_pose.orientation.y = 0;
    spawn_model_req.initial_pose.orientation.z = 0.0;
    spawn_model_req.initial_pose.orientation.w = 1.0;
    spawn_model_req.reference_frame = "world";
    std::string model_name = "marker_" + std::to_string(model_counter); 
    spawn_model_req.model_name = model_name;
    spawn_model_req.model_xml = urdf;

    bool call_service = spawnClient.call(spawn_model_req, spawn_model_resp);
    if (call_service)
    {
      if (spawn_model_resp.success)
      {
        ROS_INFO_STREAM("Model has been spawned ");

        // Increment block index
        block_index = (block_index + 1) % file_paths.size();
	//SENDING THE SPAWNED block number
	std_msgs::Int32 counterMsg;
	counterMsg.data = model_counter;

	// Publish the model_counter
	counterPub.publish(counterMsg);

        model_counter++;
      }
      else
      {
        // Handle spawn failure
      }
    }
    else
    {
      //ROS_ERROR("Failed to connect with gazebo server");
    }

    ros::Duration(4.0).sleep(); // Delay between spawns

    ros::spinOnce();
  }

  return 0;
}

