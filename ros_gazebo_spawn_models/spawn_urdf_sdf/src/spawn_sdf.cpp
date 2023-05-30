#include <gazebo_msgs/SpawnModel.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <fstream>
#include <gazebo_msgs/ApplyBodyWrench.h>

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sdf_spawner");
  ros::NodeHandle nh;

  std::string path = ros::package::getPath("spawn_urdf_sdf");  // add roslib to cmakelists in find_package
  std::string file_path = path + "/models/green_box.urdf";
  // std::cout << file_path;

  std::string urdf = getFile(file_path);
  //   std::cout << urdf;

  ros::ServiceClient spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  gazebo_msgs::SpawnModel::Request spawn_model_req;
  gazebo_msgs::SpawnModel::Response spawn_model_resp;

  spawn_model_req.initial_pose.position.x = 0.413314;
  spawn_model_req.initial_pose.position.y = -0.7;
  spawn_model_req.initial_pose.position.z = 0.82;
  spawn_model_req.initial_pose.orientation.x = 0.0;
  spawn_model_req.initial_pose.orientation.y = 1.570795;
  spawn_model_req.initial_pose.orientation.z = 0.0;
  spawn_model_req.initial_pose.orientation.w = 1.0;
  spawn_model_req.reference_frame = "world";
  spawn_model_req.model_name = "marker";
  spawn_model_req.model_xml = urdf;


  bool call_service = spawnClient.call(spawn_model_req, spawn_model_resp);
  if (call_service)
  {
    if (spawn_model_resp.success)
    {
      ROS_INFO_STREAM("model has been spawned");

      // Apply the wrench
      ros::ServiceClient applyWrenchClient = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
      gazebo_msgs::ApplyBodyWrench::Request apply_wrench_req;
      gazebo_msgs::ApplyBodyWrench::Response apply_wrench_resp;

      apply_wrench_req.body_name = "marker::base_link";  // Replace 'link_name' with the actual name of the link you want to apply the wrench to
      apply_wrench_req.reference_frame = "world";
      apply_wrench_req.reference_point.x = 0.0;
      apply_wrench_req.reference_point.y = 0.0;
      apply_wrench_req.reference_point.z = 0.0;
      apply_wrench_req.wrench.force.x = 0.0;
      apply_wrench_req.wrench.force.y = 0.0;
      apply_wrench_req.wrench.force.z = 0.0;  // Vertical wrench of 50N
      apply_wrench_req.start_time = ros::Time::now();
      apply_wrench_req.duration = ros::Duration(-1);  // Apply the wrench indefinitely

      bool wrench_call_service = applyWrenchClient.call(apply_wrench_req, apply_wrench_resp);
      if (wrench_call_service)
      {
        if (apply_wrench_resp.success)
        {
          ROS_INFO_STREAM("wrench applied successfully");
        }
        else
        {
          ROS_INFO_STREAM("failed to apply wrench");
        }
      }
      else
      {
        ROS_ERROR("failed to connect with gazebo server for applying wrench");
      }
    }
    else
    {
      ROS_INFO_STREAM("model spawn failed");
    }
  }
  else
  {
    ROS_ERROR("failed to connect with gazebo server");
  }

  return 0;
}

