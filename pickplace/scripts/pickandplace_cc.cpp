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


/// CV Moments for image detection
// Function to turn on the vacuum gripper
void turnOnGripper(ros::NodeHandle &nh, const std::string &gripper_service)
{
   ros::ServiceClient client = nh.serviceClient<std_srvs::Empty > (gripper_service + "/on");
   std_srvs::Empty srv;
   client.call(srv);
}

// Function to turn off the vacuum gripper
void turnOffGripper(ros::NodeHandle &nh, const std::string &gripper_service)
{
   ros::ServiceClient client = nh.serviceClient<std_srvs::Empty > (gripper_service + "/off");
   std_srvs::Empty srv;
   client.call(srv);
}

bool colorDetected = false;
bool ongoingpickplace = false;
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
   if (!ongoingpickplace)
   {
      try
      {
         cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

//thresholds for different colors
         cv::Scalar blackLower(0, 0, 0);
         cv::Scalar blackUpper(60, 60, 60);

         cv::Scalar redLower(0, 0, 150);
         cv::Scalar redUpper(50, 50, 255);

         cv::Scalar greenLower(0, 150, 0);
         cv::Scalar greenUpper(50, 255, 50);

         cv::Scalar blueLower(150, 0, 0);
         cv::Scalar blueUpper(255, 50, 50);

         cv::Scalar yellowLower(0, 150, 150);
         cv::Scalar yellowUpper(50, 255, 255);

        	// Create binary masks for each color
         cv::Mat redMask, greenMask, blueMask, yellowMask, blackMask;
         cv::inRange(image, redLower, redUpper, redMask);
         cv::inRange(image, greenLower, greenUpper, greenMask);
         cv::inRange(image, blueLower, blueUpper, blueMask);
         cv::inRange(image, yellowLower, yellowUpper, yellowMask);
         cv::inRange(image, blackLower, blackUpper, blackMask);

        	//  the percentage of that colored pixels for each color in the image
         double totalPixels = image.rows *image.cols;
         double redPixels = cv::countNonZero(redMask);
         double greenPixels = cv::countNonZero(greenMask);
         double bluePixels = cv::countNonZero(blueMask);
         double yellowPixels = cv::countNonZero(yellowMask);
         double blackPixels = cv::countNonZero(blackMask);

         double redPercentage = (redPixels / totalPixels) *100.0;
         double greenPercentage = (greenPixels / totalPixels) *100.0;
         double bluePercentage = (bluePixels / totalPixels) *100.0;
         double yellowPercentage = (yellowPixels / totalPixels) *100.0;
         double blackPercentage = (blackPixels / totalPixels) *100.0;

        	// Set a threshold percentage to determine if a color is present
         double thresholdPercentage = 15;	// Adjust this threshold as needed

        	// Check if any of the colors exceed the threshold percentage
         if (redPercentage > thresholdPercentage)
         {
            ROS_INFO("Red color detected");
            std::cout << redPercentage << std::endl;
            colorDetected = true;

         }
         else if (greenPercentage > thresholdPercentage)
         {
            ROS_INFO("Green color detected");
            colorDetected = true;

         }
         else if (bluePercentage > thresholdPercentage)
         {
            ROS_INFO("Blue color detected");
            colorDetected = true;

         }
         else if (yellowPercentage > thresholdPercentage)
         {
            ROS_INFO("Yellow color detected");
            colorDetected = true;

         }
         else if (blackPercentage > thresholdPercentage)
         {
            //ROS_INFO("no color detected");
            colorDetected = false;

           // std::cout << blackPercentage << std::endl;

         }
         else
         {
            //ROS_INFO("No specific color detected");
            colorDetected = false;

         }

         cv::imshow("Image", image);
         cv::waitKey(1);
      }

      catch (cv_bridge::Exception &e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
      }
   }
}

///initialize main
int main(int argc, char **argv)
{
   ros::init(argc, argv, "move_group_interface_tutorial");
   ros::NodeHandle nh;

   ros::AsyncSpinner spinner(1);
   spinner.start();
   static const std::string PLANNING_GROUP_ARM = "ur5_arm";
   static const std::string PLANNING_GROUP_GRIPPER = "gripper";
    
   moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
   moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper; // for gripper

  	//adding conveyer belt as collision object
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
   std::vector<moveit_msgs::CollisionObject > collision_objects;
   collision_objects.push_back(collision_object);
   planning_scene_interface.applyCollisionObjects(collision_objects);

   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  	// 1. Move to home position called "place"
   move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
 move_group_interface_arm.move();
   ros::Duration(0.5).sleep();

	// default picking position
   move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("pick"));
 move_group_interface_arm.move();
  	//Turn off the gripper
//open gripper
move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
move_group_interface_gripper.plan(my_plan_gripper);

   ros::Duration(0.5).sleep();

  	//begin the spawning of blocks , uncomment for automated spawning
	//const char* command = "rosrun spawn_urdf_sdf spawn";
        //system(command);
// begin reading the camera images
   image_transport::ImageTransport it(nh);
   image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
   ros::Rate loop_rate(10);

   while (ros::ok())
   {
      if (colorDetected)
      {
	ROS_INFO("ENTERED WHILEOK");
         ongoingpickplace = true; // so that image callback will not be executed in between

	//move the endeffector closer since block is detected
         geometry_msgs::PoseStamped current_pose;
         current_pose = move_group_interface_arm.getCurrentPose("ee_link");

        // geometry_msgs::Pose target_pose1;
         //target_pose1.orientation = current_pose.pose.orientation;
         //target_pose1.position = current_pose.pose.position;
         //target_pose1.position.z -= 0.001; //x 0.070916
         //move_group_interface_arm.setPoseTarget(target_pose1);
         //move_group_interface_arm.move();

	//turn on the vaccum gripper
//open gripper
move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("close"));
move_group_interface_gripper.plan(my_plan_gripper);


         // reach the placing position
         move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("place"));
 move_group_interface_arm.move();        
 ros::Duration(0.1).sleep();

         //turn off the gripper
//open gripper
move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
move_group_interface_gripper.plan(my_plan_gripper);

         ros::Duration(1).sleep();

         colorDetected = false;	// Reset the flag for the next iteration
         // return to default picking position
         move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("pick"));
 move_group_interface_arm.move();
         ongoingpickplace = false;
         ros::Duration(3).sleep();
      }

      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}
