#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "manip_msgs/InverseKinematicsFloatArray.h"

visualization_msgs::Marker endEffector_marker;

bool markerSetup()
{
  endEffector_marker.header.frame_id = "base_link";    
  endEffector_marker.header.stamp = ros::Time::now();
  endEffector_marker.ns = "endEffector_r";
    
  endEffector_marker.pose.orientation.w = 1.0;
    
  endEffector_marker.id = 0;
    
  endEffector_marker.type = visualization_msgs::Marker::SPHERE;
    
  // POINTS markers use x and y scale for width/height respectively
  endEffector_marker.scale.x = 0.040;
  endEffector_marker.scale.y = 0.040;
  endEffector_marker.scale.z = 0.040;

  endEffector_marker.color.g = 1.0f;
  endEffector_marker.color.a = 1.0;

  return true;
}


int main(int argc, char** argv)
{
  std::cout << std::endl;
  std::cout << "----> INITIALIZING A DIRECT KINEMATIC TEST BY EDGAR-II..." << std::endl;
  ros::init(argc, argv, "DK_test_MoveIt");
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Publisher right_arm_goal_pose_pub;
  ros::Publisher left_arm_goal_pose_pub;
  ros::ServiceClient cltIKinematicsLA;
  ros::ServiceClient cltIKinematicsRA;

  std::vector<float> cartesian;
  std_msgs::Float32MultiArray ra_gp_msgs;
  std_msgs::Float32MultiArray la_gp_msgs;

  manip_msgs::InverseKinematicsFloatArray srv_ki;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  geometry_msgs::Pose endEffector_pose;

  // ROS  Service Client
  cltIKinematicsLA = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_moveit/la_inverse_kinematics");
  cltIKinematicsRA = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_moveit/ra_inverse_kinematics");

  // ROS Topic Publisher 
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  right_arm_goal_pose_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 10);
  left_arm_goal_pose_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 10);


  // Resize msgs data
  ra_gp_msgs.data.resize(7);
  la_gp_msgs.data.resize(7);


  // Data request to Inverse Kinematic
  cartesian.push_back( 0.15);   // X-axis respect robot
  cartesian.push_back(-0.225);   // Y-axis respect robot     
  cartesian.push_back(0.90);     // Z-axis respect robot
  
  cartesian.push_back(0.5);      // yaw
  cartesian.push_back(-1.57);      // pitch
  cartesian.push_back(0.0);      // roll
  cartesian.push_back(0.0);
  srv_ki.request.cartesian_pose.data = cartesian;

  // Response data is already respect to base_link frame
  endEffector_pose.position.x = cartesian[0];
  endEffector_pose.position.y = cartesian[1];
  endEffector_pose.position.z = cartesian[2];

  markerSetup();

  
  

  ros::Rate loop(10);

  while(ros::ok())
    {
      endEffector_marker.pose.position = endEffector_pose.position;
      marker_pub.publish(endEffector_marker);
      ros::spinOnce();
      
      if(!cltIKinematicsRA.call(srv_ki))
        {
	  std::cout << std::endl <<
	    "Justina::Manip can't calling inverse kinematics service" << std::endl << std::endl;
        }
      else
        {
	  std::cout << "InverseKinematics.-> Calculated cartesian...." << std::endl;
	  std::cout << "[x, y, z, roll, pitch, yaw]" << std::endl;
	  for (int i=0; i < 7; i++)
	    {
	      std::cout << "   " << srv_ki.response.articular_pose.data[i] << std::endl;
	      ra_gp_msgs.data[i] = srv_ki.response.articular_pose.data[i];
	    }

	  right_arm_goal_pose_pub.publish(ra_gp_msgs);
	}

      
      std::cout << "---------------------------" << std::endl;

      
      loop.sleep();
    }
  return 0;
}
