#include <iostream>
#include <fstream>
#include <ctime>
#include <boost/thread/thread.hpp>

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
  std::cout << "----> INITIALIZING INVERSE KINEMATIC TEST BY EDGAR-II..." << std::endl;
  ros::init(argc, argv, "IK_test_MoveIt");
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Publisher right_arm_goal_pose_pub;
  ros::Publisher left_arm_goal_pose_pub;
  ros::ServiceClient cltIKinematicsLA;
  ros::ServiceClient cltIKinematicsRA;

  bool srvSucces;
  
  float x, y , z;
  
  std::vector<float> cartesian;
  std_msgs::Float32MultiArray ra_gp_msgs;
  std_msgs::Float32MultiArray la_gp_msgs;

  manip_msgs::InverseKinematicsFloatArray srv_ki;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  geometry_msgs::Pose endEffector_pose;

  std::ofstream myFile, timeSucces, timeUnsucces;
  myFile.open("/home/edgar/ws_moveit_points.txt");
  timeSucces.open("/home/edgar/timeSucces_ik.txt");
  timeUnsucces.open("/home/edgar/timeUnsucces_ik.txt");

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


  x = 0.000;
  y = 0.075;
  z = 0.700;   

  // Data request to Inverse Kinematic
  cartesian.push_back( x );   // X-axis respect robot
  cartesian.push_back( y );   // Y-axis respect robot     
  cartesian.push_back( z );     // Z-axis respect robot
  
  cartesian.push_back(0.0);      // yaw
  cartesian.push_back(-1.57);      // pitch
  cartesian.push_back(0.0);      // roll
  cartesian.push_back(0.0);
  srv_ki.request.cartesian_pose.data = cartesian;

  // Response data is already respect to base_link frame
  endEffector_pose.position.x = cartesian[0];
  endEffector_pose.position.y = cartesian[1];
  endEffector_pose.position.z = cartesian[2];

  markerSetup();  

  ros::Rate loop(50);

  while(ros::ok())
    {

      if (z > 1.200)
	{
	  if(x > 0.45)
	    {
	      if(y > 0.45)
		{
		  myFile.close();
		  return 0;
		}
	      x = 0.0;
	      y += 0.01;
	    }
	  
	  z = 0.700;
	  x += 0.01;
	}
      
      cartesian[0] = x;
      cartesian[1] = y;
      cartesian[2] = z;
      srv_ki.request.cartesian_pose.data = cartesian;

      // Response data is already respect to base_link frame
      endEffector_pose.position.x = cartesian[0];
      endEffector_pose.position.y = cartesian[1];
      endEffector_pose.position.z = cartesian[2];

      endEffector_marker.pose.position = endEffector_pose.position;
      marker_pub.publish(endEffector_marker);
      ros::spinOnce();

      std::cout << "Request: [x, y, z]: " << std::endl;
      std::cout << x << "  " << y << "  " << z << std::endl;

       clock_t begin = std::clock();
       //srvSucces = cltIKinematicsLA.call(srv_ki);
       boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
       clock_t end = std::clock();
       double duration = double(end-begin) / CLOCKS_PER_SEC;
      
      
      if(!srvSucces)
        {
      	  std::cout << std::endl <<
      	    "Justina::Manip can't calling inverse kinematics service" << std::endl << std::endl;
	  timeUnsucces << " " << duration << "\n";
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
	  
	  timeSucces << " " << duration << "\n";
      	  myFile << cartesian[0] << " " << cartesian[1] << " " << cartesian[2] << "\n";
      	  left_arm_goal_pose_pub.publish(ra_gp_msgs);
      	}
      
      
      std::cout << "---------------------------" << std::endl;
      
      //boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
      z += 0.01;
      loop.sleep();
    }
  
  myFile.close();
  timeSucces.close();
  timeUnsucces.close();

  return 0;
}
