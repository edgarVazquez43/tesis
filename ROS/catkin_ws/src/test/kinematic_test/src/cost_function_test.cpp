#include <iostream>
#include "ros/ros.h"
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp> 
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include "manip_msgs/InverseKinematicsFloatArray.h"

visualization_msgs::Marker endEffector_marker;

bool markerSetup()
{
  endEffector_marker.header.frame_id = "base_link";    
  endEffector_marker.header.stamp = ros::Time::now();
  endEffector_marker.ns = "endEffector_r";
    
  endEffector_marker.pose.orientation.w = 1.0;
  endEffector_marker.id = 0;
  endEffector_marker.type = visualization_msgs::Marker::POINTS;
    
  // POINTS markers use x and y scale for width/height respectively
  endEffector_marker.scale.x = 0.040;
  endEffector_marker.scale.y = 0.040;
  endEffector_marker.scale.z = 0.040;

  endEffector_marker.color.g = 1.0f;
  endEffector_marker.color.a = 1.0;

  return true;
}


bool transformPoint(float &x, float &y, float &z)
{
  bool succes = true;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::Vector3 v(x, y, z);

  while(ros::ok())
    {
      succes = true;
      try
	{ 
	  listener.lookupTransform("/base_ra_arm", "/base_link", ros::Time(0), transform);
	}
      catch (tf::TransformException &ex)
	{
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	  succes = false;
	}

      std::cout << succes << std::endl;
      if(succes)
	break;
    }

  if(succes)
    {  
      v = transform * v;
      x = v.x();
      y = v.y();
      z = v.z();
    }
  
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
  ros::ServiceClient cltIKinematicsMark;

  std::vector<float> p;
  std::vector<float> cost_function;
  std::vector<float> cartGP_mark;
  std::vector<float> cartGP_moveIt;
  std_msgs::Float32MultiArray ra_gp_msgs;
  std_msgs::Float32MultiArray la_gp_msgs;



  manip_msgs::InverseKinematicsFloatArray srv_ki_moveIt;
  manip_msgs::InverseKinematicsFloatArray srv_ki_mark;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  geometry_msgs::Pose endEffector_pose;
  
  // ROS  Service Client
  cltIKinematicsLA = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_moveit/la_inverse_kinematics");
  cltIKinematicsRA = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_moveit/ra_inverse_kinematics");
  cltIKinematicsMark = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");

  // ROS Topic Publisher 
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  right_arm_goal_pose_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 10);
  left_arm_goal_pose_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 10);


  // Assing the goal pose
  // Resize msgs data
  ra_gp_msgs.data.resize(7);
  la_gp_msgs.data.resize(7);
  cost_function.resize(2);
  p.resize(3);

  p[0] = 0.15;
  p[1] = -0.225;
  p[2] = 0.90;

  // Data request to Inverse Kinematic MOVE-IT
  cartGP_moveIt.push_back( p[0] );   // X-axis respect robot
  cartGP_moveIt.push_back( p[1] );  // Y-axis respect robot     
  cartGP_moveIt.push_back( p[2] );   // Z-axis respect robot
  
  cartGP_moveIt.push_back(0.0);              // yaw
  cartGP_moveIt.push_back(-1.57);            // pitch
  cartGP_moveIt.push_back(0.0);     // roll
  cartGP_moveIt.push_back(0.0);
  
  srv_ki_moveIt.request.cartesian_pose.data = cartGP_moveIt;

  p[0] -= 0.05;
  transformPoint(p[0], p[1], p[2]);
  
   // Data request to Inverse Kinematic MARK
  cartGP_mark.push_back( p[0] );   // X-axis respect robot
  cartGP_mark.push_back( p[1] );  // Y-axis respect robot     
  cartGP_mark.push_back( p[2] );   // Z-axis respect robot
  
  cartGP_mark.push_back(0.0);              // yaw
  cartGP_mark.push_back(0.0);            // pitch
  cartGP_mark.push_back(1.5707);     // roll
  cartGP_mark.push_back(0.0);
  
  srv_ki_mark.request.cartesian_pose.data = cartGP_mark;

  
  // Response data is already respect to base_link frame
  endEffector_pose.position.x = cartGP_moveIt[0];
  endEffector_pose.position.y = cartGP_moveIt[1];
  endEffector_pose.position.z = cartGP_moveIt[2];


  markerSetup();
  ros::Rate loop(10);


  
  while(ros::ok())
    {
      cost_function[0] = 0.0;
      cost_function[1] = 0.0;
      endEffector_marker.points( endEffector_pose.position );
      marker_pub.publish(endEffector_marker);
      
      ros::spinOnce();
      
      if(!cltIKinematicsRA.call(srv_ki_moveIt))
        {
	  std::cout << std::endl <<
	    "Move-It::: can't calling inverse kinematics service" << std::endl << std::endl;
        }
      else
        {
	  std::cout << std::endl <<
	    "Move-It::: Success service" << std::endl << std::endl;
	  for (int i=0; i < 7; i++)
	    {
	      // std::cout << "   " << srv_ki.response.articular_pose.data[i] << std::endl;
	      ra_gp_msgs.data[i] = srv_ki_moveIt.response.articular_pose.data[i];
	      cost_function[0] += fabs(ra_gp_msgs.data[i]);
	    } 
	  right_arm_goal_pose_pub.publish(ra_gp_msgs);
	}

      loop.sleep();
      boost::this_thread::sleep_for(boost::chrono::milliseconds(4000));
      
      if(!cltIKinematicsMark.call(srv_ki_mark))
        {
	  std::cout << std::endl <<
	    "Sr. Mark::: can't calling inverse kinematics service" << std::endl << std::endl;
        }
      else
        {
	    std::cout << std::endl <<
	    "Sr. Mark::: Success service" << std::endl << std::endl;
	  for (int i=0; i < 7; i++)
	    {
	      // std::cout << "   " << srv_ki_mark.response.articular_pose.data[i] << std::endl;
	      ra_gp_msgs.data[i] = srv_ki_mark.response.articular_pose.data[i];
	      cost_function[1] += fabs(ra_gp_msgs.data[i]);
	    }

	  right_arm_goal_pose_pub.publish(ra_gp_msgs);
	}

      std::cout << std::endl << "Cost function:  "
		<< "moveIt:  " << cost_function[0] << std::endl
		<< "mark:    " << cost_function[1] << std::endl;

      
      std::cout << "---------------------------" << std::endl;

      
      srv_ki_moveIt.request.cartesian_pose.data[0] += 0.04;
      srv_ki_mark.request.cartesian_pose.data[1] += 0.04;
      endEffector_pose.position.x += 0.04;
      
      loop.sleep();
      boost::this_thread::sleep_for(boost::chrono::milliseconds(4000));
    }
  return 0;
}
