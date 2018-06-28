#include <iostream>
#include <fstream>
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
  endEffector_marker.scale.x = 0.020;
  endEffector_marker.scale.y = 0.020;
  endEffector_marker.scale.z = 0.020;

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

  int no_calculate_moveIt;
  int no_calculate_mark;
  std::vector<float> p;
  std::vector<float> init_values_moveIt;
  std::vector<float> init_values_mark;
  std::vector<float> cost_function;
  std::vector<float> cartGP_mark;
  std::vector<float> cartGP_moveIt;
  std::vector<geometry_msgs::Point> points_request;
  std_msgs::Float32MultiArray ra_gp_msgs;
  std_msgs::Float32MultiArray la_gp_msgs;

  std::ofstream cost_function_file;

  manip_msgs::InverseKinematicsFloatArray srv_ki_moveIt;
  manip_msgs::InverseKinematicsFloatArray srv_ki_mark;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  geometry_msgs::Pose endEffector_pose;
  geometry_msgs::Point aux_point;
  
  // ROS  Service Client
  cltIKinematicsLA = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_moveit/la_inverse_kinematics");
  cltIKinematicsRA = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_moveit/ra_inverse_kinematics");
  cltIKinematicsMark = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");

  // ROS Topic Publisher 
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  right_arm_goal_pose_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 10);
  left_arm_goal_pose_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 10);


  cost_function_file.open("/home/edgar/cost_function.txt");
  

  // Assing the goal pose
  // Resize msgs data
  ra_gp_msgs.data.resize(7);
  la_gp_msgs.data.resize(7);
  cost_function.resize(2);
  p.resize(6);

  p[0] = 0.5;
  p[1] = -0.325;
  p[2] = 0.85;

  p[3] = 0.0;       // Roll
  p[4] = -1.5707;   // Pitch
  p[5] = 0.0;       // Yaw

  aux_point.x = 0.5;
  aux_point.y = -0.325;
  aux_point.z = 0.85;
  points_request.push_back(aux_point);

  aux_point.x = 0.5;
  aux_point.y = -0.265;
  aux_point.z = 0.85;
  points_request.push_back(aux_point);

  aux_point.x = 0.5;
  aux_point.y = -0.225;
  aux_point.z = 0.85;
  points_request.push_back(aux_point);

  aux_point.x = 0.55;
  aux_point.y = -0.325;
  aux_point.z = 0.85;
  points_request.push_back(aux_point);

  aux_point.x = 0.55;
  aux_point.y = -0.265;
  aux_point.z = 0.85;
  points_request.push_back(aux_point);

  aux_point.x = 0.55;
  aux_point.y = -0.225;
  aux_point.z = 0.85;
  points_request.push_back(aux_point);

  aux_point.x = 0.55;
  aux_point.y = -0.325;
  aux_point.z = 0.88;
  points_request.push_back(aux_point);

  aux_point.x =  0.55;
  aux_point.y =  -0.325;
  aux_point.z = 0.95;
  points_request.push_back(aux_point);

  aux_point.x =  0.55;
  aux_point.y =  -0.225;
  aux_point.z = 0.95;
  points_request.push_back(aux_point);

  aux_point.x =  0.44;
  aux_point.y =  -0.05;
  aux_point.z = 0.79;
  points_request.push_back(aux_point);

  aux_point.x =  0.44;
  aux_point.y =  -0.05;
  aux_point.z =  0.81;
  points_request.push_back(aux_point);

  // Data request to Inverse Kinematic MOVE-IT
  cartGP_moveIt.push_back( p[0] );   // X-axis respect robot
  cartGP_moveIt.push_back( p[1] );  // Y-axis respect robot     
  cartGP_moveIt.push_back( p[2] );   // Z-axis respect robot
  
  cartGP_moveIt.push_back( p[3] );              // yaw
  cartGP_moveIt.push_back( p[4] );            // pitch
  cartGP_moveIt.push_back( p[5] );     // roll
  cartGP_moveIt.push_back(0.0);
  
  srv_ki_moveIt.request.cartesian_pose.data = cartGP_moveIt;
  init_values_moveIt = p;

  
  p[0] -= 0.032;
  transformPoint(p[0], p[1], p[2]);
  
   // Data request to Inverse Kinematic MARK
  cartGP_mark.push_back( p[0] );   // X-axis respect robot
  cartGP_mark.push_back( p[1] );  // Y-axis respect robot     
  cartGP_mark.push_back( p[2] );   // Z-axis respect robot
  
  cartGP_mark.push_back(p[3]);              // yaw
  cartGP_mark.push_back(p[4]+1.5707);            // pitch
  cartGP_mark.push_back(p[5]+1.5707);     // roll
  cartGP_mark.push_back(0.0);
  
  srv_ki_mark.request.cartesian_pose.data = cartGP_mark;
  init_values_mark = p;
  
  // Response data is already respect to base_link frame
  endEffector_pose.position.x = cartGP_moveIt[0];
  endEffector_pose.position.y = cartGP_moveIt[1];
  endEffector_pose.position.z = cartGP_moveIt[2];

  no_calculate_mark = 0;
  no_calculate_moveIt = 0;

  markerSetup();
  ros::Rate loop(10);


  
  while(ros::ok())
    {
      for(int i=0; i < points_request.size(); i++ )
      {
	  if(!ros::ok())
	    return 0;

	  std::cout << "point request[" << i << "]:  "
		    << points_request[i] << std::endl;

	  
	  cost_function[0] = 0.0;
	  cost_function[1] = 0.0;
	  endEffector_marker.points.push_back( endEffector_pose.position );
	  marker_pub.publish(endEffector_marker);
      
	  ros::spinOnce();
	  // ///////////////////////////////////////
	  // //  -----  MOVE-IT (service)  -----  //
	  // if(!cltIKinematicsRA.call(srv_ki_moveIt))
	  //   {
	  //     std::cout << std::endl <<
	  // 	"Move-It::: can't calling inverse kinematics service" << std::endl << std::endl;
	  //     no_calculate_moveIt++;
	  //   }
	  // else
	  //   {
	  //     std::cout << std::endl <<
	  // 	"Move-It::: Success service" << std::endl << std::endl;
	  //     for (int i=0; i < 7; i++)
	  // 	{
	  // 	  // std::cout << "   " << srv_ki.response.articular_pose.data[i] << std::endl;
	  // 	  ra_gp_msgs.data[i] = srv_ki_moveIt.response.articular_pose.data[i];
	  // 	  cost_function[0] += fabs(ra_gp_msgs.data[i]);
	  // 	} 
	  //     right_arm_goal_pose_pub.publish(ra_gp_msgs);
	  //   }
      
	  // // Sleep for visualization
	  // loop.sleep();
	  // boost::this_thread::sleep_for(boost::chrono::milliseconds(4000));
	  
      


	  /////////////////////////////////////////////////////
	  //  -----  GEOMETRIC-KINEMATICS (service)  -----  //
	  if(!cltIKinematicsMark.call(srv_ki_mark))
	    {
	      std::cout << std::endl <<
	  	"Sr. Mark::: can't calling inverse kinematics service" << std::endl << std::endl;
	      no_calculate_mark++;
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
      
	  // std::cout << std::endl << "Cost function:  " << std::endl
	  // 	    << "moveIt:  " << cost_function[0] << std::endl
	  // 	    << "mark:    " << cost_function[1] << std::endl;
      
	  // std::cout << "marker   ----  point" << std::endl
	  // 	    << endEffector_pose.position << std::endl << std::endl
	  // 	    << srv_ki_moveIt.request.cartesian_pose.data[0] << std::endl
	  // 	    << srv_ki_moveIt.request.cartesian_pose.data[1] << std::endl
	  // 	    << srv_ki_moveIt.request.cartesian_pose.data[2] << std::endl;
      
	  cost_function_file << cost_function[0] << " " << cost_function[1] <<  "\n";
		  
      
	  std::cout << "---------------------------" << std::endl;

      
	  // srv_ki_moveIt.request.cartesian_pose.data[0] = points_request[i].x;
	  // srv_ki_moveIt.request.cartesian_pose.data[1] = points_request[i].y;
	  // srv_ki_moveIt.request.cartesian_pose.data[2] = points_request[i].z;
	  p[0] = points_request[i].x - 0.032 ;
	  p[1] = points_request[i].y;
	  p[2] = points_request[i].z;
	  transformPoint(p[0], p[1], p[2]);
	  srv_ki_mark.request.cartesian_pose.data[0] = p[0];
	  srv_ki_mark.request.cartesian_pose.data[1] = p[1];
	  srv_ki_mark.request.cartesian_pose.data[2] = p[2];
	  
	  endEffector_pose.position.x = points_request[i].x;
	  endEffector_pose.position.y = points_request[i].y;
	  endEffector_pose.position.z = points_request[i].z;

	  // // Sleep for visualization
	  loop.sleep();
	  boost::this_thread::sleep_for(boost::chrono::milliseconds(4000));

	}
		  
      // loop.sleep();
      // boost::this_thread::sleep_for(boost::chrono::milliseconds(4000));
      
 
    }
  std::cout << "No calculate move It:  " << no_calculate_moveIt << std::endl;
  std::cout << "No calculate mark:  " << no_calculate_mark << std::endl;
  cost_function_file.close();
  return 0;
}
