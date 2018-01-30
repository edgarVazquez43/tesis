#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float32MultiArray.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/DirectKinematicsFloatArray.h"

visualization_msgs::Marker vertix_marker, wristCenter;

void markerSetup()
{
  vertix_marker.header.frame_id = "base_ra_arm";    
  vertix_marker.header.stamp = ros::Time::now();
  vertix_marker.ns = "la_goal_pose";
  vertix_marker.pose.orientation.w = 1.0;
  vertix_marker.id = 0;
  vertix_marker.type = visualization_msgs::Marker::SPHERE_LIST;

  wristCenter.header.frame_id = "base_ra_arm";    
  wristCenter.header.stamp = ros::Time::now();
  wristCenter.ns = "wristCenter";
  wristCenter.pose.orientation.w = 1.0;
  wristCenter.id = 0;
  wristCenter.type = visualization_msgs::Marker::SPHERE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  vertix_marker.scale.x = 0.040;
  vertix_marker.scale.y = 0.040;
  vertix_marker.color.g = 1.0f;
  vertix_marker.color.b = 0.2f;
  vertix_marker.color.a = 1.0;

  // POINTS markers use x and y scale for width/height respectively
  wristCenter.scale.x = 0.040;
  wristCenter.scale.y = 0.040;
  wristCenter.color.r = 1.0f;
  wristCenter.color.b = 0.15f;
  wristCenter.color.a = 1.0;
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
    std::cout << "INITIALIZING A TEST FOR IK RIGHT ARM BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "inverse_kinematic_test");
    ros::NodeHandle n;
    ros::Rate loop(10);
    ros::ServiceClient cltInverseKinematics;
    ros::Publisher marker_pub;
    ros::Publisher raGoalPose_pub;

    tf::Quaternion q;
    tf::Transform R5_EE;
    tf::Vector3 wristPosition(0,0,0);
    
    std::vector<float> cartesian;
    std::vector<float> articular;
    
    manip_msgs::InverseKinematicsFloatArray srv_ki;
    std_msgs::Float32MultiArray raGP_msg;
    geometry_msgs::Point p1, p2;

    float wc_x, wc_y, wc_z;

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    raGoalPose_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 10);
    
    cltInverseKinematics = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");
    
    // Data request in format [x, y, z, roll, pitch, yaw, elbow]
    cartesian.push_back( 0.25);
    cartesian.push_back(-0.26);
    cartesian.push_back( 0.95);
    // Angles rotation
    cartesian.push_back(0.0);
    cartesian.push_back(1.5707);
    cartesian.push_back(1.5707);
    // Elbow angle
    cartesian.push_back(0.0);
      
    q.setRPY(cartesian[3], cartesian[4]-1.5707, cartesian[5]);
    R5_EE.setIdentity();
    R5_EE.setRotation(q);
    
    wristPosition[0] = 0.165;
    wristPosition = R5_EE * wristPosition; //XYZ position of the end effector


    wristPosition[0] = cartesian[0] - wristPosition[0];
    wristPosition[1] = cartesian[1] - wristPosition[1];
    wristPosition[2] = cartesian[2] - wristPosition[2];

    markerSetup();

    //Transform from base_link to base_la_arm
    transformPoint(cartesian[0], cartesian[1], cartesian[2]);
    srv_ki.request.cartesian_pose.data  = cartesian;

    //Display left_arm goal_pose 
    p1.x = cartesian[0];
    p1.y = cartesian[1];
    p1.z = cartesian[2];
    vertix_marker.points.push_back(p1);

    wc_x = wristPosition[0];
    wc_y = wristPosition[1];
    wc_z = wristPosition[2];
    
    transformPoint(wc_x, wc_y, wc_z);
    std::cout << "After transformation..." << std::endl;
    std::cout << "x_wc: " << wc_x << std::endl;
    std::cout << "y_wc: " << wc_y << std::endl;
    std::cout << "z_wc: " << wc_z << std::endl;
    
    p2.x = wc_x;
    p2.y = wc_y;
    p2.z = wc_z;
    wristCenter.points.push_back(p2);
    
    for(int i = 0; i < 7; i++ ) raGP_msg.data.push_back(0);

    while(ros::ok())
      {
	if(!cltInverseKinematics.call(srv_ki))
	  {
	    std::cout << std::endl << "Justina::Manip can't calling inverse kinematics service"
		      << std::endl << std::endl;
	    return false;
	  }
	else
	  {
	    articular = srv_ki.response.articular_pose.data;
	    std::cout << "Service response:" << std::endl;
	    for(int i = 0; i < articular.size(); i++ )
	      {
		std::cout << articular[i] << std::endl;
		raGP_msg.data[i] = articular[i];
	      }
	  }
	
	raGoalPose_pub.publish(raGP_msg);
	marker_pub.publish(vertix_marker);
	marker_pub.publish(wristCenter);
	loop.sleep();
      }
    
    ros::spinOnce();
    loop.sleep();
    
    return 0;
}
