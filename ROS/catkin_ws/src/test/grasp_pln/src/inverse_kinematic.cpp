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

visualization_msgs::Marker vertix_marker;

void markerSetup()
{
    vertix_marker.header.frame_id = "base_link";    
    vertix_marker.header.stamp = ros::Time::now();
    vertix_marker.ns = "la_goal_pose";
    vertix_marker.pose.orientation.w = 1.0;
    vertix_marker.id = 0;
    vertix_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    vertix_marker.scale.x = 0.040;
    vertix_marker.scale.y = 0.040;
    vertix_marker.color.g = 1.0f;
    vertix_marker.color.b = 0.5f;
    vertix_marker.color.a = 1.0;
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR WORKSPACE RIGHT ARM BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "inverse_kinematic_test");
    ros::NodeHandle n;
    ros::Rate loop(10);

    ros::ServiceClient cltDetectObjectsPCA;
    ros::ServiceClient cltInverseKinematics;
    ros::ServiceClient cltDirectKinematics;

    ros::Publisher marker_pub;

    tf::Quaternion q;
    tf::Transform R5_EE;
    tf::Vector3 wristPosition(0,0,0);
    
    std::vector<float> cartesian;
    std::vector<float> articular;
    
    manip_msgs::InverseKinematicsFloatArray srv_ki;
    
    visualization_msgs::Marker vertix;
    geometry_msgs::Point p1, p2;
      

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    cltInverseKinematics = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");
    
    // Data request in format [x, y, z, roll, pitch, yaw, elbow]
    cartesian.push_back( 0.35);
    cartesian.push_back(-0.15);
    cartesian.push_back( 0.85);
    // Angles rotation
    cartesian.push_back(0.0);
    cartesian.push_back(0.0);
    cartesian.push_back(0.0);
    // Elbow angle
    cartesian.push_back(0.2);
    srv_ki.request.cartesian_pose.data  = cartesian;

      
    q.setRPY(cartesian[3], cartesian[4], cartesian[5]);
    R5_EE.setIdentity();
    R5_EE.setRotation(q);
    
    wristPosition[0] = 0.165;
    wristPosition = R5_EE * wristPosition; //XYZ position of the end effector


    wristPosition[0] = cartesian[0] - wristPosition[0];
    wristPosition[1] = cartesian[1] - wristPosition[1];
    wristPosition[2] = cartesian[2] - wristPosition[2];

    
    markerSetup();

    //Display left_arm goal_pose
    p1.x = cartesian[0];
    p1.y = cartesian[1];
    p1.z = cartesian[2];
    vertix_marker.points.push_back(p1);

    p2.x = wristPosition[0];
    p2.y = wristPosition[1];
    p2.z = wristPosition[2];
    vertix_marker.points.push_back(p2);
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
	      std::cout << articular[i] << std::endl;
	  }

	marker_pub.publish(vertix_marker);
	
	ros::spinOnce();
	loop.sleep();
      }
    
    return 0;
}
