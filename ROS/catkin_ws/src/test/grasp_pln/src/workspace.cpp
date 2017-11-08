#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float32MultiArray.h"
#include "manip_msgs/DirectKinematics.h"

std_msgs::Float32MultiArray currentPos;

visualization_msgs::Marker endEffector_marker;

void markerSetup()
{
    endEffector_marker.header.frame_id = "base_link";
    
    endEffector_marker.header.stamp = ros::Time::now();

    endEffector_marker.ns = "endEffector_r";
    
    endEffector_marker.pose.orientation.w = 1.0;

    endEffector_marker.id = 0;

    endEffector_marker.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    endEffector_marker.scale.x = 0.010;
    endEffector_marker.scale.y = 0.010;

    endEffector_marker.color.g = 1.0f;
    endEffector_marker.color.a = 1.0;
}


void currentPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  currentPos.data = msg->data;
  std::cout << "Current pose: " << std::endl;
  for(int i=0; i < currentPos.data.size(); i++) std::cout << currentPos.data[i] << " ";
  std::cout << "" << std::endl;
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR WORKSPACE RIGHT ARM BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "grasp_pln");
    ros::NodeHandle n;

    std::vector<float> cartesian;
    std::vector<float> articular;

    geometry_msgs::Pose endEffector_pose;
    manip_msgs::DirectKinematics srv_kd;
    
    ros::ServiceClient cltDirectKinematics;
    ros::Publisher marker_pub;
    ros::Subscriber currentPos_sub;
    tf::TransformListener listener;
    tf::StampedTransform transform;


    cltDirectKinematics = n.serviceClient<manip_msgs::DirectKinematics>("/manipulation/ik_geometric/direct_kinematics");

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    currentPos_sub = n.subscribe("/hardware/right_arm/current_pose", 1000, currentPosCallback);


    // Data request to direct kinematic
    articular.push_back(0.0);
    articular.push_back(0.0);          //Inverse sense
    articular.push_back(0.0);
    articular.push_back(0.0);
    articular.push_back(0.0);
    articular.push_back(0.0);
    articular.push_back(0.0);
    srv_kd.request.articular_pose.data = articular;

    markerSetup();

    ros::Rate loop(10);

    while(ros::ok())
    {
        for(int i=0; i< currentPos.data.size(); i++)
	  articular[i] = currentPos.data[i];
	srv_kd.request.articular_pose.data = articular;
	
	std::cout << "CurrentPose size:  " << currentPos.data.size() << std::endl;
	std::cout << "Request size:  " << srv_kd.request.articular_pose.data.size() << std::endl;

	if(!cltDirectKinematics.call(srv_kd))
        {
            std::cout << std::endl << "Justina::Manip can't calling Direct kinematics service" << std::endl << std::endl;
            return false;
        }
        else
        {
	    loop.sleep();
            std::cout << "DirectKinematics.-> Calculated cartesian...." << std::endl;
	    std::cout << "[x, y, z, roll, pitch, yaw]" << std::endl;
	    for (int i=0; i < 7; i++) std::cout << "   " << srv_kd.response.cartesian_pose.data[i] << std::endl;

	    try
	    { 
	      listener.lookupTransform("/base_link", "/base_ra_arm", ros::Time(0), transform);
	    }
	    catch (tf::TransformException &ex)
	    {
	      ROS_ERROR("%s",ex.what());
	      ros::Duration(1.0).sleep();
	      continue;
	    }
            tf::Vector3 v(srv_kd.response.cartesian_pose.data[0],
			  srv_kd.response.cartesian_pose.data[1],
                          srv_kd.response.cartesian_pose.data[2]);
            v = transform * v;

            std::cout << "respect robot" << std::endl;
            std::cout << "    x = " << v.x() << std::endl;
            std::cout << "    y = " << v.y() << std::endl;
            std::cout << "    z = " << v.z() << std::endl;
            endEffector_pose.position.x = v.x();
            endEffector_pose.position.y = v.y();
            endEffector_pose.position.z = v.z();
        }

    	endEffector_marker.points.push_back(endEffector_pose.position);
	marker_pub.publish(endEffector_marker);
       
	std::cout << "---------------------------" << std::endl;
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
