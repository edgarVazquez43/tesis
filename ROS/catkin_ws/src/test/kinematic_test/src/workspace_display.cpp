#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float32MultiArray.h"
#include "manip_msgs/DirectKinematics.h"

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


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR WORKSPACE RIGHT ARM BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "workspace_display");
    ros::NodeHandle n;

    ros::Publisher marker_pub;
    geometry_msgs::Pose endEffector_pose;
    
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    //std::fstream openFile("/home/edgar/ws_points.txt", std::ios_base::in);
    std::fstream openFile("/home/edgar/ws_moveit_points.txt", std::ios_base::in);
    
    float x, y, z;
    int i=0;


    markerSetup();
    
    ros::Rate loop(80);

    while(ros::ok())
    {
        std::string line;
        std::getline(openFile, line);   
        std::istringstream in(line);      //make a stream for the line itself
        in >> x >> y >> z;                //now read the whitespace-separated floats

	endEffector_pose.position.x = x;
        endEffector_pose.position.y = y;
        endEffector_pose.position.z = z;
        endEffector_marker.points.push_back(endEffector_pose.position);

    	marker_pub.publish(endEffector_marker);
        ros::spinOnce();
        loop.sleep();
        i++;

        if(i>6200)
            return 0;
    }
    return 0;
}
