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

geometry_msgs::Pose endEffector_pose;
visualization_msgs::Marker endEffector_marker;

std::vector<geometry_msgs::Point> readTextFile()
{
  geometry_msgs::Point p;
  std::vector<geometry_msgs::Point> workSpace;

  std::fstream openFile("/home/edgar/ws_points.txt", std::ios_base::in);
  float x, y, z;
  
  for(std::string line; std::getline(openFile, line); )   //read stream line by line
    {
      std::istringstream in(line);      //make a stream for the line itself  
      float x, y, z;
      in >> x >> y >> z;                //now read the whitespace-separated floats
    
      p.x = x;
      p.y = y;
      p.z = z;
      workSpace.push_back(p); 
    }
  //std::cout << "workspace_size: " << workSpace.size() << std::endl;
  //std::cout << "workspace[0]: " << workSpace[0] << std::endl;
  return workSpace;
}

std::vector<float> getMinMax(std::vector<geometry_msgs::Point> workSpace)
{
  std::vector<float> min_max;
  geometry_msgs::Point p;
  float x_min, x_max;
  float y_min, y_max;
  float z_min, z_max;

  x_min = 10000000;
  y_min = 10000000;
  z_min = 10000000;

  x_max = 0;
  y_max = 0;
  z_max = 0;

  std::cout << "workspace[0]: " << workSpace[0] << std::endl;
  for(int i=0; i < workSpace.size(); i++)
    {
      p = workSpace[i];
      if(p.x > x_max)
	x_max = p.x;

      if(p.y > y_max)
	y_max = p.y;

      if(p.z > z_max)
	z_max = p.z;

      //// Min calculate ///
      if(p.x < x_min)
	x_min = p.x;

      if(p.y < y_min)
	y_min = p.y;

      if(p.z < z_min)
	z_min = p.z;

    }
  min_max.push_back(x_min);
  min_max.push_back(x_max);
  min_max.push_back(y_min);
  min_max.push_back(y_max);
  min_max.push_back(z_min);
  min_max.push_back(z_max);

  return min_max;
}

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
    ros::init(argc, argv, "grasp_pln");
    ros::NodeHandle n;

    ros::Publisher marker_pub;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    std::fstream openFile("/home/edgar/ws_points.txt", std::ios_base::in);

    std::vector<geometry_msgs::Point> workSpace;
    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
    std::vector<float> min_max;

    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
    
    float x, y, z;
    int i=0;


    markerSetup();

    // Read the txt file points of workspace  //
    workSpace = readTextFile();
    min_max = getMinMax(workSpace);

    x_min = min_max[0];
    x_max = min_max[1];

    y_min = min_max[2];
    y_max = min_max[3];

    z_min = min_max[4];
    z_max = min_max[5];
    /*
    std::cout << "x_min: " << min_max[0]
	      << "   x_max: " << min_max[1] << std::endl
	      << "y_min: " << min_max[2]
	      << "   y_max: " << min_max[3] << std::endl
	      << "z_min: " << min_max[4]
	      << "   z_max: " << min_max[6] << std::endl;

    */
     std::cout << "x_min: " << x_min
	      << "   x_max: " << x_max << std::endl
	      << "y_min: " << y_min
	      << "   y_max: " << y_max << std::endl
	      << "z_min: " << z_min
	      << "   z_max: " << z_max << std::endl;

    
    p1.x = x_min;
    p1.y = y_min;
    p1.y = z_min;

    p2.x = x_max;
    p2.y = y_min;
    p2.y = z_min;

    p3.x = x_max;
    p3.y = y_max;
    p3.y = z_min;

    p4.x = x_max;
    p4.y = y_max;
    p4.y = z_max;

    p5.x = x_min;
    p5.y = y_max;
    p5.y = z_max;

    p6.x = x_min;
    p6.y = y_max;
    p6.y = z_min;

    p7.x = x_min;
    p7.y = y_min;
    p7.y = z_max;

    p8.x = x_max;
    p8.y = y_min;
    p8.y = z_max;
    
    return 0;
}
