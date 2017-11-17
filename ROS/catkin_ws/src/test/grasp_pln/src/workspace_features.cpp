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

visualization_msgs::Marker vertix_marker, line_list;

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
    vertix_marker.header.frame_id = "base_link";    
    vertix_marker.header.stamp = ros::Time::now();
    vertix_marker.ns = "vertix";
    vertix_marker.pose.orientation.w = 1.0;
    vertix_marker.id = 0;
    vertix_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    line_list.header.frame_id = "base_link";    
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "bounding_box";
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    vertix_marker.scale.x = 0.040;
    vertix_marker.scale.y = 0.040;
    vertix_marker.color.r = 1.0f;
    vertix_marker.color.a = 1.0;

    
    line_list.scale.x = 0.005;
    line_list.scale.y = 0.005;
    line_list.color.b = 1.0f;
    line_list.color.a = 1.0;
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR WORKSPACE RIGHT ARM BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "workspace_features");
    ros::NodeHandle n;
    ros::Rate loop(10);

    ros::Publisher marker_pub;

    
    visualization_msgs::Marker vertix;
    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;

    std::vector<float> min_max;
    std::vector<geometry_msgs::Point> workSpace;
    std::fstream openFile("/home/edgar/ws_points.txt", std::ios_base::in);
    
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
    
    float x, y, z;
    int i=0;

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    markerSetup();

    // Read the txt file points of workspace  //
    workSpace = readTextFile();
    min_max = getMinMax(workSpace);

    x_min = min_max[0] + 0.30;
    x_max = min_max[1] - 0.30;

    y_min = min_max[2] + 0.30;
    y_max = min_max[3] - 0.30;

    z_min = min_max[4] + 0.25;
    z_max = min_max[5] - 0.38;
    
    std::cout << "x_min: " << x_min
	      << "   x_max: " << x_max << std::endl
	      << "y_min: " << y_min
	      << "   y_max: " << y_max << std::endl
	      << "z_min: " << z_min
	      << "   z_max: " << z_max << std::endl;

    
    p1.x = x_min;
    p1.y = y_min;
    p1.z = z_min;
    vertix_marker.points.push_back(p1);

    p2.x = x_max;
    p2.y = y_min;
    p2.z = z_min;
    vertix_marker.points.push_back(p2);

    p3.x = x_max;
    p3.y = y_max;
    p3.z = z_min;
    vertix_marker.points.push_back(p3);
    
    p4.x = x_max + 0.18;
    p4.y = y_max;
    p4.z = z_max;
    vertix_marker.points.push_back(p4);
    
    p5.x = x_min;
    p5.y = y_max;
    p5.z = z_max;
    vertix_marker.points.push_back(p5);
    
    p6.x = x_min;
    p6.y = y_max;
    p6.z = z_min;
    vertix_marker.points.push_back(p6);

    p7.x = x_min;
    p7.y = y_min;
    p7.z = z_max;
    vertix_marker.points.push_back(p7);

    p8.x = x_max + 0.18;
    p8.y = y_min;
    p8.z = z_max;
    vertix_marker.points.push_back(p8);

    std::cout << "p1: " << std::endl << p1 << std::endl;
    std::cout << "p2: " << std::endl << p2 << std::endl;
    std::cout << "p3: " << std::endl << p3 << std::endl;
    std::cout << "p4: " << std::endl << p4 << std::endl;
    std::cout << "p5: " << std::endl << p5 << std::endl;
    std::cout << "p6: " << std::endl << p6 << std::endl;
    std::cout << "p7: " << std::endl << p7 << std::endl;
    std::cout << "p8: " << std::endl << p8 << std::endl;
    
    line_list.points.push_back(p1);
    line_list.points.push_back(p2);

    line_list.points.push_back(p1);
    line_list.points.push_back(p7);

    line_list.points.push_back(p1);
    line_list.points.push_back(p6);

    line_list.points.push_back(p2);
    line_list.points.push_back(p3);

    line_list.points.push_back(p2);
    line_list.points.push_back(p8);

    line_list.points.push_back(p3);
    line_list.points.push_back(p4);

    line_list.points.push_back(p3);
    line_list.points.push_back(p6);

    line_list.points.push_back(p6);
    line_list.points.push_back(p5);

    line_list.points.push_back(p5);
    line_list.points.push_back(p7);

    line_list.points.push_back(p7);
    line_list.points.push_back(p8);

    line_list.points.push_back(p8);
    line_list.points.push_back(p4);

    line_list.points.push_back(p4);
    line_list.points.push_back(p5);

    while(ros::ok())
      {
	marker_pub.publish(vertix_marker);
	marker_pub.publish(line_list);
	ros::spinOnce();
	loop.sleep();
      }
    
    return 0;
}
