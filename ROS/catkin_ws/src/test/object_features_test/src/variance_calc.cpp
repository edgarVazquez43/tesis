#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float32MultiArray.h"


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR WORKSPACE RIGHT ARM BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "variance_calculator");
    ros::NodeHandle n;

    //std::fstream openFile("/home/edgar/ws_points.txt", std::ios_base::in);
    std::fstream openFile("/home/edgar/github/tesis/resultados/objetos/Alturas/height_cerealBox.txt", std::ios_base::in);
    
    float x, h_edd, h_yisus;
    std::vector<float> eddList, yisusList;
    int i=0;


    markerSetup();
    
    ros::Rate loop(80);

    for(int i = 0; i < 100; i++)
    {
      std::string line;
      std::getline(openFile, line);   
      std::istringstream in(line);      //make a stream for the line itself
      in >> x >> h_edd >> h_yisus;      //now read the whitespace-separated floats
      
      eddList.push_back(h_edd);
      yisusList.push_back(h_yisus);
      
    }
    

    while(ros::ok())
    {
        
	
        ros::spinOnce();
        loop.sleep();
        i++;

        if(i>100)
            return 0;
    }
    return 0;
}
