#include <iostream>

#include "ros/ros.h"

#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "vision_msgs/DetectObjects.h"
#include "vision_msgs/FindPlane.h"
#include "manip_msgs/DirectKinematics.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR TIMING MEASUREMENT PLANES BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "time_measure");
    ros::NodeHandle n;

    
    vision_msgs::DetectObjects detectObj_msg;
    vision_msgs::FindPlane findPlane_msg;


    ros::ServiceClient cltDetectObjectsPCA;
    ros::ServiceClient cltFindPlanes;
    

    cltDetectObjectsPCA = n.serviceClient<vision_msgs::DetectObjects>("vision/detect_object/PCA_calculator");
    cltFindPlanes = n.serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/findPlane");




    ros::Rate loop(10);

    while(ros::ok())
    {
      
        if(!cltDetectObjectsPCA.call(detectObj_msg))
        {
            std::cout << std::endl << "Justina::Vision error calling Edd-II Service" << std::endl << std::endl;
            return false;
        }

	// if(!cltFindPlanes.call(findPlane_msg))
        // {
        //     std::cout << std::endl << "Justina::Vision error calling Yisus Service" << std::endl << std::endl;
        //     return false;
        // }

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
