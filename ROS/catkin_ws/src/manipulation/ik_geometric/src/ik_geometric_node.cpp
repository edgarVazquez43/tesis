#include <iostream>
#include "ros/ros.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/InverseKinematicsPath.h"
#include "manip_msgs/InverseKinematicsPose.h"
#include "manip_msgs/DirectKinematics.h"
#include "InverseKinematics.h"

//T O D O :   T H I S   I S   A   V E R Y   I M P O R T A N T   T O - D O !!!!!!!!!
//Dimensions of the arms should be taken from the robot description (urdf file in the planning/knowledge/hardware/justina.xml)
//Values of D1, D2, D3 and D4 correspond to Denavig-Hartenberg parameters and are given in the urdf
//In the origin tag of each joint.

void printHelp()
{
    std::cout << "INVERSE KINEMATICS GEOMETRIC BY MARCOSOFT..." << std::endl;
    std::cout << " - This node calculates the inverse kinematics using a geometric approach." << std::endl;
    std::cout << " - Inverse kinematics is calculated considering a 7DOF manipulator" << std::endl;
    std::cout << " - with all joints of revolute type and with a spheric wrist." << std::endl;
    std::cout << " - Calculations are made assuming that cartesian coords are w.r.t each arm." << std::endl;
    std::cout << " - Since both arms have almost the same frame (same orientation, " << std::endl;
    std::cout << " - and translated one from each other by the shoulders lenght), then, inverse kinematics" << std::endl;
    std::cout << " - can be calculated in the same way." << std::endl;
    std::cout << " - Planning algorithms should do the corresponding transforms (eg from base_link to left_arm_link)." << std::endl;
    std::cout << " - In cartesian coords, the seven values are x, y, z, roll, pitch, yaw and elbow. " << std::endl;
    std::cout << "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH" << std::endl;
}


// #############  INVERSE KINEMATIC  ####################
bool callbackInverseKinematicsFloatArray( manip_msgs::InverseKinematicsFloatArray::Request &req,
                                         manip_msgs::InverseKinematicsFloatArray::Response &resp)
{
    std::cout << "Calling service to calculate Inverse Kinematics...." << std::endl;

    if (req.cartesian_pose.data.size() == 7 )
    {
      //InverseKinematics::GetInverseKinematics_e(req.cartesian_pose.data, resp.articular_pose.data);
      InverseKinematics::GetInverseKinematics(req.cartesian_pose.data, resp.articular_pose.data);
      return true ;
    }
    else
      {
	std::cout << "Cannot calculate inverse kinematic, invalid args number... :( " << std::endl;
 	return false;
      }
}


// #############  DIRECT KINEMATIC  ################
bool callbackDirectKinematics(manip_msgs::DirectKinematics::Request &req, manip_msgs::DirectKinematics::Response &resp)
{
    std::cout << "Calling service to calculate Direct Kinematics...." << std::endl;

    return InverseKinematics::GetDirectKinematics(req.articular_pose.data, resp.cartesian_pose.data);
}




int main(int argc, char** argv)
{
  for (int i = 0; i < argc; i++)
    {
      std::string strParam(argv[i]);
      if (strParam.compare("--help") == 0 || strParam.compare("-h") == 0)
	{
	  printHelp();
	  return 0;
	}
    }


  std::cout << "INITIALIZING INVERSE KINEMATICS GEOMETRIC BY EDGAR-II... " << std::endl;
  ros::init(argc, argv, "low_level_moves");
  ros::NodeHandle n;
  ros::ServiceServer srvSrvIKFloatArray = n.advertiseService("ik_geometric/ik_float_array", callbackInverseKinematicsFloatArray);
  ros::ServiceServer srvSrvDirectKin = n.advertiseService("ik_geometric/direct_kinematics", callbackDirectKinematics);
  ros::Rate loop(10);
  
  while(ros::ok())
    {
      ros::spinOnce();
      loop.sleep();
    }
  return 0;
}
