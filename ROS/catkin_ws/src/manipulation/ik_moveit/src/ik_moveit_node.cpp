#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/DirectKinematicsFloatArray.h"


robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* joint_group_rightArm;
robot_state::JointModelGroup* joint_group_leftArm;
std::vector<std::string> joint_names_rightArm;
std::vector<std::string> joint_names_leftArm;


// Code for calculate direcKinematic
// bool dk_right_arm(std::vector<double> pose)
// {
//   kinematic_state->setJointGroupPositions(joint_group_rightArm, pose);
//   const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("ra_link_grip_center");
//    /* Print end-effector pose. Remember that this is in the model frame */
//   ROS_INFO_STREAM("->  RIGHT ARM GRIPPER CENTER POSITION  -----------");
//   ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
//   ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
//   Eigen::Vector3d euler = end_effector_state.rotation().eulerAngles(2,1,2);
//   std::cout << "Euler angles: " << euler[0]<< "  " << euler[1]<< "  " << euler[2] << std::endl;
  
//   return true;
// }

bool callbackRightArmDK(manip_msgs::DirectKinematicsFloatArray::Request &req,
			manip_msgs::DirectKinematicsFloatArray::Response &resp)
{
  std::cout << std::endl <<" -------> " << std::endl; 
  std::cout << " Calling service to calculate Right Arm Direct Kinematic" << std::endl;
  if(req.articular_pose.data.size() != 7)
    return false;
    
  std::vector<double> pose;
  pose.resize(7);
  for(int i = 0; i < pose.size(); i++)
  {
    pose[i]=req.articular_pose.data[i];
    std::cout << pose[i] << " <--  " << req.articular_pose.data[i] << std::endl;
  }
  
  kinematic_state->setJointGroupPositions(joint_group_rightArm, pose);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("ra_link_grip_center");
   /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("->  RIGHT ARM GRIPPER CENTER POSITION  -----------");
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  Eigen::Vector3d euler = end_effector_state.rotation().eulerAngles(0,1,2);
  std::cout << "Euler angles: " << euler[0]<< "  " << euler[1]<< "  " << euler[2] << std::endl;

  resp.cartesian_pose.data.resize(7);
  resp.cartesian_pose.data[0] = end_effector_state.translation()[0];
  resp.cartesian_pose.data[1] = end_effector_state.translation()[1];
  resp.cartesian_pose.data[2] = end_effector_state.translation()[2];

  resp.cartesian_pose.data[3] = euler[0];
  resp.cartesian_pose.data[4] = euler[1];
  resp.cartesian_pose.data[5] = euler[2];
  resp.cartesian_pose.data[6] = 0.0;

  return true;									    
}


// bool dk_left_arm(std::vector<double> pose)
// {
//   kinematic_state->setJointGroupPositions(joint_group_leftArm, pose);
//   const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("la_link_grip_center");
//    /* Print end-effector pose. Remember that this is in the model frame */
//   ROS_INFO_STREAM("->  LEFT ARM GRIPPER CENTER POSITION  -----------");
//   ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
//   ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
  
//   return true;
// }

bool callbackLeftArmDK(manip_msgs::DirectKinematicsFloatArray::Request &req,
		       manip_msgs::DirectKinematicsFloatArray::Response &resp)
{
  std::cout << std::endl <<" -------> " << std::endl;
  std::cout << "Calling service to calculate Left Arm Direct Kinematic" << std::endl;
  if(req.articular_pose.data.size() != 7)
    return false;
    
  std::vector<double> pose;
  pose.resize(7);
  for(int i = 0; i < pose.size(); i++)
  {
    pose[i]=req.articular_pose.data[i];
    std::cout << pose[i] << " <--  " << req.articular_pose.data[i] << std::endl;
  }
  
  kinematic_state->setJointGroupPositions(joint_group_rightArm, pose);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("la_link_grip_center");
   /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("->  RIGHT ARM GRIPPER CENTER POSITION  -----------");
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  Eigen::Vector3d euler = end_effector_state.rotation().eulerAngles(0,1,2);
  std::cout << "Euler angles: " << euler[0]<< "  " << euler[1]<< "  " << euler[2] << std::endl;

  resp.cartesian_pose.data[0] = end_effector_state.translation()[0];
  resp.cartesian_pose.data[1] = end_effector_state.translation()[1];
  resp.cartesian_pose.data[2] = end_effector_state.translation()[2];

  resp.cartesian_pose.data[3] = euler[0];
  resp.cartesian_pose.data[4] = euler[1];
  resp.cartesian_pose.data[5] = euler[2];
  resp.cartesian_pose.data[6] = 0.0;
}



// Code for calculate inverseKinematic
// bool ik_right_arm(std::vector<double> pose)
// {

//   return true;
// }

bool callbackRightArmIK(manip_msgs::InverseKinematicsFloatArray::Request &req,
		       manip_msgs::InverseKinematicsFloatArray::Response &resp)
{
  std::cout << std::endl
	    << "Calling service to calculate Right Arm Inverse Kinematic"
	    << std::endl;
  std::cout << "----- Inverse kinematic------" << std::endl;

  bool found_ik;
  std::vector<double> result;
  
  if(req.cartesian_pose.data.size() != 7)
    return false;

  // std::cout << "Desire pose: ["
  // 	    << req.cartesian_pose.data[0] << ", "
  // 	    << req.cartesian_pose.data[1] << ", "
  // 	    << req.cartesian_pose.data[2] << "]" << std::endl;
  // 
    
  Eigen::Affine3d desired_pose = Eigen::Affine3d::Identity();
  desired_pose.translate(Eigen::Vector3d((double)req.cartesian_pose.data[0],
					 (double)req.cartesian_pose.data[1],
					 (double)req.cartesian_pose.data[2]));
  
  desired_pose.rotate(Eigen::AngleAxisd((double)req.cartesian_pose.data[3]  ,   Eigen::Vector3d(0,0,1)));   //yaw
  desired_pose.rotate(Eigen::AngleAxisd((double)req.cartesian_pose.data[4]  ,   Eigen::Vector3d(0,1,0)));   //pitch
  desired_pose.rotate(Eigen::AngleAxisd((double)req.cartesian_pose.data[5]  ,   Eigen::Vector3d(1,0,0)));   //roll

  std::cout << std::endl
	    << "Desired_pose:" << std::endl
	    << desired_pose.translation() << std::endl
	    << desired_pose.rotation() << std::endl    
	    << std::endl;
  std::cout << "Trying to calculate inverse kinematic.... " << std::endl;

  found_ik = kinematic_state->setFromIK(joint_group_rightArm, desired_pose, "ra_link_grip_center", 10, 0.5);

  if (found_ik)
  {
    resp.articular_pose.data.resize(7);
    kinematic_state->copyJointGroupPositions(joint_group_rightArm, result);
    for (std::size_t i = 0; i < result.size(); ++i)
    {
      ROS_INFO("Joint: %f", result[i]);
      resp.articular_pose.data[i] = result[i];
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
    return false;
  }

  return true;
}


// bool ik_left_arm(std::vector<double> pose)
// {

//   return true;
// }

bool callbackLeftArmIK(manip_msgs::InverseKinematicsFloatArray::Request &req,
		       manip_msgs::InverseKinematicsFloatArray::Response &resp)
{
  std::cout << "Calling service to calculate Right Arm Inverse Kinematic" << std::endl;


  return true;
}



int main(int argc, char** argv)
{
  std::cout << "INITIALIZING INVERSE KINEMATIC NODE BY EDD-II, (MOVEIT)" << std::endl;
  ros::init(argc, argv, "ik_moveit_node");
  ros::NodeHandle n;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");    // Instancia de RobotModelLoader
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();      // Get kinematic model
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Create ROS services
  ros::ServiceServer srvSrvLAIKFloatArray = n.advertiseService("ik_moveit/la_inverse_kinematics", callbackLeftArmIK);
  ros::ServiceServer srvSrvRAIKFloatArray = n.advertiseService("ik_moveit/ra_inverse_kinematics", callbackRightArmIK);
  ros::ServiceServer srvSrvLADKFloatArray = n.advertiseService("ik_moveit/la_direct_kinematics", callbackLeftArmDK);
  ros::ServiceServer srvSrvRADKFloatArray = n.advertiseService("ik_moveit/ra_direct_kinematics", callbackRightArmDK);
  
  // Create a kinematic_state
  kinematic_state = robot_state::RobotStatePtr( new robot_state::RobotState(kinematic_model) );
  joint_group_rightArm = kinematic_model->getJointModelGroup("right_arm");
  joint_group_leftArm = kinematic_model->getJointModelGroup("left_arm");
  joint_names_rightArm = joint_group_rightArm->getVariableNames();
  joint_names_leftArm = joint_group_leftArm->getVariableNames();


  std::vector<double> pose;
  pose.resize(7);
  pose[0]=  0.0;
  pose[1]=  0.0;
  pose[2]=  0.0;
  pose[3]=  0.0;
  pose[4]=  0.0;
  pose[5]=  0.0;
  pose[6]=  0.0;
  kinematic_state->setJointGroupPositions(joint_group_rightArm, pose);

  
  Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform("ra_link_grip_center");
  
  Eigen::Affine3d desired_pose = end_effector_state;
  desired_pose.translation() = Eigen::Vector3d(0.00005288, -0.225024, 0.70);
  // desired_pose.rotate(Eigen::AngleAxisd(0.0 ,   Eigen::Vector3d(0,0,1)));      // yaw
  // desired_pose.rotate(Eigen::AngleAxisd(1.5707 ,   Eigen::Vector3d(0,1,0)));   // pitch
  // desired_pose.rotate(Eigen::AngleAxisd(-1.5707 ,   Eigen::Vector3d(0,0,1)));  // roll

  std::cout << std::endl << " ---- END-EFFECTOR ---- " << std::endl;
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  std::cout << std::endl << " ---- DESIRED POSITION ---- " << std::endl;
  ROS_INFO_STREAM("Translation: " << desired_pose.translation());
  ROS_INFO_STREAM("Rotation: " << desired_pose.rotation());

  
  
  
  bool found_ik;
  std::vector<double> result;
  
  ros::Rate loop(10);
  
  while(ros::ok())
  {
    ros::spinOnce();    
    std::cout << "----- Inverse kinematic------" << std::endl;

    found_ik = kinematic_state->setFromIK(joint_group_rightArm, desired_pose, "ra_link_grip_center", 5, 0.8);

    // found_ik = kinematic_state->setFromIK(joint_group_rightArm, end_effector_state, 5, 0.1);
    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_group_rightArm, result);
      for (std::size_t i = 0; i < result.size(); ++i)
      {
    	ROS_INFO("Joint: %f", result[i]);
      }
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }
    std::cout << "   " << std::endl; 
    
    loop.sleep();
  }
  
}
