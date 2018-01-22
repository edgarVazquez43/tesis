#include "ros/ros.h"
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
bool dk_right_arm(std::vector<double> pose)
{
  kinematic_state->setJointGroupPositions(joint_group_rightArm, pose);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("ra_link_grip_center");
   /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("->  RIGHT ARM GRIPPER CENTER POSITION  -----------");
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
  Eigen::Vector3d euler = end_effector_state.rotation().eulerAngles(2,1,2);
  std::cout << "Euler angles: " << euler[0]<< "  " << euler[1]<< "  " << euler[2] << std::endl;
  
  return true;
}

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


bool dk_left_arm(std::vector<double> pose)
{
  kinematic_state->setJointGroupPositions(joint_group_leftArm, pose);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("la_link_grip_center");
   /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("->  LEFT ARM GRIPPER CENTER POSITION  -----------");
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
  
  return true;
}

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
bool ik_right_arm(std::vector<double> pose)
{

  return true;
}

bool callbackRightArmIK(manip_msgs::InverseKinematicsFloatArray::Request &req,
		       manip_msgs::InverseKinematicsFloatArray::Response &resp)
{
  std::cout << "Calling service to calculate Right Arm Inverse Kinematic" << std::endl;


  return true;
}


bool ik_left_arm(std::vector<double> pose)
{

  return true;
}

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
  
  // std::vector<double> joint_values;
  
  // // Paso por referencia a la variable joint_values
  // kinematic_state->copyJointGroupPositions(joint_group_rightArm, joint_values);
  // for(std::size_t i=0; i<joint_name.size(); ++i)
  // {
  //   ROS_INFO("Joint %s: %f", joint_name[i].c_str(), joint_values[i]);
  // }

  // joint_values[0] = 0.8;
  // kinematic_state->setJointGroupPositions(joint_group_leftArm, joint_values);

  // std::cout << "Update joints values" << std::endl;
  // kinematic_state->copyJointGroupPositions(joint_group_rightArm, joint_values);
  // for(std::size_t i=0; i<joint_name.size(); ++i)
  // {
  //   ROS_INFO("Joint %s: %f", joint_name[i].c_str(), joint_values[i]);
  // }

  std::vector<double> joint_values_ra;
  joint_values_ra.resize(7);

  Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform("ra_link_grip_center");
  end_effector_state.translate(Eigen::Vector3d(0.0, -0.225024, 0.595));
  
  Eigen::Affine3d desired_pose = Eigen::Affine3d::Identity();
  desired_pose.translate(Eigen::Vector3d(0.0, -0.225024, 0.595));
  desired_pose.rotate(Eigen::AngleAxisd(0.0 ,   Eigen::Vector3d(0,0,1)));   //yaw
  desired_pose.rotate(Eigen::AngleAxisd(1.5707 ,   Eigen::Vector3d(0,1,0)));   //pitch
  desired_pose.rotate(Eigen::AngleAxisd(-1.5707 ,   Eigen::Vector3d(0,0,1)));   //roll

  bool found_ik;
  std::vector<double> result;
  
  
  ros::Rate loop(10);
  
  while(ros::ok())
  {
    ros::spinOnce();
    
    // std::cout << "----- Direct kinematic ------" << std::endl;
    // dk_right_arm(joint_values_ra);
    // std::cout << " --- Desired position ----  " << std::endl;
    // ROS_INFO_STREAM("Translation: " << desired_pose.translation());
    // ROS_INFO_STREAM("Rotation: " << desired_pose.rotation());
    // std::cout << "   " << std::endl;
    
    // std::cout << "----- Inverse kinematic------" << std::endl;
    
    // found_ik = kinematic_state->setFromIK(joint_group_rightArm, end_effector_state, "ra_link_grip_center", 5, 0.1);
  
    // if (found_ik)
    //   {
    // 	kinematic_state->copyJointGroupPositions(joint_group_rightArm, result);
    // 	for (std::size_t i = 0; i < result.size(); ++i)
    // 	  {
    // 	    ROS_INFO("Joint: %f", result[i]);
    // 	  }
    //   }
    // else
    //   {
    // 	ROS_INFO("Did not find IK solution");
    //   }
    // std::cout << "   " << std::endl;
    loop.sleep();
  }
  
}
