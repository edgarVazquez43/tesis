#include "ros/ros.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
// #include "control_msgs/CalculateIK.h"
// #include "control_msgs/CalculateDK.h"

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


// Code for calculate inverseKinematic
bool ik_right_arm(std::vector<double> pose)
{

  return true;
}

bool ik_left_arm(std::vector<double> pose)
{

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

  //Create a kinematic_state
  kinematic_state = robot_state::RobotStatePtr( new robot_state::RobotState(kinematic_model) );
  joint_group_rightArm = kinematic_model->getJointModelGroup("right_arm");
  joint_group_leftArm = kinematic_model->getJointModelGroup("left_arm");
  joint_names_rightArm = joint_group_rightArm->getVariableNames();
  joint_names_leftArm = joint_group_leftArm->getVariableNames();

  // DIRECT KINEMATIC
  std::vector<double> joint_values_ra;
  joint_values_ra.resize(7);
  joint_values_ra[0] = -0.8;
  joint_values_ra[2] =  0.8;
  joint_values_ra[5] = -0.8;
   
  std::vector<double> joint_values_la;
  joint_values_la.resize(7);
  joint_values_la[0] = -0.8;
  joint_values_la[2] =  0.8;
  joint_values_la[5] = -0.8;

  
  dk_right_arm(joint_values_ra);
  dk_left_arm(joint_values_la);
  
  // END --- DIRECT KINEMATIC


  
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

  // bool found_ik = kinematic_state->setFromIK(joint_group_rightArm, en);
  
  ros::Rate loop(10);
  
  while(ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
  }
  
}
