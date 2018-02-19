#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "vision_msgs/DetectObjects.h"
#include "manip_msgs/DirectKinematics.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"

visualization_msgs::Marker endEffector_marker;
visualization_msgs::Marker centroid_marker, axis_list_marker;

ros::Publisher marker_pub;
ros::Publisher right_arm_goal_pose_pub;
ros::Publisher left_arm_goal_pose_pub;

ros::ServiceClient cltIKinematicsLA;
ros::ServiceClient cltIKinematicsRA;
ros::ServiceClient cltIKinematicsMark;
ros::ServiceClient cltDetectObjectsPCA;


bool transformPoint(float &x, float &y, float &z)
{
  bool succes = true;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::Vector3 v(x, y, z);

  while(ros::ok())
    {
      succes = true;
      try
	{ 
	  listener.lookupTransform("/base_ra_arm", "/base_link", ros::Time(0), transform);
	}
      catch (tf::TransformException &ex)
	{
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	  succes = false;
	}

      std::cout << succes << std::endl;
      if(succes)
	break;
    }

  if(succes)
    {  
      v = transform * v;
      x = v.x();
      y = v.y();
      z = v.z();
    }
  
  return true;
}



bool detectObjet(std::vector<float>& pose,
		 geometry_msgs::Vector3& axis_resp_0,
		 geometry_msgs::Vector3& axis_resp_1,
		 geometry_msgs::Vector3& axis_resp_2)
{  
  vision_msgs::DetectObjects srv_detectObj;

  pose.resize(6);
  
  if(!cltDetectObjectsPCA.call(srv_detectObj))
    {
      std::cout << std::endl << "Justina::Service detect object fail.... :(" << std::endl << std::endl;
      std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl;
      return false;
    }
  
  pose[0] = srv_detectObj.response.recog_objects[0].pose.position.x;
  pose[1] = srv_detectObj.response.recog_objects[0].pose.position.y;
  pose[2] = srv_detectObj.response.recog_objects[0].pose.position.z;

  axis_resp_0 = srv_detectObj.response.recog_objects[0].principal_axis[0];
  axis_resp_1 = srv_detectObj.response.recog_objects[0].principal_axis[1];
  axis_resp_2 = srv_detectObj.response.recog_objects[0].principal_axis[2];

  std::cout << "Centroid: " << pose[0] << "  " << pose[1] << "  " << pose[2] << std::endl << std::endl;
  
  std::cout << "Principal_axis: " << std::endl
	    << axis_resp_0 << std::endl;
  std::cout << axis_resp_1 << std::endl;
  std::cout << axis_resp_2 << std::endl;


  // Calcular angulos aqui

  //Roll Angle
  pose[3] = 0.0;

   //Pitch Angle
  pose[4] = 0.0;

   //Yaw Angle
  pose[5] = 0.0;
  

  return true;
}



bool la_ikCalculate(std::vector<float>& articular, std::vector<float> cartesian)
{
  std_msgs::Float32MultiArray la_gp_msgs;
  manip_msgs::InverseKinematicsFloatArray srv_ki_moveIt;

  if(cartesian.size() == 3)
    {
      cartesian.push_back(0.0);
      cartesian.push_back(0.0);
      cartesian.push_back(0.0);
    }
  
  srv_ki_moveIt.request.cartesian_pose.data = cartesian;

  articular.resize(6);
  la_gp_msgs.data.resize(7);
  
  if(!cltIKinematicsLA.call(srv_ki_moveIt))
    {
      std::cout << std::endl << "Justina::Manip can't calling inverse kinematics service" << std::endl << std::endl;
      return false;
    }

  std::cout << std::endl <<
    "Move-It::: Success service" << std::endl << std::endl;

  for (int i=0; i < 7; i++)
    {
  
      la_gp_msgs.data[i] = srv_ki_moveIt.response.articular_pose.data[i];
    } 

  left_arm_goal_pose_pub.publish(la_gp_msgs);

  return true;
}


bool ra_ikCalculate(std::vector<float>& articular, std::vector<float> cartesian)
{
  std_msgs::Float32MultiArray ra_gp_msgs;
  manip_msgs::InverseKinematicsFloatArray srv_ki_moveIt;
  
  srv_ki_moveIt.request.cartesian_pose.data = cartesian;

  articular.resize(6);
  ra_gp_msgs.data.resize(7);
  
  if(!cltIKinematicsRA.call(srv_ki_moveIt))
    {
      std::cout << std::endl << "Justina::Manip can't calling inverse kinematics service" << std::endl << std::endl;
      return false;
    }

  std::cout << std::endl <<
    "Move-It::: Success service" << std::endl << std::endl;

  for (int i=0; i < 7; i++)
    {
  
      ra_gp_msgs.data[i] = srv_ki_moveIt.response.articular_pose.data[i];
    } 

  right_arm_goal_pose_pub.publish(ra_gp_msgs);
  
  return true;
}


bool ra_mark_ikCalculate(std::vector<float>& articular, std::vector<float> cartesian)
{
  std_msgs::Float32MultiArray ra_gp_msgs;
  manip_msgs::InverseKinematicsFloatArray srv_ki;

  transformPoint(cartesian[0], cartesian[1], cartesian[2]);
  srv_ki.request.cartesian_pose.data = cartesian;

  ra_gp_msgs.data.resize(7);
  
  if(!cltIKinematicsMark.call(srv_ki))
    {
      std::cout << std::endl << "Justina::Manip can't calling inverse kinematics service" << std::endl << std::endl;
      return false;
    }


  std::cout << std::endl
	    << "Inverse Kinematic Mark::: Success service" << std::endl << std::endl;

  for (int i=0; i < 7; i++)
      ra_gp_msgs.data[i] = srv_ki.response.articular_pose.data[i];


  std::cout << std::endl
	    << "Moving right arm....   :)" << std::endl << std::endl;
  right_arm_goal_pose_pub.publish(ra_gp_msgs);
  
  return true;
}


bool moveLeftArm(std::vector<float> articular)
{
    if (articular.size() != 7)
  {
    std::cout << "Left arm must be a seven values... " << std::endl;
    return false;
  }
    
  left_arm_goal_pose_pub.publish(articular); 
  return true;
}


bool moveRightArm(std::vector<float> articular)
{
  if (articular.size() != 7)
  {
    std::cout << "Right arm must be a seven values... " << std::endl;
    return false;
  }
    
  right_arm_goal_pose_pub.publish(articular);
  return true;
}






bool markerSetup()
{

    centroid_marker.header.frame_id = "base_link";
    endEffector_marker.header.frame_id = "base_link";
    axis_list_marker.header.frame_id = "base_link";
    
    centroid_marker.header.stamp = ros::Time::now();
    endEffector_marker.header.stamp = ros::Time::now();
    axis_list_marker.header.stamp = ros::Time::now();

    centroid_marker.ns = "centroid";
    axis_list_marker.ns = "principal axis";
    endEffector_marker.ns = "endEffector_r";
    
    centroid_marker.pose.orientation.w = 1.0;
    endEffector_marker.pose.orientation.w = 1.0;
    axis_list_marker.pose.orientation.w = 1.0;

    centroid_marker.id = 0;
    endEffector_marker.id = 0;
    axis_list_marker.id = 1;

    centroid_marker.type = visualization_msgs::Marker::SPHERE;
    endEffector_marker.type = visualization_msgs::Marker::SPHERE;
    axis_list_marker.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    centroid_marker.scale.x = 0.035;
    centroid_marker.scale.y = 0.035;
    centroid_marker.scale.z = 0.035;

    endEffector_marker.scale.x = 0.040;
    endEffector_marker.scale.y = 0.040;
    endEffector_marker.scale.z = 0.040;

    axis_list_marker.scale.x = 0.03;
    axis_list_marker.scale.y = 0.03;
    axis_list_marker.scale.z = 0.03;

    centroid_marker.color.g = 1.0f;
    centroid_marker.color.a = 1.0;

    endEffector_marker.color.g = 1.0f;
    endEffector_marker.color.a = 1.0;

    axis_list_marker.color.r = 1.0f;
    axis_list_marker.color.a = 1.0;

    return true;
}

visualization_msgs::Marker buildMarkerAxis(geometry_msgs::Vector3 PCA_axis_0,
					   geometry_msgs::Vector3 PCA_axis_1,
					   geometry_msgs::Vector3 PCA_axis_2,
					   geometry_msgs::Pose centroid_pose)
{
    axis_list_marker.points.clear();
    geometry_msgs::Point px_centroid;
    geometry_msgs::Point p_0, p_1, p_2;

    px_centroid = centroid_pose.position;

    p_0.x = px_centroid.x + PCA_axis_0.x;
    p_0.y = px_centroid.y + PCA_axis_0.y;
    p_0.z = px_centroid.z + PCA_axis_0.z;

    p_1.x = px_centroid.x + PCA_axis_1.x;
    p_1.y = px_centroid.y + PCA_axis_1.y;
    p_1.z = px_centroid.z + PCA_axis_1.z;

    p_2.x = px_centroid.x + PCA_axis_2.x;
    p_2.y = px_centroid.y + PCA_axis_2.y;
    p_2.z = px_centroid.z + PCA_axis_2.z;

    axis_list_marker.points.push_back(px_centroid);
    axis_list_marker.points.push_back(p_0);

    axis_list_marker.points.push_back(px_centroid);
    axis_list_marker.points.push_back(p_1);

    axis_list_marker.points.push_back(px_centroid);
    axis_list_marker.points.push_back(p_2);

    return axis_list_marker;
}




int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR GRASP OBJECT BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "grasp_pln");
    ros::NodeHandle n;

    std::vector<float> object_pose;
    std::vector<float> articular_arm;

    geometry_msgs::Pose centroid, endEffector_pose;
    geometry_msgs::Vector3 axis_resp_0, axis_resp_1, axis_resp_2; 

    
    // ROS  Service Client
    cltIKinematicsLA = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_moveit/la_inverse_kinematics");
    cltIKinematicsRA = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_moveit/ra_inverse_kinematics");
    cltIKinematicsMark = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");
    cltDetectObjectsPCA = n.serviceClient<vision_msgs::DetectObjects>("/vision/detect_object/PCA_calculator");

    // ROS Topic Publisher 
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    right_arm_goal_pose_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 10);
    left_arm_goal_pose_pub = n.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 10);


 
    markerSetup();

    ros::Rate loop(10);

    while(ros::ok())
    {

      detectObjet(object_pose, axis_resp_0, axis_resp_1, axis_resp_2);
      ra_ikCalculate(articular_arm, object_pose);

      
      endEffector_marker.pose.position = endEffector_pose.position;
      centroid_marker.pose.position = centroid.position;
      
      marker_pub.publish(endEffector_marker);
      marker_pub.publish(centroid_marker);
      axis_list_marker = buildMarkerAxis(axis_resp_0,
					 axis_resp_1,
					 axis_resp_2,
					 centroid);
      marker_pub.publish(axis_list_marker);

      std::cout << "---------------------------" << std::endl;
      ros::spinOnce();
      loop.sleep();
    }

    
    return 0;
}
