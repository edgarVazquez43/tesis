#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <boost/thread/thread.hpp>
#include <visualization_msgs/Marker.h>
#include "vision_msgs/DetectObjects.h"
#include "manip_msgs/DirectKinematics.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"

class JustinaGrasp
{
private:
  static ros::Publisher right_arm_goal_pose_pub;
  static ros::Publisher left_arm_goal_pose_pub;
  
  static ros::ServiceClient cltIKinematicsLA;
  static ros::ServiceClient cltIKinematicsRA;
  static ros::ServiceClient cltIKinematicsMark;
  static ros::ServiceClient cltDetectObjectsPCA;
 

public:
  static bool setNodeHandle(ros::NodeHandle* nh);
  
  static bool transformPoint(float &x, float &y, float &z);
  static bool detectObjet(std::vector<float>& pose,
			  geometry_msgs::Vector3& axis_resp_0,
			  geometry_msgs::Vector3& axis_resp_1,
			  geometry_msgs::Vector3& axis_resp_2);
  static bool la_ikCalculate(std::vector<float>& articular, std::vector<float> cartesian);
  static bool ra_ikCalculate(std::vector<float>& articular, std::vector<float> cartesian);
  static bool ra_mark_ikCalculate(std::vector<float>& articular, std::vector<float> cartesian);
  static bool moveLeftArm(std::vector<float> articular);
  static bool moveRightArm(std::vector<float> articular);
  
};

// #################################################
//                 END OF CLASS
// #################################################




visualization_msgs::Marker endEffector_marker;
visualization_msgs::Marker centroid_marker, axis_list_marker;
visualization_msgs::Marker pca1, pca2, pca3, vectPlane;


ros::Publisher marker_pub;
ros::Publisher JustinaGrasp::right_arm_goal_pose_pub;
ros::Publisher JustinaGrasp::left_arm_goal_pose_pub;

ros::ServiceClient JustinaGrasp::cltIKinematicsLA;
ros::ServiceClient JustinaGrasp::cltIKinematicsRA;
ros::ServiceClient JustinaGrasp::cltIKinematicsMark;
ros::ServiceClient JustinaGrasp::cltDetectObjectsPCA;





bool JustinaGrasp::setNodeHandle(ros::NodeHandle* nh)
{
  std::cout << "JustinaGrasp.->Setting ros node..." << std::endl;

  // ROS  Service Client
  cltIKinematicsLA = nh->serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_moveit/la_inverse_kinematics");
  cltIKinematicsRA = nh->serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_moveit/ra_inverse_kinematics");
  cltIKinematicsMark = nh->serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");
  cltDetectObjectsPCA = nh->serviceClient<vision_msgs::DetectObjects>("/vision/detect_object/PCA_calculator");

  // ROS Topic Publisher 
  marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
  right_arm_goal_pose_pub = nh->advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 10);
  left_arm_goal_pose_pub = nh->advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 10);

  return true;
}

bool JustinaGrasp::transformPoint(float &x, float &y, float &z)
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



bool JustinaGrasp::detectObjet(std::vector<float>& pose,
		 geometry_msgs::Vector3& axis_resp_0,
		 geometry_msgs::Vector3& axis_resp_1,
		 geometry_msgs::Vector3& axis_resp_2)
{  
  vision_msgs::DetectObjects srv_detectObj;
  geometry_msgs::Vector3 aux;

  float objectYaw, objectYaw_rads;
  
  std::vector<float> mag_axis;
  std::vector<float> alpha, beta, gamma;
  
  alpha.resize(3);
  beta.resize(3);
  gamma.resize(3);
  mag_axis.resize(4);
  pose.resize(7);
  
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

  if( fabs(axis_resp_0.y) > fabs(aux.y)  )
    aux = axis_resp_0;

  if( fabs(axis_resp_1.y) > fabs(aux.y)  )
    aux = axis_resp_1;
	
  if( fabs(axis_resp_2.y) > fabs(aux.y)  )
    aux = axis_resp_0;

  std::cout << "Axis on plane: " << std::endl
	    << aux << std::endl;

  objectYaw_rads = -atan(aux.y/aux.x);
  objectYaw = objectYaw_rads * 180/3.141592;
	

  //Calculate magnitude principal axis
  mag_axis[0] = sqrt(axis_resp_0.x*axis_resp_0.x +
		     axis_resp_0.y*axis_resp_0.y +
		     axis_resp_0.z*axis_resp_0.z);

  mag_axis[1] = sqrt(axis_resp_1.x*axis_resp_1.x +
		     axis_resp_1.y*axis_resp_1.y +
		     axis_resp_1.z*axis_resp_1.z);

  mag_axis[2] = sqrt(axis_resp_2.x*axis_resp_2.x +
		     axis_resp_2.y*axis_resp_2.y +
		     axis_resp_2.z*axis_resp_2.z);
	
  mag_axis[3] = sqrt(aux.x*aux.x +
		     aux.y*aux.y +
		     aux.z*aux.z);

  objectYaw *= -1;
  std::cout << "Axis magnitude: " << std::endl
	    << mag_axis[0] << std::endl
	    << mag_axis[1] << std::endl
	    << mag_axis[2] << std::endl << std::endl;


  std::cout << "object yaw:  " << objectYaw << std::endl << std::endl;
  std::cout << "object yaw_rads:  " << objectYaw_rads << std::endl << std::endl;

  // Verify principal axis is in Z-Axis
  if(axis_resp_0.z > axis_resp_0.y &&
     axis_resp_0.z > axis_resp_0.x)
  {
    
    pose[3] = 0.0;               // Roll Angle
    pose[4] = objectYaw_rads;    // Pitch Angle
    pose[5] =  1.5707;    // Yaw Angle
  }
  else
  {
    pose[3] = -objectYaw_rads;               // Roll Angle
    pose[4] = 0.0;    // Pitch Angle
    pose[5] = 0.0;               // Yaw Angle
  }
  
 
  

  return true;
}



bool JustinaGrasp::la_ikCalculate(std::vector<float>& articular, std::vector<float> cartesian)
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

  articular.resize(7);
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
      articular[i] = srv_ki_moveIt.response.articular_pose.data[i];
    } 

  // left_arm_goal_pose_pub.publish(la_gp_msgs);

  return true;
}


bool JustinaGrasp::ra_ikCalculate(std::vector<float>& articular, std::vector<float> cartesian)
{
  std_msgs::Float32MultiArray ra_gp_msgs;
  manip_msgs::InverseKinematicsFloatArray srv_ki_moveIt;
  
  srv_ki_moveIt.request.cartesian_pose.data = cartesian;

  articular.resize(7);
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
      articular[i] = srv_ki_moveIt.response.articular_pose.data[i];
    } 

  right_arm_goal_pose_pub.publish(ra_gp_msgs);
  
  return true;
}


bool JustinaGrasp::ra_mark_ikCalculate(std::vector<float>& articular, std::vector<float> cartesian)
{
  std_msgs::Float32MultiArray ra_gp_msgs;
  manip_msgs::InverseKinematicsFloatArray srv_ki;

  cartesian.resize(7);
  JustinaGrasp::transformPoint(cartesian[0], cartesian[1], cartesian[2]);
  srv_ki.request.cartesian_pose.data = cartesian;

  articular.resize(7);
  ra_gp_msgs.data.resize(7);
  
  if(!cltIKinematicsMark.call(srv_ki))
    {
      std::cout << std::endl << "Justina::Manip can't calling inverse kinematics service" << std::endl << std::endl;
      return false;
    }


  std::cout << std::endl
	    << "Inverse Kinematic Mark::: Success service" << std::endl << std::endl;

  for (int i=0; i < 7; i++)
  {
    ra_gp_msgs.data[i] = srv_ki.response.articular_pose.data[i];
    articular[i] = srv_ki.response.articular_pose.data[i];
  }

  std::cout << std::endl
  	    << "Moving right arm....   :)" << std::endl << std::endl;
  right_arm_goal_pose_pub.publish(ra_gp_msgs);
  
  return true;
}


bool JustinaGrasp::moveLeftArm(std::vector<float> articular)
{
  std_msgs::Float32MultiArray msg;
  if (articular.size() != 7)
  {
    std::cout << "Left arm must be a seven values... " << std::endl;
    return false;
  }
  
  msg.data.resize(7);
  msg.data = articular;
    
  left_arm_goal_pose_pub.publish(msg); 
  return true;
}


bool JustinaGrasp::moveRightArm(std::vector<float> articular)
{
  std_msgs::Float32MultiArray msg;
  if (articular.size() != 7)
  {
    std::cout << "Right arm must be a seven values... " << std::endl;
    return false;
  }
  
  msg.data.resize(7);
  msg.data = articular;
    
  right_arm_goal_pose_pub.publish(msg);
  return true;
}


bool markerSetup()
{
    centroid_marker.header.frame_id = "base_link";
    pca1.header.frame_id = "base_link";
    pca2.header.frame_id = "base_link";
    pca3.header.frame_id = "base_link";
    vectPlane.header.frame_id = "base_link";
    
    centroid_marker.header.stamp = ros::Time::now();
    pca1.header.stamp = ros::Time::now();
    pca2.header.stamp = ros::Time::now();
    pca3.header.stamp = ros::Time::now();
    vectPlane.header.stamp = ros::Time::now();

    centroid_marker.ns = "centroid";
    pca1.ns = "principal axis1";
    pca2.ns = "principal axis2";
    pca3.ns = "principal axis3";
    vectPlane.ns = "principal axis3";
    
    centroid_marker.pose.orientation.w = 1.0;
    pca1.pose.orientation.w = 1.0;
    pca2.pose.orientation.w = 1.0;
    pca3.pose.orientation.w = 1.0;
    vectPlane.pose.orientation.w = 1.0;

    centroid_marker.id = 0;
    pca1.id = 1;
    pca2.id = 2;
    pca3.id = 3;
    vectPlane.id = 4;

    centroid_marker.type = visualization_msgs::Marker::SPHERE;
    pca1.type = visualization_msgs::Marker::ARROW;
    pca2.type = visualization_msgs::Marker::ARROW;
    pca3.type = visualization_msgs::Marker::ARROW;
    vectPlane.type = visualization_msgs::Marker::ARROW;

    // POINTS markers use x and y scale for width/height respectively
    centroid_marker.scale.x = 0.035;
    centroid_marker.scale.y = 0.035;
    centroid_marker.scale.z = 0.035;

    centroid_marker.color.r = 0.9f;
    centroid_marker.color.g = 0.9f;
    centroid_marker.color.b = 0.2f;
    centroid_marker.color.a = 1.0;

    
    pca1.scale.x = 0.015;     // Shaft diameter 
    pca1.scale.y = 0.035;    // Head diameter
    pca1.scale.z = 0.03;     // Head lenght
    pca1.color.b = 1.0f;
    pca1.color.a = 1.0; 

    pca2.scale.x = 0.015;
    pca2.scale.y = 0.035;
    pca2.scale.z = 0.03;
    pca2.color.g = 1.0f;
    pca2.color.a = 1.0;

    pca3.scale.x = 0.015;
    pca3.scale.y = 0.035;
    pca3.scale.z = 0.03;
    pca3.color.r = 1.0f;
    pca3.color.a = 1.0;

    vectPlane.scale.x = 0.015;
    vectPlane.scale.y = 0.035;
    vectPlane.scale.z = 0.03;
    vectPlane.color.r = 1.0f;
    vectPlane.color.g = 0.80f;
    vectPlane.color.b = 0.80f;
    vectPlane.color.a = 1.0;

    return true;
}

bool buildMarkerAxis(geometry_msgs::Vector3 PCA_axis_0,
		     geometry_msgs::Vector3 PCA_axis_1,
		     geometry_msgs::Vector3 PCA_axis_2,
		     geometry_msgs::Pose centroid_pose)
{
  float alpha = 2.5;
  pca1.points.clear();
  pca2.points.clear();
  pca3.points.clear();
  vectPlane.points.clear();
    
  geometry_msgs::Point px_centroid;
  geometry_msgs::Point p_0, p_1, p_2, pPlane;
    
  px_centroid = centroid_pose.position;
  
  p_0.x = px_centroid.x + PCA_axis_0.x;
  p_0.y = px_centroid.y + PCA_axis_0.y;
  p_0.z = px_centroid.z + PCA_axis_0.z;
    
  p_1.x = px_centroid.x + PCA_axis_1.x;
  p_1.y = px_centroid.y + PCA_axis_1.y;
  p_1.z = px_centroid.z + PCA_axis_1.z;
    
  p_2.x = px_centroid.x + PCA_axis_2.x * alpha;
  p_2.y = px_centroid.y + PCA_axis_2.y * alpha;
  p_2.z = px_centroid.z + PCA_axis_2.z * alpha;

  pca1.points.push_back( px_centroid );
  pca1.points.push_back( p_0 );
    
  pca2.points.push_back( px_centroid );
  pca2.points.push_back( p_1 );
  
  pca3.points.push_back( px_centroid );
  pca3.points.push_back(p_2 );
  
  return true;
}




int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR GRASP OBJECT BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "grasp_pln");
    ros::NodeHandle n;
    JustinaGrasp::setNodeHandle(&n);
    
    int use_angle_information;

    if(argc == 2)
      use_angle_information = atoi(argv[1]);

    
    std::vector<float> object_pose;
    std::vector<float> articular_arm;

    geometry_msgs::Pose centroid, endEffector_pose;
    geometry_msgs::Vector3 axis_resp_0, axis_resp_1, axis_resp_2;
    
    articular_arm.resize(7);

    markerSetup();
    ros::Rate loop(10);

    while(ros::ok())
    {
      std::cout << "Mode angle information: " << use_angle_information << std::endl;
      /// Detect object

      JustinaGrasp::detectObjet(object_pose, axis_resp_0, axis_resp_1, axis_resp_2);

      object_pose[0] =  0.15;
      object_pose[1] = -0.22;
      object_pose[2] =  0.90;
      
      if(use_angle_information == 0)
      {
        // // Calculate ik geometric with object-pose information
	object_pose[3] =  0.0;
	object_pose[4] =  0.0;
	object_pose[5] =  0.0;
	JustinaGrasp::ra_mark_ikCalculate(articular_arm, object_pose);
	boost::this_thread::sleep( boost::posix_time::milliseconds(4000) );
      }
      else if (use_angle_information == 1)
      {
	
	JustinaGrasp::ra_mark_ikCalculate(articular_arm, object_pose);
	boost::this_thread::sleep( boost::posix_time::milliseconds(4000) );
      }
      

      centroid.position.x = object_pose[0];
      centroid.position.y = object_pose[1];
      centroid.position.z = object_pose[2];
      
      centroid_marker.pose = centroid;
      buildMarkerAxis(axis_resp_0, axis_resp_1,
		      axis_resp_2, centroid);
	
      marker_pub.publish(centroid_marker);
      marker_pub.publish(pca1);
      marker_pub.publish(pca2);
      marker_pub.publish(pca3);
      marker_pub.publish(vectPlane);
      
      std::cout << "---------------------------" << std::endl;
      ros::spinOnce();
      loop.sleep();
    }

    
    return 0;
}
