#include <iostream>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include "justina_tools/JustinaTools.h"
#include "ros/ros.h"

#include "plane3D.hpp"
#include "objExtract.hpp"
#include "findPlaneRansac.hpp"
#include "segmentedObject.hpp"
#include "vision_msgs/DetectObjects.h"

std::ofstream myFile;

ros::ServiceClient cltRgbdRobot;
point_cloud_manager::GetRgbd srv;

bool callbackPCAobject(vision_msgs::DetectObjects::Request &req,
		       vision_msgs::DetectObjects::Response &resp)
{
	std::cout << "Calling service to calculate PCA....." << std::endl;

	vision_msgs::VisionObject objectDetected;

	std::vector<float> centroid_coord;
	std::vector<cv::Point3f> principal_axis_calculated;
	std::vector<segmentedObject> objectList;
	segmentedObject object_1;

	int xmin, ymin, H, W;
	int x_min, x_max;
	int y_min, y_max;
	int attemps;
	int points_obj;
	float x_obj, y_obj, z_obj;
	float threshold;
	float h_table;

	cv::Mat imgBGR;
	cv::Mat imgDepth;
	cv::Mat planeBGR;
	cv::Mat objectsBGR;
	cv::Mat objectsDepth;
	cv::Mat croppedDepth;
	cv::Mat croppedBRG;

	cv::Vec4f planeComp;
	cv::Point3f px;

	plane3D bestPlane;

	// *** Parametros de RANSAC *** //
	attemps = 60;		// Numero de iteraciones para RANSAC
	threshold = 0.005;	// Distancia al plano en metros

	x_min = 1000;
	y_min = 1000;

	x_max = 0;
	y_max = 0;

	x_obj = 0.0;
	y_obj = 0.0;
	z_obj = 0.0;
	h_table = 0.0;

	points_obj = 0;

	xmin = 300;
	ymin = 140;

	W = 150;
	H = 280;

	centroid_coord.push_back(0.0);
	centroid_coord.push_back(0.0);
	centroid_coord.push_back(0.0);



	if(!cltRgbdRobot.call(srv))
	{
		std::cout << "Angle_Calc.-> Cannot get point cloud.... :(  " << std::endl;
		return -1;
	}

	JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imgBGR, imgDepth);


	cv::Rect myROI(xmin, ymin, W, H);
	croppedDepth = imgDepth(myROI);
	croppedBRG = imgBGR(myROI);

	planeBGR = croppedBRG.clone();
	objectsBGR = croppedBRG.clone();


	// ##### Find best fit model to point cloud
	// Return plane with Normal = (1, 1, 1) is didn't find plane

	clock_t begin = std::clock();
	bestPlane = FindPlaneRANSAC(croppedDepth, threshold, attemps );
	clock_t end = std::clock();
	double duration = double(end-begin) / CLOCKS_PER_SEC;
	
	std::cout << "--- inliersOn - modelPlane:  " << bestPlane.inliers << std::endl; 
	std::cout << "--- Duration process: " << duration << std::endl;

	// /* Code for coloring the plane
	if(bestPlane.GetNormal() != cv::Point3f(1.0, 1.0, 1.0) )
	{
		for(int j = 0; j < planeBGR.rows; j++)
			for (int i = 0; i < planeBGR.cols; i++)
			{
			  // Calculamos la distancia de cada uno de los puntos al plan
			  px = croppedDepth.at<cv::Point3f>(j, i);
			  // Camparamos si la distancia está dentro de la tolerancia
			  if (bestPlane.DistanceToPoint(px, false) < threshold)
			    planeBGR.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 255, 0);
			}

		// ##### Return the point cloud of objects cropped
		object_1 = ExtractObj(bestPlane, croppedDepth);
		h_table = CalculateZPlane(bestPlane, croppedDepth);
	}
	else
	{
		std::cout << "I can't found the plane....   :( " << std::endl;
		objectsDepth = cv::Mat(50, 50, CV_8UC3);
	}

	objectsDepth = object_1.pointsObject;


	// Search the centroid of object PointCloud
	if(objectsDepth.size() != cv::Size(50, 50) )
	{
		object_1.getCentroid();
		object_1.getPrincipalAxis();

		//This is for response
		resp.recog_objects.push_back(objectDetected);
		resp.recog_objects[0].pose.position.x = object_1.centroid[0];
		resp.recog_objects[0].pose.position.y = object_1.centroid[1];
		resp.recog_objects[0].pose.position.z = object_1.centroid[2];

		std::cout << "   Z_prom:  " << h_table  << std::endl;
		//std::cout << "   axis[0]:  " << principal_axis_calculated[0] << "  -  norm:  " << cv::norm(principal_axis_calculated[0]) << std::endl;
		//std::cout << "   axis[1]:  " << principal_axis_calculated[1] << "  -  norm:  " << cv::norm(principal_axis_calculated[1]) << std::endl;
		//std::cout << "   axis[2]:  " << principal_axis_calculated[2] << "  -  norm:  " << cv::norm(principal_axis_calculated[2]) << std::endl;


		geometry_msgs::Vector3 q1;
		resp.recog_objects[0].principal_axis.push_back(q1);
		resp.recog_objects[0].principal_axis.push_back(q1);
		resp.recog_objects[0].principal_axis.push_back(q1);

		//This is the bigger axis
		resp.recog_objects[0].principal_axis[0].x = float(object_1.principalAxis[0].x);
		resp.recog_objects[0].principal_axis[0].y = float(object_1.principalAxis[0].y);
		resp.recog_objects[0].principal_axis[0].z = float(object_1.principalAxis[0].z);

		resp.recog_objects[0].principal_axis[1].x = float(object_1.principalAxis[1].x);
		resp.recog_objects[0].principal_axis[1].y = float(object_1.principalAxis[1].y);
		resp.recog_objects[0].principal_axis[1].z = float(object_1.principalAxis[1].z);

		resp.recog_objects[0].principal_axis[2].x = float(object_1.principalAxis[2].x);
		resp.recog_objects[0].principal_axis[2].y = float(object_1.principalAxis[2].y);
		resp.recog_objects[0].principal_axis[2].z = float(object_1.principalAxis[2].z);


	}
	else
		std::cout << "    I can't find a object on the table..... :(" << std::endl;


	myFile << "centroid: " << object_1.centroid[0] << " " << object_1.centroid[1] << " " << object_1.centroid[2] << "\n";
	std::cout << "centroid_segme: " << object_1.centroid[0] << "  " << object_1.centroid[1] << "  " << object_1.centroid[2] << std::endl;
	std::cout << "--------------------------------------" << std::endl;

	cv::rectangle(imgBGR, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
	cv::rectangle(imgDepth, cv::Point(xmin, ymin), cv::Point(xmin+W, ymin+H), cv::Scalar(0, 255, 0));
	cv::rectangle(objectsBGR, object_1.ROI, cv::Scalar(250, 10, 0));

	cv::imshow("Original RGB", imgBGR);
	cv::imshow("objects", objectsBGR);
	cv::imshow("plane 3D", planeBGR);
	cv::imshow("Objects Point Cloud", objectsDepth);
	


	/*
	//######  Code for video recorder  ######
	plane_video.write(planeBGR);
	depth_video.write(objectsDepth*255.0);
	object_video.write(objectsBGR);
	*/

	return true;
}


int main(int argc, char** argv)
{
	std::cout << "INITIALIZING OBJECT ANGLE CALCULATOR BY  EDGAR-II" << std::endl;


	// Initializing ROS node
	ros::init(argc, argv, "obj_angle_calc");
	ros::NodeHandle n;
	ros::ServiceServer srvPCAobject;
	ros::Publisher marker_pub;

	myFile.open("/home/edgar/centroid_juice.txt");
	srvPCAobject = n.advertiseService("detect_object/PCA_calculator", callbackPCAobject);
	cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");


	ros::Rate loop(10);

	while( ros::ok() && cv::waitKey(15) != 27)
	{
		// ROS
		ros::spinOnce();
		loop.sleep();

		if( cv::waitKey(5) == 'q' )
			break;
	}
	myFile.close();
	cv::destroyAllWindows();
	return 0;
}
