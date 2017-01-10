#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#ifndef PLANE_3D
#define PLANE_3D
#include "plane3D.hpp"
#include "findPlaneRansac.hpp"
#endif


int rectXMinD = 0, rectXMinBGR = 0;
int rectYMinD = 0, rectYMinBGR = 0;
int rectXMaxD = 1, rectXMaxBGR = 1;
int rectYMaxD = 1, rectYMaxBGR = 1;

void callback_click_depth(int event, int x, int y, int flags, void* userdata)
{
  if(event == cv::EVENT_LBUTTONDOWN)
    {
      rectXMinD = (x - 3) >= 0 ? x - 3 : 0;
      rectYMinD = (y - 3) >= 0 ? y - 3 : 0;
      rectXMaxD = (x + 3) < 640 ? x + 3 : 639;
      rectYMaxD = (y + 3) < 480 ? y + 3 : 479;
    }
}

void callback_click_BGR(int event, int x, int y, int flags, void* userdata)
{
  if(event == cv::EVENT_LBUTTONDOWN)
    {
      rectXMinBGR = (x - 3) >= 0 ? x - 3 : 0;
      rectYMinBGR = (y - 3) >= 0 ? y - 3 : 0;
      rectXMaxBGR = (x + 3) < 640 ? x + 3 : 639;
      rectYMaxBGR = (y + 3) < 480 ? y + 3 : 479;
    }
}


int main(int argc, char** argv)
{
  float x=0, y=0, z=0;
  int r=0, g=0, b=0;
  int counter = 0;

  std::cout << "Initializing kinect manager...... " << std::endl;
  ros::init(argc, argv, "kinect_man");
  ros::NodeHandle n;

  std::cout << "Kinect manager.-> Triying to initialize kinect sensor...." << std::endl;
  cv::VideoCapture capture(CV_CAP_OPENNI);
  cv::Vec3f depth;

  if(!capture.isOpened())
  {
    std::cout << "Kinect manager.-> Cannot open kinect..... :´(" << std::endl;
    return -1;
  }

  //Aligning each pixel in the image to a pixel in the depth image
  capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON);
  std::cout << "Kinect manager.-> Kinect sensor started :D" << std::endl;
  std::cout << "Kinect manager.-> Sistem ready to use" << std::endl;
  
  cv::namedWindow("Kinect depth");
  cv::setMouseCallback("Kinect depth", callback_click_depth, NULL);
  // New window for BGR image
  cv::namedWindow("Kinect BGR");
  cv::setMouseCallback("Kinect BGR", callback_click_BGR, NULL);

  cv::Mat depthMap;
  cv::Mat rgbImage;
  //std::vector<cv::Vec3f> randomSamples;
  cv::Mat randomSamples;
  cv::Mat consensus;
  cv::Vec3f point0, point1, point2;

  while( ros::ok() && cv::waitKey(15) != 27 )
  {
    r=0, g=0, b=0;
    x=0.0, y=0.0, z=0.0;
    counter=0;

    capture.grab();
    capture.retrieve(depthMap, CV_CAP_OPENNI_POINT_CLOUD_MAP);
    capture.retrieve(rgbImage, CV_CAP_OPENNI_BGR_IMAGE);


    for(int i=rectXMinD; i<=rectXMaxD; i++)
      for(int j=rectYMinD; j<=rectYMaxD; j++)
	     {
	       depth = depthMap.at<cv::Vec3f>(j, i);
	       x += depth.val[0];
	       y += depth.val[1];
	       z += depth.val[2];
	       counter++;
       }

    counter /=2;
    //std::cout << "Mean BGR: B=" << b/counter << "  G=" << g/counter << "  R=" << r/counter << std::endl;
    //std::cout << "Counter: " << counter << std::endl;
    //if (x/counter < 1000 && y/counter < 1000 && z/counter < 1000)
    //  std::cout << "Mean XYZ: X= " << x/counter << "  Y=" << y/counter << "  Z=" << z/counter << std::endl;
    

  randomSamples = randomSample(3, depthMap);
  //point0 = randomSamples.at<cv::Vec3f>(0, 0);
  //point1 = randomSamples.at<cv::Vec3f>(1, 0);
  //point2 = randomSamples.at<cv::Vec3f>(2, 0);

  cv::rectangle(depthMap, cv::Point(rectXMinD, rectYMinD), cv::Point(rectXMaxD, rectYMaxD), cv::Scalar(0, 250, 0));
  cv::rectangle(rgbImage, cv::Point(rectXMinBGR, rectYMinBGR), cv::Point(rectXMaxBGR, rectYMaxBGR), cv::Scalar(0, 0, 255));
  cv::imshow("Kinect depth", depthMap);
  cv::imshow("Kinect BGR", rgbImage);
  }
  consensus = findPlaneConsensus(randomSamples, depthMap, 0.005);
 // std::cout << "Mat_Consensus: " << consensus << std::endl;
}
