#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
  VideoCapture cap(0);
  if(!cap.isOpened())
    return -1;

  Mat edges;
  namedWindow("edges", 1);
  while(true)
  {
    Mat frame;
    cap >> frame;
    cvtColor(frame, edges, CV_BGR2GRAY);
    GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
    Canny(edges, edges, 0, 30, 3);
    imshow("edges", edges);
    if(waitKey(30) >= 0) break;
  }
  
  return 0;
}  
