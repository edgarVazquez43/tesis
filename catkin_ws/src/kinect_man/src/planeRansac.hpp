#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#define ERROR_APROX_PIX 3.0 
#define ERROR_APROX_METRIC 0.01

//Random sampling n numbers out maxn
cv::Mat randomSample(int n, cv::Mat points);

//Get the inliers in a point set over a 3D line defined by two points
std::vector<int> findPlaneConsensus(cv::Mat points, cv::Mat sample);

//Get the best line that fits a point set using RANSAC
std::vector<int> planeRANSAC(cv::Mat points);


//Random sampling n numbers out maxn
cv::Mat randomSample(int n, cv::Mat points)
{
	cv::Mat sample;
	std::vector<int> rand_x;

	int x;
	bool isReg;

	int H = points.rows;

	// Taking n samples and push into Mat
	for(int i = 0; i < n; i++ )
	{
		//Cicle for verify don't take the same sample
		do{
			isReg = false;
			x = rand() % H;	
			for(int j = 0 ;j < int(rand_x.size()); j++)
			{
				if(x == rand_x[j]){
					isReg=true;
					break;
				}
			}
		}
		while( isReg );

		std::cout << "x " << x << std::endl;
		
		rand_x.push_back(x);
	}
	
	for (int i = 0; i < n; i++){
		std::cout << "rand_x[" << i << "]: " << rand_x[i] << std::endl;
		sample.push_back(points.row(rand_x[i]));
	}

	//Return a Mat object
	return sample;
};


//Get the inliers in a point set over the line defined by two points
std::vector<int> findPlaneConsensus(cv::Mat points, cv::Mat sample)
{
	std::vector<int> consensus;

	int inliers = 0;
	int H = points.rows;

	cv::Mat p0, p1, p2;

	p1 = sample.row(0);
	p2 = sample.row(1);

	double diff = cv::norm(p2 - p1);

	double distance;
	double errorAprox = ERROR_APROX_METRIC;
	for(int i = 0; i < H; i++ )
	{
		//Get 3D point to line (defined by two 3D points) distance
		cv::Mat p0 = points.row(i);

		cv::Mat d1 = p0 - p1;
		cv::Mat d2 = p0 - p2;

		double d = cv::norm(d1.cross(d2));

		distance = d/diff;

		if( distance <=  errorAprox )
			consensus.push_back(i);
	}

	return consensus;
};

//Get the best line that fits a point set using RANSAC
std::vector<int> planeRANSAC(cv::Mat points)
{
	//Initialize random number generator 
	srand( (unsigned int) time(NULL) );

	int nloop = 0;
	int leftPoint, rightPoint;
	
	std::vector<int> best_sample;

	int myConsensus = 0, maxConsensus = 0;
	int minConsensus = (int)(0.51*points.rows); //51% data points

	double averageInliers = 0;

	bool found = false;
	while( nloop++ < 200 )
	{
		cv::Mat sample = randomSample(2, points);
		
		std::vector<int> consensus;

		if (points.cols == 3)
			consensus = findPlaneConsensus(points, sample); //3D line

		myConsensus = consensus.size();

		if( myConsensus > maxConsensus )
		{
			double xmin = 1000, xmax = -1000;
			for(int i = 0; i < myConsensus ;i++)
			{
				cv::Mat point = points.row(consensus[i]);

				//2D line extreme fitted points
				if (points.cols == 2)
				{
					if( point.at<int>(0) < xmin  )
					{
						leftPoint = consensus[i];
						xmin = point.at<int>(0);
					}
					else if( point.at<int>(0)  > xmax  )
					{
						rightPoint = consensus[i];
						xmax = point.at<int>(0);
					}
				} 
				//3D line extreme fitted points
				else if (points.cols == 3)
				{
					if( point.at<double>(0) < xmin  )
					{
						leftPoint = consensus[i];
						xmin = point.at<double>(0);
					}
					else if( point.at<double>(0)  > xmax  )
					{
						rightPoint = consensus[i];
						xmax = point.at<double>(0);
					}
				}
			}

			best_sample.clear();
			best_sample.push_back(leftPoint);
			best_sample.push_back(rightPoint);

			maxConsensus = myConsensus;
			averageInliers = (double) maxConsensus / points.rows;
		}

		if( maxConsensus >= minConsensus )
			found = true;
	}

	return best_sample;
};

