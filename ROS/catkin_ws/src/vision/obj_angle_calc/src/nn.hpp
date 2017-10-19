#pragma once
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "plane3D.hpp"
#include "segmentedObject.hpp"

std::vector<int> normalizingVector(std::vector<float> analogVector);

std::vector<int> nn_calculate(std::vector<int> P);



std::vector<int> normalizingVector(std::vector<float> analogVector)
{
	std::vector<int> p;

	if (analogVector[0] > 0.04)
		p.push_back(1);
	else
		p.push_back(0);

	if (analogVector[1] > 0.06)
		p.push_back(1);
	else
		p.push_back(0);

	if (analogVector[2] > 0.115)
		p.push_back(1);
	else
		p.push_back(0);

	if (analogVector[3] > 0.6)
		p.push_back(1);
	else
		p.push_back(0);

	if (analogVector[4] < 0.45)
		p.push_back(1);
	else
		p.push_back(0);

	if (analogVector[5] > 0.045)
		p.push_back(1);
	else
		p.push_back(0);

	return p;
}


std::vector<int> nn_calculate(std::vector<int> Pt)
{
	std::vector<int> a;
	a.push_back(0);
	a.push_back(0);
	a.push_back(0);

	Eigen::ArrayXXf T(3,4);
  	Eigen::ArrayXXf P(4,6);
  	Eigen::ArrayXXf A(3,6);
  	
  	T << 0, 0, 0, 0,
  		 0, 1, 0, 1,
    	 0, 0, 1, 1;
  	
  	P << 1, 1, 0, 1, 1, 1,
       	 1, 1, 1, 1, 0, 1,
       	 1, 1, 0, 0, 0, 1,
       	 0, 0, 1, 0, 1, 1;

    A = T*P;

    return a;
}
