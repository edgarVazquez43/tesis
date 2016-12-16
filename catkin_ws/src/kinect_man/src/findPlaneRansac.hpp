#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// Muetreo aleatorio de n muestras de la nube de puntos
// Devuele nos valores del punto P[x, y, z] respecto al robot
cv::Mat randomSample(int n, cv::Mat points);

// Obtenemos los puntos que se ajustan al plano definido por tres puntos
cv::Mat findPlaneConsensus(std::vector<cv::Vec3f> sample, cv::Mat points, float threshold);

// Obtenemos la ecuacion del plano que mejor se justa a los puntos 
std::vector<float> planeRANSAC(cv::Mat points);



/*
	Definicion de los metodos
*/
cv::Mat randomSample(int n, cv::Mat points)
{
	int rand_x, rand_y;
	int H = points.rows;
	int W = points.cols;
	bool isReg;
	
	std::vector<int> pixel_i;
	std::vector<int> pixel_j;
	cv::Mat sample;
	
	// Generamos numeros aleatorios
	for (int i = 0; i < n; i++)
	{
		do{
			isReg = false;
			rand_x = rand() % H;
			rand_y = rand() % W;
			// Verificamos que el punto tomado de la muestra no se haya tomado antes
			for (int ip = 0; ip < int(pixel_i.size()); ip++)
				for (int jp = 0; jp < int(pixel_j.size()); jp++)
				{
					if(rand_x == pixel_i[ip] || rand_y == pixel_j[jp])
					{
						isReg = true;
						break;
					}
				}
		}
		while(isReg);
		
		std::cout << "rand_x: " << rand_x << std::endl;
		std::cout << "rand_y: " << rand_y << std::endl;
		pixel_i.push_back(rand_x);
		pixel_j.push_back(rand_y);
	}

	for (int i = 0; i < n; i++)
	{
			sample.push_back(points.at<cv::Vec3f>(pixel_i[i], pixel_j[i]));	
	}

	return sample;
}