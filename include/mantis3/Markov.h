/*
 * Author: Logesh Roshan Ramadoss
 */
#include <math.h>
#include <array>
#include<float.h>
#include <opencv2/plot.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

#define DEGREES 360
typedef std::array<double, 360> markovPlane;
/*
 * Returns a weighted value of gaussian distribution
 */
double calculateWeight(double x, double mu, double stddev, double y)
{
	return (y/(stddev*sqrt(2*M_PI)))*exp(-((x-mu)*(x-mu)/(2*stddev*stddev)));
}

void normalize(markovPlane& input)
{
	double sum = 0.0;
	for(int i=0; i<input.size(); ++i)
	{
		sum += input[i];
	}

	for(int i=0; i<input.size(); ++i)
	{
		input[i] = input[i] / sum;
	}
}

void normalize(markovPlane input, markovPlane& output)
{
	double sum = 0.0;
	for(int i=0; i<input.size(); ++i)
	{
		sum += input[i];
	}

	for(int i=0; i<input.size(); ++i)
	{
		output[i] = input[i] / sum;
	}

//	sum = 0.0;
//	for(int i=0; i<output.size(); ++i)
//		sum+= output[i];
//	ROS_DEBUG_STREAM("NEW SUM:" << sum);
}

void updateWeights(markovPlane& yaw, double stddev)
{
	markovPlane aux;
	double max, temp;
	int diff;
	for(int i=0; i<aux.size(); ++i)
	{
		max = DBL_MIN;
		diff = (DEGREES/2)+i;

		/*when i < 180
		 * 	Go right and find new possible weight, then go left(wrap around) and find weight.
		 *  Going left since that displacement is small
		 */
		if(i <= DEGREES/2)
		{
			for(int j=0; j<diff; ++j)
			{
				temp = calculateWeight((double)i, (double)j, stddev, yaw[j]);
				if (temp > max)
					max = temp;
			}

			for(int j=diff; j<DEGREES; ++j)
			{
				//DEGREES - j is basically looking at the array in the opposite direction (-1, -2 ....)
				temp = calculateWeight((double)i, (double)(-(DEGREES-j)), stddev, yaw[j]);
				if(temp > max)
					max = temp;
			}
		}
		// when i > 180
		else
		{
			for(int j=i; j<diff; ++j)
			{
				temp = calculateWeight((double)i, (double)j, stddev, yaw[j%DEGREES]);
				if(temp > max)
					max = temp;
			}

			for(int j=(diff%DEGREES); j<i; ++j)
			{
				temp = calculateWeight((double)i, (double)j, stddev, yaw[j]);
				if(temp > max)
					max = temp;
			}
		}

		aux[i] = max;
	}

//	ROS_DEBUG_STREAM("AUX");
//	for(int i=0; i<aux.size(); ++i)
//		std::cout<<aux[i]<<" ";
//	std::cout<<"\n";

	//normalize and copy
	normalize(aux, yaw);
}

void plotMarkovPlane(markovPlane yaw)
{
	cv::Mat data(DEGREES, 1, CV_64F);
	for(int i=0; i<DEGREES; ++i)
	{
		data.at<double>(i, 0) = yaw[i];
	}

	cv::Mat plot_result;
	cv::Ptr<cv::plot::Plot2d> plot = plot::createPlot2d(data);
	plot->setPlotBackgroundColor( Scalar( 50, 50, 50 ) ); // i think it is not implemented yet
	plot->setPlotLineColor( Scalar( 50, 50, 255 ) );
	plot->render( plot_result );

	imshow( "plot", plot_result );
	waitKey(500);

}
