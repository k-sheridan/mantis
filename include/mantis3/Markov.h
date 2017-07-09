/*
 * Author: Logesh Roshan Ramadoss
 */
#include <math.h>
#include <array>
#include<float.h>
#include <opencv2/plot.hpp>
#include <opencv2/highgui.hpp>
#include "Mantis3Types.h"
using namespace cv;

#define DEGREES 360
typedef std::array<double, 360> markovPlane;

/*
 * TODO: Make sure yaw coming from hypothesis is correct
 */





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
//		{
//			sum+= output[i];
//			std::cout<<output[i]<<" ";
//		}
//	std::cout<<"\n";
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

	//normalize and copy
	normalize(aux, yaw);
}

//
///*
// * The issue with this:
// * 	The distribution tends to get flatter and flatter as time passes. However,
// * 		there's always one peak that rises early. This peak will keep growing taller,
// * 		the difference between this peak and everyother peak keeps increasing. So the model essentially keeps getting more sure about this peak when it actually shouldnt.
// * 		This is because it adds its own gaussian distribution onto itself. ie, adding the largest value of distribution to the highest peak.
// */
//void updateWeights(markovPlane& yaw, double stddev)
//{
//	markovPlane aux;
//	double max, temp;
//	int diff;
//	for(int i=0; i<aux.size(); ++i)
//	{
//		max = 0.0;
//		diff = (DEGREES/2)+i;
//
//		/*when i < 180
//		 * 	Go right and find new possible weight, then go left(wrap around) and find weight.
//		 *  Going left since that displacement is small
//		 */
//		if(i <= DEGREES/2)
//		{
//			for(int j=0; j<diff; ++j)
//			{
//				//if(j == i) continue;
//				max += calculateWeight((double)i, (double)j, stddev, yaw[j]);
//			}
//
//			for(int j=diff; j<DEGREES; ++j)
//			{
//				//DEGREES - j is basically looking at the array in the opposite direction (-1, -2 ....)
//				max += calculateWeight((double)i, (double)(-(DEGREES-j)), stddev, yaw[j]);
//			}
//		}
//		// when i > 180
//		else
//		{
//			for(int j=i; j<diff; ++j)
//			{
//				//if(j==i) continue;
//				max += calculateWeight((double)i, (double)j, stddev, yaw[j%DEGREES]);
//			}
//
//			for(int j=(diff%DEGREES); j<i; ++j)
//			{
//				max += calculateWeight((double)i, (double)j, stddev, yaw[j]);
//			}
//		}
//
//		aux[i] = max;
//	}
//
//	//normalize and copy
//	normalize(aux, yaw);
//}

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
	waitKey(5000);

}

void mergeMarkovPlanes(markovPlane& history, markovPlane newYaw)
{
	//MULTIPLYING DOES NOT WORK. DROPS BELOW DBL_MIN
	//maybe scale new value and add?
	for(int i=0; i<history.size(); ++i)
	{
		std::cout<<history[i]<<" ";
		history[i] *= newYaw[i];
	}
	/*
	std::cout<<"\n";
	for(int i=0; i<360; ++i)
	{
		std::cout<<history[i]<<" ";
	}
	*/
//	for(int i=0; i<history.size(); ++i)
//		history[i] += newYaw[i];

	normalize(history);
	ROS_DEBUG_STREAM("merge done");
}

void updateHypothesis(markovPlane yaw, std::vector<Hypothesis>& hypothesis)
{
	tf::Quaternion q;
	float hypothesisYaw;
	for(int i=0; i<hypothesis.size(); ++i)
	{
		q = hypothesis[i].getQuaternion();
		//yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
		hypothesisYaw   = atan2(2.0 * (q.getZ() * q.getW() + q.getX() * q.getY()) , - 1.0 + 2.0 * (q.getW() * q.getW() + q.getX() * q.getX()));
		//TODO: the 1/yaw[] could be extremely large if yaw[] is extremely small. make sure it works
		hypothesis[i].error = hypothesis[i].error * 1/(yaw[(int)hypothesisYaw]);
	}
}

