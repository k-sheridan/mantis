#include "Markov.h"

//FWHM
#define FWHM_NOISE 0.84925690021
#define GAUSSIAN_WIDTH_NOISE 1.0/3.0

MarkovModel::MarkovModel(markovPlane inp)
{
	p = inp;
}

MarkovModel::MarkovModel(Hypothesis hyp)
{
	double roll, pitch, yaw;
	hyp.getW2C().getBasis().getRPY(roll, pitch, yaw);
	for(int i=0; i<p.size();++i)
		p[i] = 0.0;

	p[(int)(yaw*180.0/M_PI)] = 1;
}

double MarkovModel::calculateWeight(double x, double mu, double stddev, double y)
{
	return (y/(stddev*sqrt(2*M_PI)))*exp(-((x-mu)*(x-mu)/(2*stddev*stddev)));
}

void MarkovModel::normalize()
{
	double sum = 0.0;
	for(int i=0; i<p.size(); ++i)
	{
		sum += p[i];
	}

	for(int i=0; i<p.size(); ++i)
	{
		p[i] = p[i] / sum;
	}
}

void MarkovModel::normalize(markovPlane& input)
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

void MarkovModel::normalize(markovPlane input, markovPlane& output)
{
	double sum = 0.0;
	for(int i=0; i<input.size(); ++i)
	{
		sum += input[i];
	}

	for(int i=0; i<input.size(); ++i)
	{
		output[i] = (input[i] / sum);
	}
}

void MarkovModel::updateWeights(markovPlane& yaw, double stddev)
{
	markovPlane aux;
	double max, temp;
	int diff;
	for(int i=0; i<aux.size(); ++i)
	{
		max = 0.0;
		diff = (DEGREES/2)+i;

		/*when i < 180
		 * 	Go right and find new possible weight, then go left(wrap around) and find weight.
		 *  Going left since that displacement is small
		 */
		if(i <= DEGREES/2)
		{
			for(int j=0; j<diff; ++j)
			{
				//if(j == i) continue;
				max += calculateWeight((double)i, (double)j, stddev, yaw[j]);
			}

			for(int j=diff; j<DEGREES; ++j)
			{
				//DEGREES - j is basically looking at the array in the opposite direction (-1, -2 ....)
				max += calculateWeight((double)i, (double)(-(DEGREES-j)), stddev, yaw[j]);
			}
		}
		// when i > 180
		else
		{
			for(int j=i; j<diff; ++j)
			{
				//if(j==i) continue;
				max += calculateWeight((double)i, (double)j, stddev, yaw[j%DEGREES]);
			}

			for(int j=(diff%DEGREES); j<i; ++j)
			{
				max += calculateWeight((double)i, (double)j, stddev, yaw[j]);
			}
		}

		aux[i] = max;
	}

	//normalize and copy
	normalize(aux, yaw);

}

void MarkovModel::plotMarkovPlane()
{
	cv::Mat data(DEGREES, 1, CV_64F);
	for(int i=0; i<DEGREES; ++i)
	{
		data.at<double>(i, 0) = p[i];
	}

	cv::Mat plot_result;
	cv::Ptr<cv::plot::Plot2d> plot = plot::createPlot2d(data);
	plot->setPlotBackgroundColor( Scalar( 50, 50, 50 ) ); // i think it is not implemented yet
	plot->setPlotLineColor( Scalar( 50, 50, 255 ) );
	plot->render( plot_result );

	imshow( "plot", plot_result );
	waitKey(50);

}

void MarkovModel::plotMarkovPlane(markovPlane yaw)
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
	waitKey(50);

}

void MarkovModel::senseFusion(markovPlane sense)
{
	double newMax = sqrt(DBL_MAX);

	for(int i=0; i<p.size(); ++i)
	{
		//INCREASE PRECISION TO PREVENT IT GOES BELOW DBL_MIN.
		p[i] = p[i]*newMax;
		sense[i] = sense[i]*newMax;

	}


	for(int i=0; i<p.size(); ++i)
	{
		p[i] *= sense[i];//(history[i] + newYaw[i])/2;
	}

//	int max = 0;
//	for(int i=0; i<360; ++i)
//	{
//		if(history[i] > history[max])
//			max = i;
//	}
//	ROS_DEBUG_STREAM("MAX:"<<max);
	/*
	std::cout<<"\n";
	for(int i=0; i<360; ++i)
	{
		std::cout<<history[i]<<" ";
	}
	*/
//	for(int i=0; i<history.size(); ++i)
//		history[i] += newYaw[i];

	normalize();
	ROS_DEBUG_STREAM("merge done");
}

void MarkovModel::senseFusion(Hypothesis hypothesis)
{
	double r, p, y;
	hypothesis.getW2C().getBasis().getRPY(r,p,y);
	markovPlane sense;
	for(int i=0; sense.size(); ++i)
		sense[i] = 0.0;
	sense[(int)(y*180.0/M_PI)] = 1;

	//FWHM = 8.25. ie. +- 4.125 degrees
	updateWeights(sense, 3.5);

	senseFusion(sense);
}

void MarkovModel::updateHypothesis(std::vector<Hypothesis>& hypothesis)
{
	double roll, pitch, yaw;
	for(int i=0; i<hypothesis.size(); ++i)
	{
		hypothesis[i].getW2C().getBasis().getRPY(roll,pitch,yaw);
		//yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
		//TODO: the 1/yaw[] could be extremely large if yaw[] is extremely small. make sure it works
		hypothesis[i].error = hypothesis[i].error * 1/(p[(int)(yaw*180.0/M_PI)]);
	}
}

/*Convolves the distribution based on change in yaw and time since last  sensefusion update.
 * It adds a noise of +- 1 degree, with the probability distributed about a gaussian.
 * Parameters:
 * dTheta is in radians
 * dt is in seconds
 */
void MarkovModel::convolve(double dTheta, double dt)
{
	markovPlane aux;
	double convDisplacement = dTheta*180/M_PI;
	for(int i=convDisplacement; i<p.size()+convDisplacement; ++i)
	{
		aux[i%DEGREES] = p[i-convDisplacement];
	}

	for(int i=0; i<360; ++i)
		std::cout<<aux[i]<<" ";

	p = aux;
	std::cout<<"\n";
	for(int i=0; i<360; ++i)
		std::cout<<p[i]<<" ";

	for(int i=0; i<360; ++i)
			if(p[i] == 1) std::cout<<"\n\n"<<i<<"\n\n";



	//
	updateWeights(p, GAUSSIAN_WIDTH_NOISE*dt*11.5/30.0);
	for(int i=0; i<360; ++i)
		std::cout<<p[i]<<" ";
	//stepwise update < ONESTEP update (linearly)(1.5*(60 times) < 30*(1 time))
}


