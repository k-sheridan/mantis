#include "Markov.h"

//FWHM
#define FWHM_NOISE 0.84925690021
//Total width (3*stddev)
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

	yaw *= 180.0/M_PI;
	if(yaw < 0) yaw += DEGREES;

	p[(int)(yaw)] = 1;
	updateWeights(p, 3);
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
	plot->setPlotBackgroundColor( Scalar( 50, 50, 50 ) );
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
	plot->setPlotBackgroundColor( Scalar( 50, 50, 50 ) );
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
		//INCREASE PRECISION TO PREVENT IT GOING BELOW DBL_MIN.
		p[i] = p[i]*newMax;
		sense[i] = sense[i]*newMax;

	}


	for(int i=0; i<p.size(); ++i)
	{
		p[i] *= sense[i];
	}

	normalize();
}

void MarkovModel::senseFusion(Hypothesis hypothesis)
{
	double r, p, y;
	hypothesis.getW2C().getBasis().getRPY(r,p,y);
	markovPlane sense;
	for(int i=0; sense.size(); ++i)
		sense[i] = 0.0;

	y *=180.0/M_PI;
	if(y<0) y+= DEGREES;

	sense[(int)(y)] = 1;

	//FWHM = 8.25. ie. +- 4.125 degrees
	updateWeights(sense, 3.5);

	senseFusion(sense);
}

/*
 *
 */
void MarkovModel::updateHypothesis(std::vector<Hypothesis>& hypothesis)
{
	double roll, pitch, yaw;
	for(int i=0; i<hypothesis.size(); ++i)
	{
		hypothesis[i].getW2C().getBasis().getRPY(roll,pitch,yaw);
		yaw *= 180.0/M_PI;
		if(yaw < 0) yaw += DEGREES;

		//TODO: the 1/yaw[] could be extremely large if yaw[] is extremely small. make sure it works
		hypothesis[i].error = hypothesis[i].error * 1/(p[(int)(yaw)]);
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
	int convDisplacement = (int)dTheta*180/M_PI;
	if(dTheta > 0)
	{
		for(int i=convDisplacement; i<p.size()+convDisplacement; ++i)
		{
			aux[i%DEGREES] = p[i-convDisplacement];
		}
	}
	else
	{
		convDisplacement *= -1;
		for(int i=0; i<p.size(); ++i)
		{
			aux[i] = p[(convDisplacement + i)%DEGREES];
		}
	}



	p = aux;





	//stepwise update < ONESTEP update (linearly)(1.5*(60 times) < 30*(1 time))
	updateWeights(p, GAUSSIAN_WIDTH_NOISE*dt*11.5/30.0);

}

markovPlane MarkovModel::getDistrbution()
{
	return p;
}

double MarkovModel::getYaw()
{
	int max = 0;
	for(int i=0; i<p.size();++i)
	{
		if(p[i] > p[max])
			max = i;
	}

	return (p[max] * M_PI / 180);
}

