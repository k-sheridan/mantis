/*
 * HypothesisEvaluation.h
 *
 *  Created on: Jun 12, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_ROBUSTPLANARPOSE_HYPOTHESISEVALUATION_H_
#define MANTIS_INCLUDE_MANTIS3_ROBUSTPLANARPOSE_HYPOTHESISEVALUATION_H_

#include <float.h>

bool inFrame(cv::Point2d px, int rows, int cols);
double computeColorError(cv::Vec3i meas, cv::Vec3i des);
double computePointError(cv::Point2d px, cv::Size searchArea, cv::Mat img, cv::Vec3i desired);
double evaluateHypothesisWithImage(Hypothesis hyp, MantisImage img, int& numProjections);
double evaluateHypothesis(Hypothesis hyp, MantisImage img);


void evaluateHypotheses(std::vector<Hypothesis>& hyps, MantisImage img){

	for(auto& e : hyps)
	{
		e.error = evaluateHypothesis(e, img);
	}

}

/*
 * evaluates the hypothesis with all images
 */
double evaluateHypothesis(Hypothesis hyp, MantisImage img)
{
	int projections = 0;

	double error = evaluateHypothesisWithImage(hyp, img, projections);

	//ROS_DEBUG_STREAM(" projections " << projections);

	if(projections <= 0)
	{
		return DBL_MAX;
	}
	else
	{
		return error / (double)projections;
	}
}

double evaluateHypothesisWithImage(Hypothesis hyp, MantisImage img, int& numProjections)
{
	//TODO look up the transform or use it
	double error = 0;

	for(auto e : white_map)
	{
		tf::Vector3 reproj = hyp.projectPoint(e);
		if(reproj.z() > 0)
		{
			cv::Point2d px = hyp.distortPixel(reproj, img.K, img.D);

			if(inFrame(px, img.img.rows, img.img.cols))
			{
				numProjections++; // add a projection
				error += computePointError(px, COLOR_SEARCH_AREA, img.img, WHITE); // compute the error
			}
		}
	}

	for(auto e : red_map)
	{
		tf::Vector3 reproj = hyp.projectPoint(e);
		if(reproj.z() > 0)
		{
			cv::Point2d px = hyp.distortPixel(reproj, img.K, img.D);

			if(inFrame(px, img.img.rows, img.img.cols))
			{
				numProjections++; // add a projection
				error += computePointError(px, COLOR_SEARCH_AREA, img.img, RED); // compute the error
			}
		}
	}

	for(auto e : green_map)
	{
		tf::Vector3 reproj = hyp.projectPoint(e);
		if(reproj.z() > 0)
		{
			cv::Point2d px = hyp.distortPixel(reproj, img.K, img.D);

			if(inFrame(px, img.img.rows, img.img.cols))
			{
				numProjections++; // add a projection
				error += computePointError(px, COLOR_SEARCH_AREA, img.img, GREEN); // compute the error
			}
		}
	}

	return error;
}

/*
 * searches a box around the pixel for the closest color
 * to the desired color
 */
double computePointError(cv::Point2d px, cv::Size searchArea, cv::Mat img, cv::Vec3i desired)
{
	double minError = DBL_MAX;

	double half_x = searchArea.width/2.0;
	double half_y = searchArea.height/2.0;

	for(double x = -half_x; x < half_x; x += 1)
	{
		for(double y = -half_y; y < half_y; y += 1)
		{
			cv::Vec3b meas = img.at<cv::Vec3b>(px + cv::Point2d(x, y));

			double pt_error = computeColorError(cv::Vec3i(meas[0], meas[1], meas[2]), desired);

			if(pt_error < minError)
			{
				minError = pt_error;
			}
		}
	}

	return minError;
}

double computeColorError(cv::Vec3i meas, cv::Vec3i des)
{
	int db = meas[0] - des[0];
	int dg = meas[1] - des[1];
	int dr = meas[2] - des[2];

	return (double)(db*db + dg*dg + dr*dr);
}

bool inFrame(cv::Point2d px, int rows, int cols)
{
	if(px.x < cols && px.y < rows && px.x >= 0 && px.y > 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}


cv::Mat visualizeHypothesis(cv::Mat src, Hypothesis hyp, cv::Mat K, cv::Mat D)
{
	for(auto e : white_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		//ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::drawMarker(src, hyp.distortPixel(reproj_raw, K, D), cv::Scalar(255, 255, 255));
		}
	}
	for(auto e : red_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		//ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::drawMarker(src, hyp.distortPixel(reproj_raw, K, D), cv::Scalar(0, 0, 255));
		}
	}
	for(auto e : green_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		//ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::drawMarker(src, hyp.distortPixel(reproj_raw, K, D), cv::Scalar(0, 255, 0));
		}
	}

	cv::imshow("test", src);
	cv::waitKey(30);

	//ros::Duration sleep(1);
	//sleep.sleep();

	return src;
}

cv::Mat visualizeHypothesis(cv::Mat src, Hypothesis hyp, Quadrilateral quad, cv::Mat K, cv::Mat D)
{
	for(auto e : white_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		//ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::drawMarker(src, hyp.distortPixel(reproj_raw, K, D), cv::Scalar(255, 255, 255));
		}
	}
	for(auto e : red_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		//ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::drawMarker(src, hyp.distortPixel(reproj_raw, K, D), cv::Scalar(0, 0, 255));
		}
	}
	for(auto e : green_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		//ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::drawMarker(src, hyp.distortPixel(reproj_raw, K, D), cv::Scalar(0, 255, 0));
		}
	}

	for(auto e : quad.test_points)
	{
		cv::drawMarker(src, hyp.distortNormalPixel(e, K, D), cv::Scalar(255, 0, 0), cv::MARKER_SQUARE);
	}

	cv::imshow("test", src);
	cv::waitKey(30);

	//ros::Duration sleep(1);
	//sleep.sleep();

	return src;
}

static bool wayToSort(Hypothesis i, Hypothesis j)
{
	//bool v = i.quality<j.quality;
	return j.error<i.error;
}

std::vector<Hypothesis> getBestNHypotheses(int n, std::vector<Hypothesis> hyps){

	if(hyps.size() <= n)
	{
		return hyps;
	}
	else
	{
		std::vector<Hypothesis> new_hyps;

		std::nth_element(hyps.begin(), hyps.begin() + hyps.size() - n, hyps.end(), wayToSort);

		for(int i = hyps.size() - n; i < hyps.size(); ++i)
		{
			new_hyps.push_back(hyps.at(i));
		}

#if SUPER_DEBUG
		ROS_DEBUG("BEST HYPOTHESES");
		for(auto e : new_hyps)
		{
			ROS_DEBUG_STREAM("error: " << e.error);
		}
#endif

		return new_hyps;
	}

}


#endif /* MANTIS_INCLUDE_MANTIS3_ROBUSTPLANARPOSE_HYPOTHESISEVALUATION_H_ */
