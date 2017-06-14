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

double evaluateHypothesisWithImage(Hypothesis hyp, MantisImage img)
{
	int numProjections = 0;
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

			double pt_error = computeColorError(meas, desired);

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


#endif /* MANTIS_INCLUDE_MANTIS3_ROBUSTPLANARPOSE_HYPOTHESISEVALUATION_H_ */
