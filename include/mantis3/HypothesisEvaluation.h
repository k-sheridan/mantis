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
double computePointError(cv::Point2d px, cv::Size searchArea, cv::Mat img, cv::Vec3i desired, bool);
double evaluateHypothesisWithImageWHITE(Hypothesis hyp, MantisImage img, int& numProjections, bool);
double evaluateHypothesisWithImageCOLOR(Hypothesis hyp, MantisImage img, int& numProjections, bool);
double evaluateHypothesis(Hypothesis hyp, MantisImage img, bool);
double evaluateHypothesisCOLOR(Hypothesis hyp, MantisImage img, bool fast);

cv::Mat visualizeHypothesis(cv::Mat src, Hypothesis hyp, cv::Mat K, cv::Mat D);

void evaluateOneHypothesis(Hypothesis& hyp, MantisImage img, bool fast = true){


	hyp.error = evaluateHypothesis(hyp, img, fast);


}

void evaluateHypotheses(std::vector<Hypothesis>& hyps, MantisImage img, bool fast = true){

	for(auto& e : hyps)
	{
		e.error = evaluateHypothesis(e, img, fast);
#if SUPER_DEBUG
		visualizeHypothesis(img.img.clone(), e, img.K, img.D);
#endif
	}

}

double evaluateHypothesesColor(std::vector<Hypothesis>& hyps, MantisImage img, bool fast = true){

	double totalError = 0;
	int success = 0;
	for(auto& e : hyps)
	{
		e.error = evaluateHypothesisCOLOR(e, img, fast);
		if(fabs(e.error - DBL_MAX) > 0.001)
		{
			success++;
			totalError += e.error;
		}
	}

	if(success == 0)
	{
		return DBL_MAX;
	}
	else
	{
		return totalError / (double)success;
	}

}

/*
 * evaluates the hypothesis with all images
 */
double evaluateHypothesis(Hypothesis hyp, MantisImage img, bool fast)
{
	int projections = 0;

	double error = evaluateHypothesisWithImageWHITE(hyp, img, projections, fast);

	//ROS_DEBUG_STREAM(" projections " << projections);

	if(projections <= 0)
	{
		return DBL_MAX;
	}
	else
	{
		return error / ((double)projections * PROJECTION_BIAS);
	}
}

double evaluateHypothesisCOLOR(Hypothesis hyp, MantisImage img, bool fast)
{
	int projections = 0;

	double error = evaluateHypothesisWithImageCOLOR(hyp, img, projections, fast);

	//ROS_DEBUG_STREAM(" projections " << projections);

	if(projections <= 0)
	{
		return DBL_MAX;
	}
	else
	{
		return error / ((double)projections * COLOR_PROJECTION_BIAS);
	}
}

double evaluateHypothesisWithImageWHITE(Hypothesis hyp, MantisImage img, int& numProjections, bool fast)
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
				error += computePointError(px, COLOR_SEARCH_AREA, img.img, WHITE, fast); // compute the error
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
				error += computePointError(px, COLOR_SEARCH_AREA, img.img, WHITE, fast); // compute the error
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
				error += computePointError(px, COLOR_SEARCH_AREA, img.img, WHITE, fast); // compute the error
			}
		}
	}

	return error;
}

double evaluateHypothesisWithImageCOLOR(Hypothesis hyp, MantisImage img, int& numProjections, bool fast)
{
	//TODO look up the transform or use it
	double error = 0;

	/*	for(auto e : white_map)
	{
		tf::Vector3 reproj = hyp.projectPoint(e);
		if(reproj.z() > 0)
		{
			cv::Point2d px = hyp.distortPixel(reproj, img.K, img.D);

			if(inFrame(px, img.img.rows, img.img.cols))
			{
				numProjections++; // add a projection
				error += computePointError(px, COLOR_SEARCH_AREA, img.img, WHITE, fast); // compute the error
			}
		}
	}*/
	if(RED_ONLY || RED_AND_GREEN)
	{
		for(auto e : red_map)
		{
			tf::Vector3 reproj = hyp.projectPoint(e);
			if(reproj.z() > 0)
			{
				cv::Point2d px = hyp.distortPixel(reproj, img.K, img.D);

				if(inFrame(px, img.img.rows, img.img.cols))
				{
					numProjections++; // add a projection
					error += computePointError(px, COLOR_SEARCH_AREA, img.img, RED, fast); // compute the error
				}
			}
		}
	}

	if(GREEN_ONLY || RED_AND_GREEN)
	{
		for(auto e : green_map)
		{
			tf::Vector3 reproj = hyp.projectPoint(e);
			if(reproj.z() > 0)
			{
				cv::Point2d px = hyp.distortPixel(reproj, img.K, img.D);

				if(inFrame(px, img.img.rows, img.img.cols))
				{
					numProjections++; // add a projection
					error += computePointError(px, COLOR_SEARCH_AREA, img.img, GREEN, fast); // compute the error
				}
			}
		}
	}

	return error;
}

double computePointErrorFAST(cv::Point2d px, cv::Mat img, cv::Vec3i desired)
{
	//double minError = DBL_MAX;
	double err = 0;
	cv::Vec3b meas = img.at<cv::Vec3b>(px);

	err = computeColorError(cv::Vec3i(meas[0], meas[1], meas[2]), desired);

	return err;
}

/*
 * searches a box around the pixel for the closest color
 * to the desired color
 */
double computePointError(cv::Point2d px, cv::Size searchArea, cv::Mat img, cv::Vec3i desired, bool fast)
{
	if(fast)
	{
		return computePointErrorFAST(px, img, desired);
	}

	//double minError = DBL_MAX;
	double err = 0;

	double half_x = searchArea.width/2.0;
	double half_y = searchArea.height/2.0;

	for(double x = -half_x; x < half_x; x += 1)
	{
		for(double y = -half_y; y < half_y; y += 1)
		{
			cv::Vec3b meas = img.at<cv::Vec3b>(px + cv::Point2d(x, y));

			double pt_error = computeColorError(cv::Vec3i(meas[0], meas[1], meas[2]), desired);

			/*if(pt_error < minError)
			{
				minError = pt_error;
			}*/

			err += pt_error;
		}
	}

	err /= (double)(searchArea.height * searchArea.width);

	return err;
}

double computeColorError(cv::Vec3i meas, cv::Vec3i des)
{
	int db = meas[0] - des[0];
	int dg = meas[1] - des[1];
	int dr = meas[2] - des[2];

	return (double)(db*db + dg*dg + dr*dr);
}

cv::Mat cleanImageByColor(cv::Mat img)
{
	cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8U);

	for(int i = 0; i < img.rows; i++)
	{
		for(int j = 0; j < img.cols; j++)
		{
			double err = computeColorError(img.at<cv::Vec3b>(i, j), WHITE);
			if(err < MAX_WHITE_ERROR)
			{
				mask.at<char>(i, j) = 255;
				continue;
			}
			err = computeColorError(img.at<cv::Vec3b>(i, j), RED);
			if(err < MAX_RED_ERROR)
			{
				mask.at<char>(i, j) = 255;
				continue;
			}
			err = computeColorError(img.at<cv::Vec3b>(i, j), GREEN);
			if(err < MAX_GREEN_ERROR)
			{
				mask.at<char>(i, j) = 255;
				continue;
			}

		}
	}

	//cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), MASK_DILATE);
	//cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), MASK_ERODE);
	cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3)));

	cv::Mat out;
	img.copyTo(out, mask);

	cv::GaussianBlur(out, out, cv::Size(0, 0), MASK_BLUR_SIGMA, MASK_BLUR_SIGMA);
	//out = mask;
	return out;
}

cv::Mat cleanImageByEdge(cv::Mat img)
{
	cv::Mat mono;
	//cv::fisheye::undistortImage(f->img, f->img, f->K, f->D, f->K);
	cv::cvtColor(img, mono, CV_BGR2GRAY);

	cv::Mat canny;
	cv::GaussianBlur(mono, mono, MASK_BLUR_SIZE, MASK_BLUR_SIGMA, MASK_BLUR_SIGMA);
	cv::Canny(mono, canny, CANNY_HYSTERESIS, 3 * CANNY_HYSTERESIS, 3);


	cv::imshow("test", canny);
	cv::waitKey(30);
	ros::Duration wait(1);
	wait.sleep();


	cv::Mat tmp=canny.clone();
	cv::morphologyEx(tmp,tmp,cv::MORPH_GRADIENT,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3)));
	cv::bitwise_not(tmp,tmp);
	cv::Mat smallholes = cv::Mat::zeros(tmp.size(), CV_8UC1);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(tmp,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
	for(int i = 0; i < contours.size(); i++)
	{

		double area = cv::contourArea(contours[i]);

		cv::drawContours(smallholes, contours, i, 255, 1);
	}
	cv::Mat mask;
	cv::bitwise_or(canny,smallholes,mask);

	for(int i = 0; i < MASK_CLOSE_ASCEND_ITER; i++)
	{
		cv::dilate(mask, mask, cv::Mat(), cv::Size(-1, -1), MASK_INIT_CLOSE_ITER+i, 1, 1);
#if SUPER_DEBUG
		cv::imshow("test", mask);
		cv::waitKey(30);
		ros::Duration wait1(1);
		//wait1.sleep();
#endif

		cv::erode(mask, mask, cv::Mat(), cv::Size(-1, -1), MASK_INIT_CLOSE_ITER+i, 1, 1);
#if SUPER_DEBUG
		cv::imshow("test", mask);
		cv::waitKey(30);
		ros::Duration wait2(1);
		//wait2.sleep();
#endif
	}

	cv::erode(mask, mask, cv::Mat(), cv::Size(-1, -1), MASK_ERODE_ITER, 1, 1);
#if SUPER_DEBUG
	cv::imshow("test", mask);
	cv::waitKey(30);
	ros::Duration wait3(1);
	//wait3.sleep();
#endif



	cv::Mat out;

	img.copyTo(out, mask);
	return out;
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
			cv::drawMarker(src, hyp.distortPixel(reproj_raw, K, D), WHITE);
		}
	}
	for(auto e : red_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		//ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::drawMarker(src, hyp.distortPixel(reproj_raw, K, D), RED);
		}
	}
	for(auto e : green_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		//ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::drawMarker(src, hyp.distortPixel(reproj_raw, K, D), GREEN);
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

		std::sort(hyps.begin(), hyps.end(), wayToSort);

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


std::vector<Hypothesis> determineBestYaw(std::vector<Hypothesis> hyps, MantisImage img, double min_error_diff){
	tf::Transform rotZ = tf::Transform(tf::Quaternion(0, 0, 1/sqrt(2), 1/sqrt(2)));

	std::vector<std::vector<Hypothesis> > rots;

	rots.push_back(hyps); // no rot
	for(int i = 1; i < 4; i++)
	{
		rots.push_back(rots.back());
		for(int j = 0; j < hyps.size(); j++)
		{
			Hypothesis temp = rots.at(i).at(j);
			temp.setW2C(rotZ * rots.at(i-1).at(j).getW2C());
			rots.at(i).at(j) = temp; // rotate the last rotation by 90 deg
		}
		ROS_ASSERT(rots.at(i).size() == hyps.size());
	}

	ROS_ASSERT(rots.size() == 4);

	std::vector<Hypothesis> best;
	double best_error = DBL_MAX;

	std::vector<double> errors;

	for(auto e : rots)
	{
		double tempError = evaluateHypothesesColor(e, img, false);
		ROS_DEBUG_STREAM("yaw error: " << tempError);

		if(tempError < best_error)
		{
			best_error = tempError;
			best = e;
		}
		errors.push_back(tempError);
	}

	ROS_DEBUG_STREAM("best error: " << best_error);

	double min1=DBL_MAX, min2=DBL_MAX;

	for(auto e : errors)
	{
		double diff = e - best_error;
		if(diff < min1)
		{
			min2 = min1;
			min1 = diff;
		}
		else if(diff < min2)
		{
			min2 = diff;
		}
	}

	ROS_DEBUG_STREAM("abs min: " << min1 <<" abs min 2: " << min2);
	min_error_diff = min1;

	return best;
}

std::vector<Hypothesis> getAllRotations(std::vector<Hypothesis> hyps){
	tf::Transform rotZ = tf::Transform(tf::Quaternion(0, 0, 1/sqrt(2), 1/sqrt(2)));
	std::vector<Hypothesis> out;

	for(auto e : hyps)
	{
		Hypothesis hyp;
		out.push_back(e);
		hyp.setW2C(rotZ * out.back().getW2C());
		out.push_back(hyp);
		hyp.setW2C(rotZ * out.back().getW2C());
		out.push_back(hyp);
		hyp.setW2C(rotZ * out.back().getW2C());
		out.push_back(hyp);
	}

	return out;
}




#endif /* MANTIS_INCLUDE_MANTIS3_ROBUSTPLANARPOSE_HYPOTHESISEVALUATION_H_ */
