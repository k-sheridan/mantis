/*
 * Mantis2Error.h
 *
 *  Created on: Jun 9, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS2_MANTIS2ERROR_H_
#define MANTIS_INCLUDE_MANTIS2_MANTIS2ERROR_H_

#include <float.h>

void evaluateHypothesis(Hypothesis& hyp, Frame frame);
double computeColorError(cv::Vec3i meas, cv::Vec3i des);
double computePointError(Hypothesis hyp, cv::Mat img, cv::Mat K, cv::Mat D, tf::Vector3 point, cv::Vec3i color, bool& in_frame);
double computeImageError(Hypothesis hyp, cv::Mat img, cv::Mat K, cv::Mat D, std::string tf_frame, int& projected);
bool inFrame(cv::Point2d px, int rows, int cols);
cv::Mat visualizeHypothesis(cv::Mat src, Hypothesis hyp, cv::Mat K, cv::Mat D);

/*
 * compute the new likelihoods of each hypothesis
 */
void evaluateHypotheses(std::vector<Hypothesis>& hyps, Frame frame){
	for(auto& e : hyps)
	{
		//ROS_DEBUG_STREAM("evaluating hypothesis");
		evaluateHypothesis(e, frame);
		//ROS_DEBUG_STREAM("hypothesis error: " << e.error);
	}
}

void evaluateHypothesis(Hypothesis& hyp, Frame frame)
{
	int projected = 0;
	double error = computeImageError(hyp, frame.img, frame.K, frame.D, frame.img_msg.header.frame_id, projected);

	//ROS_DEBUG_STREAM("computed error: " << error);
	//ROS_DEBUG_STREAM("num obs: " << hyp.observations);
	//ROS_DEBUG_STREAM("projected: " << projected);

	if(projected == 0)
	{
		hyp.error = DBL_MAX;
	}
	else
	{
		//hyp.error = (error / (double)projected) / (double)hyp.observations;
		hyp.error = error / (double)projected;
	}

	//visualizeHypothesis(frame.img.clone(), hyp, frame.K, frame.D);

}

double computeImageError(Hypothesis hyp, cv::Mat img, cv::Mat K, cv::Mat D, std::string tf_frame, int& projected)
{
	//get the transformation from the base to the camera
	tf::StampedTransform b2c;
	try {
		tf_listener->lookupTransform(BASE_FRAME, tf_frame,
				ros::Time(0), b2c);
	} catch (tf::TransformException& e) {
		ROS_WARN_STREAM(e.what());
	}
	//ROS_DEBUG_STREAM("got tf transform");

	Hypothesis new_hyp;
	new_hyp.setW2C(hyp.getW2C() * b2c);

	//ROS_DEBUG_STREAM("got hyp of camera");

	double error = 0;
	//projected = 0;

	for(auto e : white_map)
	{
		bool in_frame = false;
		error += computePointError(new_hyp, img, K, D, e, WHITE, in_frame);
		if(in_frame){
			projected++;
		}
	}
	for(auto e : red_map)
	{
		bool in_frame = false;
		error += computePointError(new_hyp, img, K, D, e, WHITE, in_frame);
		if(in_frame){
			projected++;
		}
	}
	for(auto e : green_map)
	{
		bool in_frame = false;
		error += computePointError(new_hyp, img, K, D, e, WHITE, in_frame);
		if(in_frame){
			projected++;
		}
	}

	return error;
}

double computePointError(Hypothesis hyp, cv::Mat img, cv::Mat K, cv::Mat D, tf::Vector3 point, cv::Vec3i color, bool& in_frame)
{
	std::vector<cv::Point2f> undist, dist;
	tf::Vector3 reproj = hyp.projectPoint(point);

	//ROS_DEBUG_STREAM("projected point: " << reproj.x() << ", " << reproj.y() << ", " << reproj.z());

	if(reproj.z() <= 0)
	{
		in_frame = false;
		return 0;
	}

	cv::Point2d reproj_d = hyp.normalizePoint(reproj);

	undist.push_back(cv::Point2f(reproj_d.x, reproj_d.y));

	//ROS_DEBUG_STREAM("undistort: " << undist.at(0));

	cv::fisheye::distortPoints(undist, dist, K, D);

	//check if in frame and report
	if(!inFrame(dist.at(0), img.rows, img.cols))
	{
		in_frame = false;
		return 0;
	}
	else
	{
		in_frame = true;
	}


	//ROS_DEBUG_STREAM("distort: " << dist.at(0));

	cv::Vec3b meas = img.at<cv::Vec3b>(dist.at(0));

	return computeColorError(cv::Vec3i(meas[0], meas[1], meas[2]), color);
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

cv::Mat visualizeHypothesis(cv::Mat src, Hypothesis hyp, cv::Mat K, cv::Mat D)
{
	for(auto e : white_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::Point2d reproj_d = hyp.normalizePoint(reproj_raw);
			std::vector<cv::Point2f> dist, undist;
			undist.push_back(cv::Point2f(reproj_d.x, reproj_d.y));

			ROS_DEBUG_STREAM("undistort: " << undist.at(0));

			cv::fisheye::distortPoints(undist, dist, K, D);

			ROS_DEBUG_STREAM("distort: " << dist.at(0));

			cv::drawMarker(src, dist.at(0), cv::Scalar(255, 255, 255));
		}
	}
	for(auto e : red_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::Point2d reproj_d = hyp.normalizePoint(reproj_raw);
			std::vector<cv::Point2f> dist, undist;
			undist.push_back(cv::Point2f(reproj_d.x, reproj_d.y));

			ROS_DEBUG_STREAM("undistort: " << undist.at(0));

			cv::fisheye::distortPoints(undist, dist, K, D);

			ROS_DEBUG_STREAM("distort: " << dist.at(0));

			cv::drawMarker(src, dist.at(0), cv::Scalar(0, 0, 255));
		}
	}
	for(auto e : green_map)
	{
		tf::Vector3 reproj_raw = hyp.projectPoint(e);
		ROS_DEBUG_STREAM("test projected point: " << reproj_raw.x() << ", " << reproj_raw.y() << ", " << reproj_raw.z());
		if(reproj_raw.z() > 0)
		{
			cv::Point2d reproj_d = hyp.normalizePoint(reproj_raw);
			std::vector<cv::Point2f> dist, undist;
			undist.push_back(cv::Point2f(reproj_d.x, reproj_d.y));


			ROS_DEBUG_STREAM("undistort: " << undist.at(0));
			cv::fisheye::distortPoints(undist, dist, K, D);

			ROS_DEBUG_STREAM("distort: " << dist.at(0));

			cv::drawMarker(src, dist.at(0), cv::Scalar(0, 255, 0));
		}
	}


	cv::imshow("grid", src);
	cv::waitKey(30);

	//ros::Duration sleep(1);
	//sleep.sleep();

	return src;
}


#endif /* MANTIS_INCLUDE_MANTIS2_MANTIS2ERROR_H_ */
