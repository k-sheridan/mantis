/*
 * HypothesisEvaluation.h
 *
 *  Created on: Jun 12, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_ROBUSTPLANARPOSE_HYPOTHESISEVALUATION_H_
#define MANTIS_INCLUDE_MANTIS3_ROBUSTPLANARPOSE_HYPOTHESISEVALUATION_H_

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


#endif /* MANTIS_INCLUDE_MANTIS3_ROBUSTPLANARPOSE_HYPOTHESISEVALUATION_H_ */
