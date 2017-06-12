/*
 * CoPlanarPoseEstimator.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: pauvsi
 */

#include <mantis3/CoPlanarPoseEstimator.h>

CoPlanarPoseEstimator::CoPlanarPoseEstimator() {
	// TODO Auto-generated constructor stub

}

tf::Transform CoPlanarPoseEstimator::estimatePose(std::vector<cv::Point2d> img_pts, std::vector<tf::Vector3> obj_pts)
{
	ROS_ASSERT(img_pts.size() == obj_pts.size());
	cv::Mat obj = cv::Mat::zeros(3, obj_pts.size(), CV_64F);
	cv::Mat img = cv::Mat::zeros(3, img_pts.size(), CV_64F);

	for(int i = 0; i < img_pts.size(); i++)
	{
		img.at<double>(0, i) = img_pts.at(i).x;
		img.at<double>(1, i) = img_pts.at(i).y;
		img.at<double>(2, i) = 1.0;
	}

	for(int i = 0; i < obj_pts.size(); i++)
	{
		obj.at<double>(0, i) = obj_pts.at(i).x();
		obj.at<double>(1, i) = obj_pts.at(i).y();
		obj.at<double>(2, i) = obj_pts.at(i).z();
	}

	cv::Mat rot;
	cv::Mat tvec;
	int iterations;
	double obj_err;
	double img_err;

	ROS_DEBUG_COND(!RPP::Rpp(obj, img, rot, tvec, iterations, obj_err, img_err), "Planar Pose Estimator Failed.");

	ROS_DEBUG_STREAM("Pose Estimate: img_err: " << img_err << " obj_err: " << obj_err << " iterations: " << iterations);

	ROS_DEBUG_STREAM("estimated rot: " << rot);
	ROS_DEBUG_STREAM("estimated tvec: " << tvec);

	tf::Transform trans;

	trans.getBasis().setValue(rot.at<double>(0), rot.at<double>(1), rot.at<double>(2), rot.at<double>(3), rot.at<double>(4), rot.at<double>(5), rot.at<double>(6), rot.at<double>(7), rot.at<double>(8));
	trans.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

	return trans;

}



