/*
 * unit_test.cpp
 *
 *  Created on: Jun 8, 2017
 *      Author: pauvsi
 */

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include <eigen3/Eigen/Eigen>

#include "mantis2/Mantis2Params.h"

#include "mantis2/Mantis2Types.h"

#include "mantis2/QuadDetection.h"

#include "mantis2/PoseEstimator.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "unit_test");

	ros::NodeHandle nh;

	cv::Mat_<float> K = (cv::Mat_<float>(3, 3) << 300, 0, 150, 0, 300, 150, 0, 0, 1);

	//print
	ROS_INFO_STREAM("K: " << K);

	std::vector<cv::Point2d> img_pts;
	std::vector<cv::Point3d> obj_pts;

	obj_pts.push_back(cv::Point3d(0.5, 0.5, 0));
	obj_pts.push_back(cv::Point3d(-0.5, 0.5, 0));
	obj_pts.push_back(cv::Point3d(-0.5, -0.5, 0));
	obj_pts.push_back(cv::Point3d(0.5, -0.5, 0));

	img_pts.push_back(cv::Point2d(K(0)*0.5+K(2), K(4)*0.5 + K(5)));
	img_pts.push_back(cv::Point2d(K(0)*-0.5+K(2), K(4)*0.5 + K(5)));
	img_pts.push_back(cv::Point2d(K(0)*-0.5+K(2), K(4)*-0.5 + K(5)));
	img_pts.push_back(cv::Point2d(K(0)*0.5+K(2), K(4)*-0.5 + K(5)));

	Hypothesis hyp1 = computeHypothesis(img_pts, obj_pts, K);

	std::vector<cv::Point2d> img2_pts;

	cv::Mat_<double> rvec = (cv::Mat_<double>(3, 1) << 0.4, CV_PI, 1);
	cv::Mat_<double> tvec = (cv::Mat_<double>(3, 1) << 0, 0, -3);

	cv::projectPoints(obj_pts, rvec, tvec, K, cv::noArray(), img2_pts);

	cv::Mat test = cv::Mat::zeros(300, 300, CV_8UC3);
	for(auto e : img2_pts)
	{
		ROS_DEBUG_STREAM("2d point: " << e);
	}

	cv::drawMarker(test, img2_pts.at(0), cv::Scalar(255, 255, 255));
	cv::drawMarker(test, img2_pts.at(1), cv::Scalar(255, 0, 0));
	cv::drawMarker(test, img2_pts.at(2), cv::Scalar(0, 255, 0));
	cv::drawMarker(test, img2_pts.at(3), cv::Scalar(0, 0, 255));


	Hypothesis hyp2 = computeHypothesis(img2_pts, obj_pts, K);

	ROS_DEBUG_STREAM("position: " << hyp2.getPosition().x() << ", " << hyp2.getPosition().y() << ", " << hyp2.getPosition().z());

	cv::imshow("test", test);
	cv::waitKey(30);
	ros::Duration sleep(1);
	sleep.sleep();

	//try to rotate the points 90 degrees
	std::vector<cv::Point3d> obj2_pts;
	obj2_pts.push_back(cv::Point3d(0.5, -0.5, 0));
	obj2_pts.push_back(cv::Point3d(0.5, 0.5, 0));
	obj2_pts.push_back(cv::Point3d(-0.5, 0.5, 0));
	obj2_pts.push_back(cv::Point3d(-0.5, -0.5, 0));

	Hypothesis hyp3 = computeHypothesis(img2_pts, obj2_pts, K);

	ROS_DEBUG_STREAM("position: " << hyp3.getPosition().x() << ", " << hyp3.getPosition().y() << ", " << hyp3.getPosition().z());

	cv::imshow("test", test);
	cv::waitKey(30);
	ros::Duration sleep2(1);
	sleep2.sleep();

	//MORE

	ROS_DEBUG_STREAM("Possibilities: ");
	std::vector<std::vector<cv::Point3d>> possibilities = generatePossibleOrientations(1);
	for(auto e : possibilities)
	{
		ROS_DEBUG("MAP\n");
		for(auto i : e)
		{
			ROS_DEBUG_STREAM(i);
		}
	}

	ROS_DEBUG_STREAM("\n\n\n\n\nTESTING Possibilities");
	Quadrilateral quad;
	for(auto e : img2_pts)
	{
		quad.test_points.push_back(cv::Point(e.x, e.y));
	}

	std::vector<Hypothesis> hypos = computeAllCentralHypothesis(quad, possibilities, K);

	for(auto e : hypos)
	{
		//ROS_DEBUG_STREAM(e.stream());
		//std::cout << e;
		ROS_DEBUG_STREAM( "Pos: x: " << e.getPosition().x() << " y: " << e.getPosition().y() << " z: " << e.getPosition().z());
	}

	//loop till end
	while(ros::ok()){
		cv::imshow("test", test);
		cv::waitKey(30);
	}
}


