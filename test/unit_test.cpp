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

#include "mantis2/Mantis2Error.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "unit_test");

	ros::NodeHandle nh;

	double error;

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

	Hypothesis hyp1 = computeHypothesis(img_pts, obj_pts, K, error);

	std::vector<cv::Point2d> img2_pts;

	cv::Mat_<double> rvec = (cv::Mat_<double>(3, 1) << -1, CV_PI, 0);
	cv::Mat_<double> tvec = (cv::Mat_<double>(3, 1) << 0, 0.6, 3);

	ROS_DEBUG_STREAM("ACTUAL rvec: " << rvec << " tvec: " << tvec);

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


	Hypothesis hyp2 = computeHypothesis(img2_pts, obj_pts, K, error);

	ROS_DEBUG_STREAM("position: " << hyp2.getPosition().x() << ", " << hyp2.getPosition().y() << ", " << hyp2.getPosition().z());

	cv::imshow("actual", test);
	cv::waitKey(30);
	ros::Duration sleep(1);
	sleep.sleep();

	//TEST THE ERROR
	cv::Mat rvec_test;
	cv::Mat tvec_test;
	cv::solvePnP(obj_pts, img2_pts, K, cv::noArray(), rvec_test, tvec_test, false, POSE_SOLVE_METHOD);
	std::vector<cv::Point2d> reprojTest;
	cv::projectPoints(obj_pts, rvec_test, tvec_test, K, cv::noArray(), reprojTest);
	test = cv::Mat::zeros(300, 300, CV_8UC3);
	cv::drawMarker(test, reprojTest.at(0), cv::Scalar(255, 255, 255));
	cv::drawMarker(test, reprojTest.at(1), cv::Scalar(255, 0, 0));
	cv::drawMarker(test, reprojTest.at(2), cv::Scalar(0, 255, 0));
	cv::drawMarker(test, reprojTest.at(3), cv::Scalar(0, 0, 255));
	cv::imshow("reproj", test);
	cv::waitKey(30);
	ros::Duration sleepers(5);

	error = 0;
	for(int i = 0; i < reprojTest.size(); i++)
	{
		error += (reprojTest.at(i)-img2_pts.at(i)).ddot((reprojTest.at(i)-img2_pts.at(i)));
	}
	ROS_DEBUG_STREAM("ERROR: " << sqrt(error));
	sleepers.sleep();

	/*//try to rotate the points 90 degrees
	std::vector<cv::Point3d> obj2_pts;
	obj2_pts.push_back(cv::Point3d(0.5, -0.5, 0));
	obj2_pts.push_back(cv::Point3d(0.5, 0.5, 0));
	obj2_pts.push_back(cv::Point3d(-0.5, 0.5, 0));
	obj2_pts.push_back(cv::Point3d(-0.5, -0.5, 0));

	Hypothesis hyp3 = computeHypothesis(img2_pts, obj2_pts, K);

	ROS_DEBUG_STREAM("hyp3 position: " << hyp3.getPosition().x() << ", " << hyp3.getPosition().y() << ", " << hyp3.getPosition().z());

	cv::imshow("test", test);
	cv::waitKey(30);
	ros::Duration sleep2(1);
	sleep2.sleep();*/

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
		quad.test_points.push_back(cv::Point2d(e.x, e.y));
	}

	//std::vector<Hypothesis> hypos = computeAllCentralHypothesis(quad, possibilities, K);
	std::vector<Hypothesis> hypos = computeAllCentralHypothesisFAST(quad, obj_pts, K);


	//test using tf
	tf::Vector3 pt1(0.5, 0.5, 0);
	tf::Vector3 pt2(-0.5, 0.5, 0);
	tf::Vector3 pt3(-0.5, -0.5, 0);
	tf::Vector3 pt4(0.5, -0.5, 0);

	for(auto e : hypos)
	{
		ROS_DEBUG_STREAM( "Pos: x: " << e.getPosition().x() << " y: " << e.getPosition().y() << " z: " << e.getPosition().z());
	}
	/*for(auto e : hypos)
	{
		//ROS_DEBUG_STREAM(e.stream());
		//std::cout << e;
		ROS_DEBUG_STREAM( "Pos: x: " << e.getPosition().x() << " y: " << e.getPosition().y() << " z: " << e.getPosition().z());
		test = cv::Mat::zeros(300, 300, CV_8UC3);


		cv::drawMarker(test, e.projectPoint(pt1, K), cv::Scalar(255, 255, 255));
		cv::drawMarker(test, e.projectPoint(pt2, K), cv::Scalar(255, 0, 0));
		cv::drawMarker(test, e.projectPoint(pt3, K), cv::Scalar(0, 255, 0));
		cv::drawMarker(test, e.projectPoint(pt4, K), cv::Scalar(0, 0, 255));

		cv::imshow("test", test);
		cv::waitKey(30);

		ros::Duration sleep3(2);
		sleep3.sleep();


	}*/

	//test the error functions

	//load map
	getParameters();

	cv::Mat_<float> D = (cv::Mat_<float>(4, 1) << 0.0029509200248867273, -0.009944040328264236, 0.005587350111454725, -0.00205406011082232);
	K = (cv::Mat_<float>(3, 3) << 323.1511535644531, 0.0, 642.658203125, 0.0, 322.78955078125, 501.5538330078125, 0.0, 0.0, 1.0);

	cv::Mat blank = cv::Mat::zeros(1024, 1280, CV_8UC3);

	ros::Duration errorSleep(1);
	ROS_DEBUG("TESTING GRID PROJECTION UNDISTORTED");
	errorSleep.sleep();

	visualizeHypothesis(blank.clone(), hyp2, K, D);


	//loop till end
	while(ros::ok()){
		cv::imshow("test", test);
		cv::waitKey(30);
	}
}


