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

#include "mantis3/Mantis3Params.h"

#include "mantis3/Mantis3Types.h"

#include "mantis3/QuadDetection.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "unit_test");

	ros::NodeHandle nh;

	//load map
	getParameters();

	cv::Mat_<float> D = (cv::Mat_<float>(4, 1) << 0.0029509200248867273, -0.009944040328264236, 0.005587350111454725, -0.00205406011082232);
	cv::Mat K = (cv::Mat_<float>(3, 3) << 323.1511535644531, 0.0, 642.658203125, 0.0, 322.78955078125, 501.5538330078125, 0.0, 0.0, 1.0);

	cv::Mat test = cv::Mat::zeros(1024, 1280, CV_8UC3);

	ros::Duration errorSleep(1);
	ROS_DEBUG("TESTING GRID PROJECTION UNDISTORTED");
	errorSleep.sleep();






	//loop till end
	while(ros::ok()){
		cv::imshow("test", test);
		cv::waitKey(30);
	}
}


