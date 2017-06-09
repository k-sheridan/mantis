/*
 * mantis2.cpp
 *
 *  Created on: Jun 8, 2017
 *      Author: Kevin Sheridan
 *
 *  This is a new method for estimating pose with respect to a grid
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

Frame quad_detect_frame;

std::vector<Hypothesis> hypotheses; // all of our best guesses as to where we are

void quadDetection(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{

	ROS_INFO("reading message");
	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	//frame1.K = get3x3FromVector(cam->K);
	//frame1.D = cv::Mat(cam->D, false);
	quad_detect_frame.cam_info_msg = *cam;
	quad_detect_frame.img_msg = *img;
	quad_detect_frame.img = temp;


	//ROS_INFO_STREAM("intrinsic; " << frame0.K);
	//ROS_INFO_STREAM("distortion; " << frame0.D);

	// detect quads in the image
	int quadCount = detectQuadrilaterals(&quad_detect_frame);

	ROS_DEBUG_STREAM(quadCount << " quads detected");
	ROS_WARN_COND(!quadCount, "no quadrilaterlals detected!");

	// determine our next guesses
	//computeHypotheses(quad_detect_frame.quads, );
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle nh;

	getParameters();

	ros::Rate loop_rate(RATE);

	image_transport::ImageTransport it(nh);

	image_transport::CameraSubscriber quadDetectCameraSub = it.subscribeCamera(QUAD_DETECT_CAMERA_TOPIC, 1, quadDetection);

	ros::spin();

	return 0;
}

