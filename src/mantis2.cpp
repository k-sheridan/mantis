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

Frame frame0;

void cameraCallback0(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{

	ROS_INFO("reading message");
	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	//frame1.K = get3x3FromVector(cam->K);
	//frame1.D = cv::Mat(cam->D, false);
	frame0.cam_info_msg = *cam;
	frame0.img_msg = *img;
	frame0.img = temp;


	//ROS_INFO_STREAM("intrinsic; " << frame0.K);
	//ROS_INFO_STREAM("distortion; " << frame0.D);

	int quadCount = detectQuadrilaterals(&frame0);
	ROS_DEBUG_STREAM(quadCount << " quads detected");
	ROS_WARN_COND(!quadCount, "no quadrilaterlals detected!");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle nh;

	getParameters();

	ros::Rate loop_rate(RATE);

	image_transport::ImageTransport it(nh);

	image_transport::CameraSubscriber cameraSub0 = it.subscribeCamera(CAMERA_0_TOPIC, 1, cameraCallback0);

	while(nh.ok()){

		//ROS_DEBUG("spinning once");
		ros::spinOnce();
		//ROS_DEBUG("spun once");
		loop_rate.sleep();

#if SUPER_DEBUG
		if(imgReady){
		cv::imshow("debug", final);
		cv::waitKey(30);}
#endif
	}

	return 0;
}

