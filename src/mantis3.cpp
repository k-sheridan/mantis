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

#include "geometry_msgs/PoseArray.h"

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

ros::Publisher hypotheses_pub;

Frame quad_detect_frame;

std::vector<Hypothesis> hypotheses; // all of our best guesses as to where we are

geometry_msgs::PoseArray formPoseArray(std::vector<Hypothesis> hyps);

void quadDetection(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{
	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	quad_detect_frame.K = get3x3FromVector(cam->K);
	quad_detect_frame.D = cv::Mat(cam->D, false);
	quad_detect_frame.cam_info_msg = *cam;
	quad_detect_frame.img_msg = *img;
	quad_detect_frame.img = temp;

	// detect quads in the image
	int quadCount = detectQuadrilaterals(&quad_detect_frame);

	ROS_DEBUG_STREAM(quadCount << " quads detected");
	ROS_WARN_COND(!quadCount, "no quadrilaterlals detected!");

	// determine our next guesses

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mantis");

	tf_listener = new tf::TransformListener;

	ros::NodeHandle nh;

	getParameters();

	hypotheses_pub = nh.advertise<geometry_msgs::PoseArray>(HYPOTHESES_PUB_TOPIC, 1);

	ros::Rate loop_rate(RATE);

	image_transport::ImageTransport it(nh);

	image_transport::CameraSubscriber quadDetectCameraSub = it.subscribeCamera(QUAD_DETECT_CAMERA_TOPIC, 2, quadDetection);

	ros::spin();

	return 0;
}

geometry_msgs::PoseArray formPoseArray(std::vector<Hypothesis> hyps)
{
	geometry_msgs::PoseArray poses;
	for(auto& e : hyps){
		poses.poses.push_back(e.toPoseMsg(quad_detect_frame.img_msg.header.stamp, WORLD_FRAME).pose);
	}
	poses.header.frame_id = WORLD_FRAME;
	return poses;
}

