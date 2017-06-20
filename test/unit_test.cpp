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

#include "geometry_msgs/PoseArray.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <eigen3/Eigen/Eigen>

#include "mantis3/GridRenderer.h"

#include "mantis3/Mantis3Params.h"

#include "mantis3/Mantis3Types.h"

#include "mantis3/QuadDetection.h"

#include "mantis3/CoPlanarPoseEstimator.h"

#include "mantis3/HypothesisGeneration.h"

#include "mantis3/HypothesisEvaluation.h"

#include "mantis3/PoseClusterer.h"

#include "mantis3/PoseAdjustment.h"

#include "mantis3/PosePub.h"

ros::Publisher hypotheses_pub;

Frame quad_detect_frame;

geometry_msgs::PoseArray formPoseArray(std::vector<Hypothesis> hyps);
std::vector<tf::Quaternion> getQuatVec(std::vector<Hypothesis> hyps);


void quadDetection(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{
	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	quad_detect_frame.img.K = get3x3FromVector(cam->K);
	quad_detect_frame.img.D = cv::Mat(cam->D, false);
	quad_detect_frame.img.img = temp;
	quad_detect_frame.img.frame_id = img->header.frame_id;

	// detect quads in the image
	int quadCount = detectQuadrilaterals(&quad_detect_frame);

	ROS_DEBUG_STREAM(quadCount << " quads detected");
	ROS_WARN_COND(!quadCount, "no quadrilaterlals detected!");
	if(quadCount == 0)
		return;

	// determine our next guesses
	std::vector<Hypothesis> hyps = generateHypotheses(undistortAndNormalizeQuadTestPoints(quad_detect_frame.quads, quad_detect_frame.img.K, quad_detect_frame.img.D), quad_detect_frame.img);

	PoseClusterer pc(getQuatVec(hyps));
	//hyps = pc.clusterByAngle(MAX_ANGLE_DIFFERENCE, quadCount).removeSmallClusters().convert2Hypotheses(hyps, false);
	hyps = pc.clusterByAngle(MAX_ANGLE_DIFFERENCE, quadCount).keepLargestCluster().convert2Hypotheses(hyps, false);

	if(hyps.size() == 0)
	{
		return;
	}

	cv::Mat cleaned = cleanImageByEdge(quad_detect_frame.img.img);
	cv::Mat original = quad_detect_frame.img.img;
	quad_detect_frame.img.img = cleaned;

	//ros::Duration sleep(1);
	//sleep.sleep();
	evaluateHypotheses(hyps, quad_detect_frame.img);

	hyps = getBestNHypotheses(1, hyps);

	ROS_DEBUG("OPTIMIZING HYPO");
	Hypothesis optim = optimizeHypothesisWithParticleFilter(hyps.back(), quad_detect_frame.img);

	visualizeHypothesis(quad_detect_frame.img.img.clone(), optim, quad_detect_frame.img.K, quad_detect_frame.img.D);

	hyps = computeAllShiftedHypothesesFAST(optim);

	evaluateHypotheses(hyps, quad_detect_frame.img, true);
	hyps = getBestNHypotheses(20, hyps);

#if SUPER_DEBUG
	for(auto e : hyps)
	{
		visualizeHypothesis(quad_detect_frame.img.img.clone(), e, quad_detect_frame.img.K, quad_detect_frame.img.D);
	}
#endif

	quad_detect_frame.img.img = original;
	double min_yaw_diff = 0;
	hyps = determineBestYaw(hyps, quad_detect_frame.img, min_yaw_diff);

	visualizeHypothesis(quad_detect_frame.img.img.clone(), hyps.back(), quad_detect_frame.img.K, quad_detect_frame.img.D);

	hypotheses_pub.publish(formPoseArray(hyps));

	publishPose(hyps.back(), quad_detect_frame.img, min_yaw_diff);

	ros::Duration finalWait(0.5);
	finalWait.sleep();

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "unit_test");

	ros::NodeHandle nh;

	//load map
	getParameters();

	cv::Mat_<float> D = (cv::Mat_<float>(4, 1) << 0.0029509200248867273, -0.009944040328264236, 0.005587350111454725, -0.00205406011082232);
	cv::Mat K = (cv::Mat_<float>(3, 3) << 323.1511535644531, 0.0, 642.658203125, 0.0, 322.78955078125, 501.5538330078125, 0.0, 0.0, 1.0);

	cv::Mat test = cv::Mat::zeros(1024, 1280, CV_8UC3);

	std::vector<tf::Vector3> object_tf;
	object_tf.push_back(tf::Vector3(0.155, 0.155, 0));
	object_tf.push_back(tf::Vector3(-0.155, 0.155, 0));
	object_tf.push_back(tf::Vector3(-0.155, -0.155, 0));
	object_tf.push_back(tf::Vector3(0.155, -0.155, 0));

	std::vector<cv::Point3d> object_cv;
	object_cv.push_back(cv::Point3d(0.155, 0.155, 0));
	object_cv.push_back(cv::Point3d(-0.155, 0.155, 0));
	object_cv.push_back(cv::Point3d(-0.155, -0.155, 0));
	object_cv.push_back(cv::Point3d(0.155, -0.155, 0));

	cv::Mat_<double> rvec = (cv::Mat_<double>(3, 1) << 0.8, CV_PI, 0.6);
	cv::Mat_<double> tvec = (cv::Mat_<double>(3, 1) << 0, 0.7, 1);
	cv::Mat_<double> rot;
	cv::Rodrigues(rvec, rot);

	std::vector<cv::Point2d> img_pts;

	Hypothesis actual_hyp;
	actual_hyp.setC2W(actual_hyp.rotAndtvec2tf(rot, tvec));

	ROS_DEBUG_STREAM("actual rot: " << rot);
	ROS_DEBUG_STREAM("actual tvec: " << tvec);

	ROS_DEBUG_STREAM("actual position: " << actual_hyp.getPosition().x() << ", " << actual_hyp.getPosition().y() << ", " << actual_hyp.getPosition().z());

	img_pts.push_back(actual_hyp.distortPixel(actual_hyp.projectPoint(object_tf.at(0)), K, D));
	img_pts.push_back(actual_hyp.distortPixel(actual_hyp.projectPoint(object_tf.at(1)), K, D));
	img_pts.push_back(actual_hyp.distortPixel(actual_hyp.projectPoint(object_tf.at(2)), K, D));
	img_pts.push_back(actual_hyp.distortPixel(actual_hyp.projectPoint(object_tf.at(3)), K, D));

	cv::drawMarker(test, img_pts.at(0), cv::Scalar(255, 255, 255));
	cv::drawMarker(test, img_pts.at(1), cv::Scalar(255, 0, 0));
	cv::drawMarker(test, img_pts.at(2), cv::Scalar(0, 255, 0));
	cv::drawMarker(test, img_pts.at(3), cv::Scalar(0, 0, 255));

	cv::imshow("test", test);
	cv::waitKey(30);

	ros::Duration errorSleep(1);
	errorSleep.sleep();

	ROS_DEBUG("SOLVING FOR POSE");

	std::vector<cv::Point2d> img_pts_normal;

	cv::RNG rng(10);

	double noiseLevel = 0.01;

	img_pts_normal.push_back(actual_hyp.normalizePoint(actual_hyp.projectPoint(object_tf.at(0) + tf::Vector3(rng.gaussian(noiseLevel), rng.gaussian(noiseLevel), 0))));
	img_pts_normal.push_back(actual_hyp.normalizePoint(actual_hyp.projectPoint(object_tf.at(1) + tf::Vector3(rng.gaussian(noiseLevel), rng.gaussian(noiseLevel), 0))));
	img_pts_normal.push_back(actual_hyp.normalizePoint(actual_hyp.projectPoint(object_tf.at(2) + tf::Vector3(rng.gaussian(noiseLevel), rng.gaussian(noiseLevel), 0))));
	img_pts_normal.push_back(actual_hyp.normalizePoint(actual_hyp.projectPoint(object_tf.at(3) + tf::Vector3(rng.gaussian(noiseLevel), rng.gaussian(noiseLevel), 0))));

	CoPlanarPoseEstimator pe;

	double err;

	Hypothesis estimate;
	estimate.setC2W(pe.estimatePose(img_pts_normal, object_tf, err));

	ROS_DEBUG_STREAM("estimate position: " << estimate.getPosition().x() << ", " << estimate.getPosition().y() << ", " << estimate.getPosition().z());

	visualizeHypothesis(test, actual_hyp, K, D);

	ros::Duration sleep1(1);
	sleep1.sleep();

	visualizeHypothesis(test, estimate, K, D);

	ros::Duration sleep2(1);
	sleep2.sleep();

	GridRenderer gr(cv::Size(1024, 1280));

	gr.setColors(WHITE, GREEN, RED);
	gr.setIntrinsic(K);
	gr.setC2W(actual_hyp.getC2W());

	cv::Mat render = gr.renderGrid();

	cv::imshow("test", render);
	cv::waitKey(30);

	ros::Duration sleep3(1);
	sleep3.sleep();


	ROS_DEBUG("beggining dataset");

	hypotheses_pub = nh.advertise<geometry_msgs::PoseArray>(HYPOTHESES_PUB_TOPIC, 1);

	posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_estimate", 1);

	image_transport::ImageTransport it(nh);

	image_transport::CameraSubscriber quadDetectCameraSub = it.subscribeCamera(QUAD_DETECT_CAMERA_TOPIC, 2, quadDetection);


	ros::spin();

}


std::vector<tf::Quaternion> getQuatVec(std::vector<Hypothesis> hyps){
	std::vector<tf::Quaternion> quatVec;

	for(auto e : hyps)
	{
		quatVec.push_back(e.getQuaternion());
	}

	return quatVec;
}

geometry_msgs::PoseArray formPoseArray(std::vector<Hypothesis> hyps)
{
	geometry_msgs::PoseArray poses;
	for(auto& e : hyps){
		poses.poses.push_back(e.toPoseMsg(quad_detect_frame.img.stamp, WORLD_FRAME).pose);
	}
	poses.header.frame_id = WORLD_FRAME;
	return poses;
}

