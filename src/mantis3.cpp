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

#include "geometry_msgs/PoseArray.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <eigen3/Eigen/Eigen>

#include "mantis3/Mantis3Params.h"

#include "mantis3/Mantis3Types.h"

#include "mantis3/QuadDetection.h"

#include "mantis3/CoPlanarPoseEstimator.h"

#include "mantis3/HypothesisGeneration.h"

#include "mantis3/HypothesisEvaluation.h"

#include "mantis3/PoseClusterer.h"

#include "mantis3/PoseAdjustment.h"

#include "mantis3/PosePub.h"

#include "mantis3/Markov.cpp"
ros::Publisher hypotheses_pub;

Frame measurement;

std::vector<Hypothesis> hypotheses; // all of our best guesses as to where we are

geometry_msgs::PoseArray formPoseArray(std::vector<Hypothesis> hyps);
std::vector<tf::Quaternion> getQuatVec(std::vector<Hypothesis> hyps);

void quadDetection(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{

	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	measurement.img.K = get3x3FromVector(cam->K);
	measurement.img.D = cv::Mat(cam->D, false);
	measurement.img.img = temp;
	measurement.img.frame_id = img->header.frame_id;

	// detect quads in the image
	int quadCount = detectQuadrilaterals(&measurement);

	//ROS_DEBUG_STREAM(quadCount << " quads detected");
	ROS_WARN_COND(!quadCount, "no quadrilaterlals detected!");
	if(quadCount == 0)
		return;

	// determine our central guesses
	std::vector<Hypothesis> hyps = generateHypotheses(undistortAndNormalizeQuadTestPoints(measurement.quads, measurement.img.K, measurement.img.D), measurement.img);

	// cluster by angle
	PoseClusterer pc(getQuatVec(hyps));
	//hyps = pc.clusterByAngle(MAX_ANGLE_DIFFERENCE, quadCount).removeSmallClusters().convert2Hypotheses(hyps, false);
	hyps = pc.clusterByAngle(MAX_ANGLE_DIFFERENCE, quadCount).keepLargestCluster().convert2Hypotheses(hyps, false);

	if(hyps.size() == 0)
	{
		return;
	}

	//TODO add cases for too few hypos

	// make it less likely for false positives to occur
	cv::Mat cleaned = cleanImageByEdge(measurement.img.img);
	cv::Mat original = measurement.img.img;
	measurement.img.img = cleaned;

	// evaluate each of the hypos using only white
	evaluateHypotheses(hyps, measurement.img);

	// get the best one
	hyps = getBestNHypotheses(1, hyps);

	// optimize this hypo
	Hypothesis optim = optimizeHypothesisWithParticleFilter(hyps.back(), measurement.img);

	// get all shifts for this hypo
	hyps = computeAllShiftedHypothesesFAST(optim);


	// evaluate all shifts
	evaluateHypotheses(hyps, measurement.img, true);

	// get the top 20
	hyps = getBestNHypotheses(20, hyps);

	// eliminate the yaw ambiguity using the full image
	measurement.img.img = original;
	double min_yaw_diff = 0;
	hyps = determineBestYaw(hyps, measurement.img, min_yaw_diff);


	// publish the pose if it meets our standards
	publishPose(hyps.back(), measurement.img, min_yaw_diff);


}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "mantis");

	tf_listener = new tf::TransformListener;

	ros::NodeHandle nh;

	getParameters();

	//hypotheses_pub = nh.advertise<geometry_msgs::PoseArray>(HYPOTHESES_PUB_TOPIC, 1);
	posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("mantis/pose_estimate", 1);

	image_transport::ImageTransport it(nh);

	image_transport::CameraSubscriber quadDetectCameraSub = it.subscribeCamera(QUAD_DETECT_CAMERA_TOPIC, 2, quadDetection);

	ros::spin();

	return 0;
}


/*
// TEST MARKOV
int main(int argc, char **argv)
{
	markovPlane yaw, nyaw;
	for(int i=0; i<yaw.size(); ++i)
		{
			yaw[i] = 0.0; nyaw[i] = 0.0;
		}

	yaw[130] = 0;
	yaw[120] = 1;
	yaw[320] = 0;
	yaw[190] = 0;
	MarkovModel mod(yaw);
	double dTheta = 45*M_PI/180;
	double dT = 5;

	for(int i=0; i<50; ++i)
	{
		mod.plotMarkovPlane();
	}
	mod.convolve(dTheta, dT);

	for(int i=0; i<100; ++i)
	{
		mod.plotMarkovPlane();
	}

	nyaw[10] = 0;
	nyaw[80] = 0;
	nyaw[120] = 1;
	nyaw[330] = 0;
	nyaw[180] = 0;


//	ROS_DEBUG_STREAM("Stepwise update");
//	for(int i=0; i<60; ++i)
//	{
//		updateWeights(yaw, 1.5);
//		plotMarkovPlane(yaw);
//	}
//	for(int i=0; i<50; ++i)
//		plotMarkovPlane(yaw);
//
//	ROS_DEBUG_STREAM("ONESTEP update");
//	updateWeights(nyaw, (90.0/3)*11.5/30);
//	for(int i=0; i<100; ++i)
//		plotMarkovPlane(nyaw);
//
//
//
//	for(int i=0; i<yaw.size(); ++i)
//			{
//				yaw[i] = 0.0; nyaw[i] = 0.0;
//			}
//
//		yaw[130] = 0;
//		yaw[120] = 1;
//		yaw[320] = 0;
//		yaw[190] = 0;
//
//		nyaw[10] = 0;
//		nyaw[80] = 0;
//		nyaw[120] = 1;
//		nyaw[330] = 0;
//		nyaw[180] = 0;
//
//		ROS_DEBUG_STREAM("Stepwise update");
//		for(int i=0; i<60; ++i)
//		{
//			updateWeights(yaw, 4);
//			plotMarkovPlane(yaw);
//		}
//		for(int i=0; i<50; ++i)
//			plotMarkovPlane(yaw);
//
//		ROS_DEBUG_STREAM("ONESTEP update");
//		updateWeights(nyaw, (240.0/3)*11.5/30);
//		for(int i=0; i<100; ++i)
//			plotMarkovPlane(nyaw);


//	ROS_DEBUG_STREAM("SHOWING MODEL");
//	plotMarkovPlane(yaw);
//	updateWeights(nyaw, 30);
//	ROS_DEBUG_STREAM("SHOWING INPUT");
//	plotMarkovPlane(nyaw);
//	mergeMarkovPlanes(yaw, nyaw);
//	for(int i=0; i<100; ++i)
//		plotMarkovPlane(yaw);

//	double stddev = 0.8;
//	//IT SATURATES WITH THE MAX VALUE. maybe make a liear or sub-linear changing stddev and test?
//	for(int i=0; i<600; ++i)
//	{
//		ROS_DEBUG_STREAM(i);
//		updateWeights(yaw, stddev); stddev+= 0.001;
////		updateWeights(yaw, (i/100+1)*1.2);
////		updateWeights(yaw, (i/100+1)*1.2);
//
//		plotMarkovPlane(yaw);
//	}
	return 0;
}
*/

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
		poses.poses.push_back(e.toPoseMsg(measurement.img.stamp, WORLD_FRAME).pose);
	}
	poses.header.frame_id = WORLD_FRAME;
	return poses;
}

