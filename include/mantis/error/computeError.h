/*
 * computeError.h
 *
 *  Created on: Feb 10, 2017
 *      Author: will
 */

#ifndef MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_
#define MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_

#include "mantis/MantisTypes.h"

#include <mantis/mantisService.h>

cv::Mat get3x3FromVector(boost::array<double, 9> vec)
{
	cv::Mat mat = cv::Mat(3, 3, CV_32F);
	for(int i = 0; i < 3; i++)
	{
		mat.at<float>(i, 0) = vec.at(3 * i + 0);
		mat.at<float>(i, 1) = vec.at(3 * i + 1);
		mat.at<float>(i, 2) = vec.at(3 * i + 2);
	}

	ROS_DEBUG_STREAM_ONCE("K = " << mat);
	return mat;
}

MantisRequest parseRequest(MonteCarlo* mc, mantis::mantisServiceRequest req) {
	MantisRequest data;

	try {

		mc->tfListener.lookupTransform("base_link", req.image.at(0).header.frame_id,
				ros::Time(0), data.c1.b2c);
	} catch (tf::TransformException& e) {
		ROS_WARN_STREAM(e.what());
	}

	try {

		mc->tfListener.lookupTransform("base_link", "front_camera",
				ros::Time(0), data.c2.b2c);
	} catch (tf::TransformException& e) {
		ROS_WARN_STREAM(e.what());
	}
	data.c1.b2c_inv = data.c1.b2c.inverse();
	data.c2.b2c_inv = data.c2.b2c.inverse();

	return data;
}

double computeCameraWeight(MonteCarlo* mc, MantisRequest& req, MantisRequest::cam& cam, tf::Transform& w2c)
{
	int projectedPoints = 0;// n - number of point projected from camera to world
	int pointsWithinThreshold = 0;// k - number of points projected within threshold
	double sumOfDistances = 0;
	for(auto e : req.white_map)
	{

	}

	return mc->WEIGHT_BIAS * ((double)pointsWithinThreshold / projectedPoints) + (1.0 - mc->WEIGHT_BIAS) * (1 / ((sumOfDistances/pointsWithinThreshold) + 1));
}

/*
 * Assuming the Transform is from WORLD to BASE
 * MULTIPLY BY BASE TO CAMERA.
 * Assume we have world coordinate points.
 */
double computeError(MonteCarlo* mc, MantisRequest& req, Particle& particle) {
	tf::Transform w2c1 = particle.inverseTimes(req.c1.b2c_inv);
	tf::Transform w2c2 = particle.inverseTimes(req.c2.b2c_inv);//When multiplied by tf vector 3 it will transform into a camera's coordinate frame
	return 0;
}

#endif /* MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_ */
