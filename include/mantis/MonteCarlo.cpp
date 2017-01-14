/*
 * MonteCarlo.cpp
 *
 *  Created on: Nov 12, 2016
 *      Author: kevin
 */

#include "MonteCarlo.h"

MonteCarlo::MonteCarlo() {
	this->rng = cv::RNG(1);

	ros::param::param<float>("~minx", MINX, DEFAULT_MINX);
	ros::param::param<float>("~miny", MINY, DEFAULT_MINY);
	ros::param::param<float>("~minz", MINZ, DEFAULT_MINZ);
	ros::param::param<float>("~maxx", MAXX, DEFAULT_MAXX);
	ros::param::param<float>("~maxy", MAXY, DEFAULT_MAXY);
	ros::param::param<float>("~maxz", MAXZ, DEFAULT_MAXZ);
}

tf::Transform MonteCarlo::generateRandomTransform()
{
	tf::Quaternion q = tf::Quaternion(this->rng.uniform(0.0, 1.0),   this->rng.uniform(0.0,1.0),	 this->rng.uniform(0.0, 1.0) ,	this->rng.uniform(0.0, 1.0));
	tf::Vector3 t = tf::Vector3(this->rng.uniform(MINX, MAXX), this->rng.uniform(MINY,MAXY), this->rng.uniform(MINZ,MAXZ));
	return(tf::Transform(q,t));
}
