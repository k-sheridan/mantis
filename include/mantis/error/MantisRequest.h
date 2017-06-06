/*
 * mantisRequest.h
 *
 *  Created on: Feb 10, 2017
 *      Author: will
 */

#ifndef MANTIS_INCLUDE_MANTIS_ERROR_MANTISREQUEST_H_
#define MANTIS_INCLUDE_MANTIS_ERROR_MANTISREQUEST_H_

#include "mantis/MantisTypes.h"

struct MantisRequest {
	struct cam {
		cv::Mat img;
		cv::Mat K;
		cv::Mat D;
		tf::StampedTransform b2c; //base to camera
		tf::Transform b2c_inv; //base to camera inverted
	};

	cam c1;
	cam c2;

	tf::Vector3 delta_pos; // change in position for each particle
	tf::Quaternion delta_quat; // change in angle for each particle
	tf::Transform delta_transform; // the transformation matrix for each particle

	std::vector<tf::Vector3> white_map;
	std::vector<tf::Vector3> red_map;
	std::vector<tf::Vector3> green_map;
};




#endif /* MANTIS_INCLUDE_MANTIS_ERROR_MANTISREQUEST_H_ */
