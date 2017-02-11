/*
 * mantisRequest.h
 *
 *  Created on: Feb 10, 2017
 *      Author: will
 */

#ifndef MANTIS_INCLUDE_MANTIS_ERROR_MANTISREQUEST_H_
#define MANTIS_INCLUDE_MANTIS_ERROR_MANTISREQUEST_H_

struct MantisRequest {
	struct cam {
		cv::flann::KDTreeIndexParams paramIndex_w;
		cv::flann::Index kdtree_w;
		cv::flann::KDTreeIndexParams paramIndex_r;
		cv::flann::Index kdtree_r;
		cv::flann::KDTreeIndexParams paramIndex_g;
		cv::flann::Index kdtree_g;
		tf::StampedTransform b2c; //base to camera
		tf::Transform b2c_inv; //base to camera inverted
	};

	cam c1;
	cam c2;
};




#endif /* MANTIS_INCLUDE_MANTIS_ERROR_MANTISREQUEST_H_ */
