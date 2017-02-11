/*
 * computeError.h
 *
 *  Created on: Feb 10, 2017
 *      Author: will
 */

#ifndef MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_
#define MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_

struct MantisRequest {
	struct cam {
		cv::flann:: KDTreeIndexParams paramIndex_w;
		cv::flann:: Index kdtree_w;
		cv::flann:: KDTreeIndexParams paramIndex_r;
		cv::flann:: Index kdtree_r;
		cv::flann:: KDTreeIndexParams paramIndex_g;
		cv::flann:: Index kdtree_g;
	};

	cam c1;
	cam c2;
};



double computeError(MantisRequest& req)
{
	return 0;
}



#endif /* MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_ */
