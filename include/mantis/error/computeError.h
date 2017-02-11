/*
 * computeError.h
 *
 *  Created on: Feb 10, 2017
 *      Author: will
 */

#ifndef MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_
#define MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_


MantisRequest parseRequest(MonteCarlo* mc) {
	MantisRequest data;

	try {

		mc->tfListener.lookupTransform("bottom_camera", "base_link",
				ros::Time(0), data.c1.b2c);
	} catch (tf::TransformException& e) {
		ROS_WARN_STREAM(e.what());
	}

	try {

		mc->tfListener.lookupTransform("front_camera", "base_link",
				ros::Time(0), data.c2.b2c);
	} catch (tf::TransformException& e) {
		ROS_WARN_STREAM(e.what());
	}
	data.c1.b2c_inv = data.c1.b2c.inverse();
	data.c2.b2c_inv = data.c2.b2c.inverse();

	return data;
}

double computeError(MantisRequest& req) {
	return 0;
}

#endif /* MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_ */
