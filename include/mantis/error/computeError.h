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

		mc->tfListener.lookupTransform("base_link", "bottom_camera",
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

/*
 * Assuming the Transform is from WORLD to BASE
 * MULTIPLY BY BASE TO CAMERA.
 * Assume we have world coordinate points.
 */
double computeError(MantisRequest& req, tf::Transform particle) {
	tf::Transform w2c1 = particle.inverseTimes(req.c1.b2c_inv);
	tf::Transform w2c2 = particle.inverseTimes(req.c2.b2c_inv);//When multiplied by tf vector 3 it will transform into a camera's coordinate frame


	return 0;
}

#endif /* MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_ */
