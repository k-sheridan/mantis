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

double computeCameraWeight(MonteCarlo* mc, MantisRequest& req, MantisRequest::cam& cam, tf::Transform& w2c)
{
	int projectedPoints = 0;// n - number of point projected from camera to world
	int pointsWithinThreshold = 0;// k - number of points projected within threshold
	double sumOfDistances = 0;
	for(auto e : req.white_map)
	{
		tf::Vector3 projected = w2c * e;

		if(projected.z() > 0) {

			double u = projected.x() / projected.z();
			double v = projected.y() / projected.z();

			if((u < mc->MAXX) && (u > mc->MINX) && (v < mc->MAXY) && (v > mc->MINY)) {

				projectedPoints += 1;

				std::vector<float> query;

				query.push_back(u);
				query.push_back(v);

				std::vector<int> indices;
				std::vector<double> distances;

				double thresh = mc->RADIUS_INVERSE_MULTIPLIER * (1 / projected.z());

				cam.kdtree_w.radiusSearch(query, indices, distances, thresh, 1);

				if(distances.size() != 0) {
					sumOfDistances += distances.front() / thresh;
					pointsWithinThreshold += 1;
				}
			}
		}
	}

	return mc->WEIGHT_BIAS * ((double)pointsWithinThreshold / projectedPoints) + (1.0 - mc->WEIGHT_BIAS) * (1 / ((sumOfDistances/pointsWithinThreshold) + 1));
}

/*
 * Assuming the Transform is from WORLD to BASE
 * MULTIPLY BY BASE TO CAMERA.
 * Assume we have world coordinate points.
 */
double computeError(MonteCarlo* mc, MantisRequest& req, tf::Transform& particle) {
	tf::Transform w2c1 = particle.inverseTimes(req.c1.b2c_inv);
	tf::Transform w2c2 = particle.inverseTimes(req.c2.b2c_inv);//When multiplied by tf vector 3 it will transform into a camera's coordinate frame
	return 0;
}

#endif /* MANTIS_INCLUDE_MANTIS_ERROR_COMPUTEERROR_H_ */
