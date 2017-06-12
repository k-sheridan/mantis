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

	std::string whiteTemp;
	ros::param::param<std::string>("~whiteMap", whiteTemp, "");
	ROS_DEBUG_STREAM(whiteTemp);
	this->white_map = this->parseCoordinatesFromString(whiteTemp);
	ROS_DEBUG_STREAM("size of white map "<< this->white_map.size());

	std::string redTemp;
	ros::param::param<std::string>("~redMap", redTemp, "");
	ROS_DEBUG_STREAM(redTemp);
	this->red_map = this->parseCoordinatesFromString(redTemp);
	ROS_DEBUG_STREAM("size of red map"<< this->red_map.size());


	std::string greenTemp;
	ros::param::param<std::string>("~greenMap", greenTemp, "");
	ROS_DEBUG_STREAM(greenTemp);
	this->green_map = this->parseCoordinatesFromString(greenTemp);
	ROS_DEBUG_STREAM("size of greenMap"<< this->green_map.size());

	ros::param::param<double>("~projected_min_u", MIN_PROJECTED_U, DEFAULT_PROJECTED_MIN_PROJECTED_U);
	ros::param::param<double>("~projected_min_v", MIN_PROJECTED_V, DEFAULT_PROJECTED_MIN_PROJECTED_V);
	ros::param::param<double>("~projected_max_u", MAX_PROJECTED_U, DEFAULT_PROJECTED_MAX_PROJECTED_U);
	ros::param::param<double>("~projected_max_v", MAX_PROJECTED_V, DEFAULT_PROJECTED_MIN_PROJECTED_V);
	ros::param::param<double>("~weight_bias", WEIGHT_BIAS, DEFAULT_WEIGHT_BIAS);
}

Particle MonteCarlo::runFilter(mantis::mantisServiceRequest req)
{
	this->mantis_req = parseRequest(req); // parse the request into a usable format

	double best = 1e19;
	Particle part;
	part.trans = tf::Transform(tf::Quaternion(0, 0 , 0, 1), tf::Vector3(0, 0, 1.1));
	for(int i = 0; i < 100; i++)
	{
		cv::Mat final = mantis_req.c1.img.clone();

		Particle part2;
		part2.trans = part.trans * generateGaussianTransform(2);

		double test = computeWeight(mantis_req, part2);

		cv::imshow("server debug", projectGrid(final, part2, mantis_req));
		//cv::waitKey(30);


		if(test < best)
		{
			ROS_DEBUG_STREAM("NEW BEST! " << test);
			best = test;
			current_best = part2;
		}
	}

#if SUPER_DEBUG
	cv::Mat final = mantis_req.c1.img;

	//final = projectGrid( final, part, mantis_req);
	final = projectGrid(final, current_best, mantis_req);

	//ROS_DEBUG_STREAM()

	cv::imshow("server debug", final);
	cv::waitKey(30);
#endif

}

tf::Transform MonteCarlo::generateRandomTransform()
{
	double u1 = this->rng.uniform(0.0, 1.0);
	double u2 = this->rng.uniform(0.0, 1.0);
	double u3 = this->rng.uniform(0.0, 1.0);

	tf::Quaternion q = tf::Quaternion(sqrt(1-u1)*sin(2*CV_PI*u2),   sqrt(1-u1)*cos(2*CV_PI*u2),	 sqrt(u1)*sin(2*CV_PI*u3) ,	sqrt(u1)*cos(2*CV_PI*u3));


	tf::Vector3 t = tf::Vector3(this->rng.uniform(MINX, MAXX), this->rng.uniform(MINY,MAXY), this->rng.uniform(MINZ,MAXZ));
	return(tf::Transform(q,t));
}

tf::Transform MonteCarlo::generateGaussianTransform(double sigma)
{
	double u1 = this->rng.gaussian(sigma);
	double u2 = this->rng.gaussian(sigma);
	double u3 = this->rng.gaussian(sigma);
	double u4 = this->rng.gaussian(sigma);
	double u5 = this->rng.gaussian(sigma);
	double u6 = this->rng.gaussian(sigma);

	tf::Matrix3x3 rot;
	rot.getRPY(u1, u2, u3);

	tf::Vector3 t = tf::Vector3(this->rng.uniform(MINX, MAXX), this->rng.uniform(MINY,MAXY), this->rng.uniform(MINZ,MAXZ));

	tf::Transform tran;
	tran.setBasis(rot);
	tran.setOrigin(t);
	return tran;
}

std::vector<tf::Vector3> MonteCarlo::parseCoordinatesFromString(std::string str) {
	//sets up the row strings
	std::vector<std::string> rowStrings;
	std::stringstream textStream(str);
	std::string temp;
	std::vector<tf::Vector3> fin;

	while(std::getline(textStream, temp, ';')) {
		temp.erase(std::remove(temp.begin(), temp.end(), '\n'), temp.end()); //removes end lines
		temp.erase(std::remove(temp.begin(), temp.end(), ' '), temp.end());// removes spaces
		rowStrings.push_back(temp);
	}

	for(auto& e : rowStrings) {
		std::string rowTemp;
		tf::Vector3 transfer;
		std::stringstream rowStream(e);

		for(int hold = 0; hold < 3; hold += 1) {
			std::getline(rowStream, rowTemp, ',');
			transfer[hold] = std::atof(rowTemp.data());
			//ROS_DEBUG_STREAM("num: " << transfer[hold]);
		}
		fin.push_back(transfer);
	}

	return fin;
}

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

double colorError(cv::Vec3b actual, cv::Vec3b desired)
{
	return pow(actual[0]-desired[0], 2) + pow(actual[1]-desired[1], 2) + pow(actual[2]-desired[2], 2);
}

cv::Point2f MonteCarlo::project2d(cv::Mat& K, tf::Vector3& pt, tf::Transform& trans)
{
	//ROS_DEBUG_STREAM("projecting: " << pt.x() <<", " << pt.y() << ", " << pt.z());
	tf::Vector3 proj = (trans * pt);
	//ROS_DEBUG_STREAM("after proj: " << proj.x() <<", " << proj.y() << ", " << proj.z());
	//ROS_DEBUG_STREAM("K(0, 0): " << K.at<float>(0, 0));
	cv::Point2f out;
	out.x = ((K.at<float>(0, 0)*proj.x()) / proj.z()) + K.at<float>(0, 2);
	out.y = ((K.at<float>(1, 1)*proj.y()) / proj.z()) + K.at<float>(1, 2);

	//ROS_DEBUG_STREAM("project2d: " << out);
	return out;
}

double MonteCarlo::computeCameraError(MantisRequest& req, MantisRequest::cam& cam, tf::Transform& w2c)
{
	int pointsProjected = 0;
	double error = 0;
	for(auto e : req.white_map)
	{
		cv::Point2f proj = this->project2d(cam.K, e, w2c);

		if(proj.x > 0 && proj.x < cam.img.cols && proj.y > 0 && proj.y < cam.img.rows)
		{
			error += colorError(cam.img.at<cv::Vec3b>(proj), WHITE);
			pointsProjected++;
		}
	}

	for(auto e : req.red_map)
	{
		cv::Point2f proj = this->project2d(cam.K, e, w2c);

		if(proj.x > 0 && proj.x < cam.img.cols && proj.y > 0 && proj.y < cam.img.rows)
		{
			error += colorError(cam.img.at<cv::Vec3b>(proj), RED);
			pointsProjected++;
		}
	}

	for(auto e : req.green_map)
	{
		cv::Point2f proj = this->project2d(cam.K, e, w2c);

		if(proj.x > 0 && proj.x < cam.img.cols && proj.y > 0 && proj.y < cam.img.rows)
		{
			error += colorError(cam.img.at<cv::Vec3b>(proj), GREEN);
			pointsProjected++;
		}
	}

	if(pointsProjected < 10)
		{
			error = 1e17;
		}
	ROS_DEBUG_STREAM("error: " << error);
	return error / (double)pointsProjected;
}

/*
 * Assuming the Transform is from WORLD to BASE
 * MULTIPLY BY BASE TO CAMERA.
 * Assume we have world coordinate points.
 */
double MonteCarlo::computeWeight(MantisRequest& req, Particle& particle) {
	tf::Transform inv = particle.trans.inverse();
	tf::Transform w2c1 = inv * (req.c1.b2c_inv);
	//tf::Transform w2c2 = inv * (req.c2.b2c_inv);//When multiplied by tf vector 3 it will transform into a camera's coordinate frame

	double error = computeCameraError(req, req.c1, w2c1);

	return error;
}

MantisRequest MonteCarlo::parseRequest(mantis::mantisServiceRequest req) {
	MantisRequest data;

	data.white_map = this->white_map;
	data.red_map = this->red_map;
	data.green_map = this->green_map;

	//TODO make work with both cameras

	ROS_DEBUG_STREAM("looking for frame : " << req.image.at(0).header.frame_id);

	try {

		tfListener.lookupTransform("base_link", req.image.at(0).header.frame_id,
				ros::Time(0), data.c1.b2c);
	} catch (tf::TransformException& e) {
		ROS_WARN_STREAM(e.what());
	}

	try {

		tfListener.lookupTransform("base_link", "front_camera",
				ros::Time(0), data.c2.b2c);
	} catch (tf::TransformException& e) {
		ROS_WARN_STREAM(e.what());
	}

	data.c1.b2c_inv = data.c1.b2c.inverse();
	data.c2.b2c_inv = data.c2.b2c.inverse();

	//get the deltas
	data.delta_pos = tf::Vector3(req.delta_pos.x, req.delta_pos.y, req.delta_pos.z);
	data.delta_quat = tf::Quaternion(req.delta_quat.x, req.delta_quat.y, req.delta_quat.z, req.delta_quat.w);
	data.delta_transform = tf::Transform(data.delta_quat, data.delta_pos);

	//get the image
	cv::Mat temp = cv_bridge::toCvCopy(req.image.at(0), req.image.at(0).encoding)->image.clone();
	data.c1.K = get3x3FromVector(req.camera_info.at(0).K);
	data.c1.K_inv = data.c1.K.inv();
	data.c1.D = cv::Mat(req.camera_info.at(0).D, CV_32F);
	ROS_DEBUG_STREAM("D1: " << data.c1.D);
	cv::fisheye::undistortImage(temp, data.c1.img, data.c1.K, data.c1.D, data.c1.K);

	//ROS_DEBUG_STREAM(data.c1.img);

	return data;
}

cv::Mat MonteCarlo::projectGrid(cv::Mat src, Particle part, MantisRequest req)
{
	tf::Transform w2c = part.trans.inverse() * req.c1.b2c_inv;
	for(auto e : req.white_map)
	{
		cv::Point2f proj = this->project2d(req.c1.K, e, w2c);

		if(proj.x > 0 && proj.x < src.cols && proj.y > 0 && proj.y < src.rows)
		{
			cv::drawMarker(src, proj, cv::Scalar(255, 255, 255), cv::MARKER_SQUARE);
		}
	}

	for(auto e : req.red_map)
	{
		cv::Point2f proj = this->project2d(req.c1.K, e, w2c);

		if(proj.x > 0 && proj.x < src.cols && proj.y > 0 && proj.y < src.rows)
		{
			cv::drawMarker(src, proj, cv::Scalar(0, 0, 255), cv::MARKER_SQUARE);
		}
	}

	for(auto e : req.green_map)
	{
		cv::Point2f proj = this->project2d(req.c1.K, e, w2c);

		if(proj.x > 0 && proj.x < src.cols && proj.y > 0 && proj.y < src.rows)
		{
			cv::drawMarker(src, proj, cv::Scalar(0, 255, 0), cv::MARKER_SQUARE);
		}
	}

	//ROS_DEBUG_STREAM("recomputed error is " << computeCameraError(req, req.c1, w2c));

	return src;
}
