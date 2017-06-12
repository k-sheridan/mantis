/*
 * Mantis3Types.h
 *
 *  Created on: Jun 11, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_MANTIS3TYPES_H_
#define MANTIS_INCLUDE_MANTIS3_MANTIS3TYPES_H_


#include <iosfwd>
#include <ostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"

/*
 * a guess about our pose
 */
struct Hypothesis{
private:
	tf::Transform c2w, w2c; // cam to/from world transform
	tf::Quaternion q;
public:
	double error;
	int observations;

	geometry_msgs::PoseStamped toPoseMsg(ros::Time stamp, std::string frame)
	{
		geometry_msgs::PoseStamped msg;

		tf::Vector3 pos = w2c.getOrigin();

		msg.pose.position.x = pos.x();
		msg.pose.position.y = pos.y();
		msg.pose.position.z = pos.z();

		msg.pose.orientation.w = q.w();
		msg.pose.orientation.x = q.x();
		msg.pose.orientation.y = q.y();
		msg.pose.orientation.z = q.z();

		msg.header.frame_id = frame;
		msg.header.stamp = stamp;

		return msg;
	}

	tf::Quaternion& getQuaternion(){return q;}
	tf::Vector3& getPosition(){return w2c.getOrigin();}

	double getDifferenceAngle(Hypothesis& other)
	{
		return this->getQuaternion().angle(other.getQuaternion());
	}

	double getDistance(Hypothesis& other)
	{
		return (this->getPosition() - other.getPosition()).length();
	}

	void setC2W(tf::Transform trans)
	{
		c2w = trans;
		w2c = c2w.inverse();
		q = w2c.getRotation();
	}

	void setW2C(tf::Transform trans)
	{
		w2c = trans;
		c2w = w2c.inverse();
		q = w2c.getRotation();
	}

	tf::Transform getW2C(){return w2c;}
	tf::Transform getC2W(){return c2w;}

	/*
	 * project point into camera frame from world
	 */
	tf::Vector3 projectPoint(tf::Vector3 in)
	{
		return c2w * in;
	}

	/*
	 * project 3d point to pixel point from world
	 * returns -1, -1 if behind camera
	 */
	cv::Point2d projectPoint(tf::Vector3 in, cv::Mat_<float> K)
	{
		tf::Vector3 reproj = projectPoint(in);

		return point2Pixel(reproj, K);
	}

	cv::Point2d projectPointNormal(tf::Vector3 in)
	{
		tf::Vector3 reproj = projectPoint(in);

		return normalizePoint(reproj);
	}

	cv::Point2d normalizePoint(tf::Vector3 pt)
	{
		return cv::Point2d((pt.x()/pt.z()), (pt.y()/pt.z()));
	}

	cv::Point2d point2Pixel(tf::Vector3 pt, cv::Mat_<float> K)
	{
		return cv::Point2d(K(0)*(pt.x()/pt.z()) + K(2), K(4)*(pt.y()/pt.z()) + K(5));
	}

	cv::Point2d distortNormalPixel(cv::Point2d px, cv::Mat K, cv::Mat D)
	{
		std::vector<cv::Point2d> undist, dist;
		undist.push_back(px);
		cv::fisheye::distortPoints(undist, dist, K, D);
		return dist.front();
	}

	cv::Point2d distortPixel(tf::Vector3 px, cv::Mat K, cv::Mat D)
	{
		return distortNormalPixel(normalizePoint(px), K, D);
	}

	inline tf::Transform rotAndtvec2tf(cv::Mat_<double> rot, cv::Mat_<double> tvec)
	{
		tf::Transform trans;

		trans.getBasis().setValue(rot(0), rot(1), rot(2), rot(3), rot(4), rot(5), rot(6), rot(7), rot(8));
		trans.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

		return trans;
	}

	inline tf::Transform rvecAndtvec2tf(cv::Mat_<double> rvec, cv::Mat_<double> tvec)
	{
		tf::Transform trans;

		cv::Mat_<double> rot;
		cv::Rodrigues(rvec, rot);

		trans.getBasis().setValue(rot(0), rot(1), rot(2), rot(3), rot(4), rot(5), rot(6), rot(7), rot(8));
		trans.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

		return trans;
	}

};

std::ostream& operator<< (std::ostream& o, const Hypothesis& hyp)
{
	//return o << "Pos: x: " << hyp.getPosition().x() << " y: " << hyp.getPosition().y() << " z: " << hyp.getPosition().z() << std::endl;
	return o << "test";
}


struct MantisImage{
	cv::Mat img;
	cv::Mat K;
	cv::Mat D;

	MantisImage(){

	}

	MantisImage(cv::Mat img, cv::Mat K, cv::Mat D)
	{
		this->K = K;
		this->D = D;
		this->img = img;
	}
};


#endif /* MANTIS_INCLUDE_MANTIS3_MANTIS3TYPES_H_ */
