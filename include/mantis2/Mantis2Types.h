/*
 * Mantis2Types.h
 *
 *  Created on: Jun 8, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS2_MANTIS2TYPES_H_
#define MANTIS_INCLUDE_MANTIS2_MANTIS2TYPES_H_

#include <iosfwd>
#include <ostream>

/*
 * a guess about our pose
 */
struct Hypothesis{
private:
	tf::Transform c2w, w2c; // cam to/from world transform
	tf::Quaternion q;
public:
	double likelihood;
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
	 */
	cv::Point2d projectPoint(tf::Vector3 in, cv::Mat_<float> K)
	{
		tf::Vector3 reproj = projectPoint(in);

		return cv::Point2d(K(0)*(reproj.x()/reproj.z()) + K(2), K(4)*(reproj.y()/reproj.z()) + K(5));
	}

};

std::ostream& operator<< (std::ostream& o, const Hypothesis& hyp)
{
	//return o << "Pos: x: " << hyp.getPosition().x() << " y: " << hyp.getPosition().y() << " z: " << hyp.getPosition().z() << std::endl;
	return o << "test";
}



#endif /* MANTIS_INCLUDE_MANTIS2_MANTIS2TYPES_H_ */
