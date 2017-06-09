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
		tf::Quaternion q = w2c.getRotation();

		geometry_msgs::PoseStamped msg;

		msg.pose.position.x = w2c.getOrigin().x();
		msg.pose.position.y = w2c.getOrigin().y();
		msg.pose.position.z = w2c.getOrigin().z();

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

};

std::ostream& operator<< (std::ostream& o, const Hypothesis& hyp)
{
	//return o << "Pos: x: " << hyp.getPosition().x() << " y: " << hyp.getPosition().y() << " z: " << hyp.getPosition().z() << std::endl;
	return o << "test";
}



#endif /* MANTIS_INCLUDE_MANTIS2_MANTIS2TYPES_H_ */
