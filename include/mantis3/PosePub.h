/*
 * PosePub.h
 *
 *  Created on: Jun 19, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_POSEPUB_H_
#define MANTIS_INCLUDE_MANTIS3_POSEPUB_H_


void publishPose(Hypothesis hyp, ros::Time stamp, double max_yaw_difference)
{
	ROS_DEBUG_STREAM("yaw diff: " << max_yaw_difference);

	if(max_yaw_difference > MINIMUM_YAW_DIFFERENCE)
	{
		ROS_DEBUG_STREAM("publishing pose");

		geometry_msgs::PoseWithCovarianceStamped pose;

		pose.header.stamp = stamp;
		pose.header.frame_id = WORLD_FRAME;

		pose.pose.pose.orientation.w = hyp.getQuaternion().w();
		pose.pose.pose.orientation.x = hyp.getQuaternion().x();
		pose.pose.pose.orientation.y = hyp.getQuaternion().y();
		pose.pose.pose.orientation.z = hyp.getQuaternion().z();

		pose.pose.pose.position.x = hyp.getPosition().x();
		pose.pose.pose.position.y = hyp.getPosition().y();
		pose.pose.pose.position.z = hyp.getPosition().z();

		double var = hyp.error * PIXEL_ERROR_VARIANCE_COEFF;

		ROS_DEBUG_STREAM("variance: " << var);

		pose.pose.covariance = {var, 0, 0, 0, 0, 0,
								0, var, 0, 0, 0, 0,
								0, 0, var, 0, 0, 0,
								0, 0, 0, var, 0, 0,
								0, 0, 0, 0, var, 0,
								0, 0, 0, 0, 0, var};

		posePub.publish(pose);

	}
}



#endif /* MANTIS_INCLUDE_MANTIS3_POSEPUB_H_ */
