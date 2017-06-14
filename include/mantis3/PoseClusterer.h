/*
 * PoseClusterer.h
 *
 *  Created on: Jun 14, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_POSECLUSTERER_H_
#define MANTIS_INCLUDE_MANTIS3_POSECLUSTERER_H_

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>


#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"

#include "mantis3/Mantis3Types.h"

class PoseClusterer {
public:

	struct Results{
		std::vector<std::vector<int> > indexes;
		std::vector<cv::Point3f> centers;

		Results removeSmallClusters(int min_size = 3)
		{
			ROS_ASSERT(centers.size() == indexes.size());
			for(int i = 0; i < indexes.size(); i++)
			{
				if(indexes.at(i).size() < min_size)
				{
					indexes.erase(indexes.begin()+i);
					centers.erase(centers.begin()+i);
					i--; // move back to account for lost memeber
				}
			}

			ROS_DEBUG_STREAM("NEW CLUSTER COUNT " << indexes.size());

			return *this;
		}

		/*
		 * converts the results to hypotheses
		 * replaces the orientation of the hypos with centers if desired
		 */
		std::vector<Hypothesis> convert2Hypotheses(std::vector<Hypothesis> in, bool replace_quat)
		{
			std::vector<Hypothesis> hyps;

			for(int i = 0; i < indexes.size(); i++)
			{
				for(auto j : indexes.at(i))
				{
					if(replace_quat)
					{
						tf::Quaternion q;
						q.setRPY(centers.at(i).x, centers.at(i).y, centers.at(i).z);
						Hypothesis hyp = in.at(j);
						hyp.setW2C(tf::Transform(q, hyp.getPosition())); // change the rotation
						hyps.push_back(hyp);
					}
					else
					{
						hyps.push_back(in.at(j));
					}
				}
			}

			return hyps;
		}
	};

	struct Pose{
		tf::Quaternion q;
		tf::Vector3 r;

		bool q_neighbor, r_neighbor;

		Pose(tf::Transform trans)
		{
			q = trans.getRotation();
			r = trans.getOrigin();

			q_neighbor = false;
			r_neighbor = false;
		}

		Pose(tf::Quaternion quat)
		{
			q = quat;
			r = tf::Vector3();

			q_neighbor = false;
			r_neighbor = false;
		}

		Pose(tf::Vector3 vec)
		{
			q = tf::Quaternion(0, 0, 0, 1);
			r = vec;

			q_neighbor = false;
			r_neighbor = false;
		}

		tf::Vector3 fixEuler(tf::Vector3 in)
		{
			bool more = true;

			double two_pi = 2*CV_PI;

			while(more)
			{
				more = false;
				if(in.x() < 0)
				{
					in.setX(in.x() + two_pi);
					more = true;
				}
				if(in.x() > two_pi)
				{
					in.setX(in.x() - two_pi);
					more = true;
				}

				if(in.y() < 0)
				{
					in.setY(in.y() + two_pi);
					more = true;
				}
				if(in.y() > two_pi)
				{
					in.setY(in.y() - two_pi);
					more = true;
				}

				if(in.z() < 0)
				{
					in.setZ(in.z() + two_pi);
					more = true;
				}
				if(in.z() > two_pi)
				{
					in.setZ(in.z() - two_pi);
					more = true;
				}

			}

			return in;
		}

		tf::Vector3 getEuler(){
			double x, y, z;

			tf::Matrix3x3(q).getRPY(x, y, z);

			//return fixEuler(tf::Vector3(x, y, z));
			return (tf::Vector3(x, y, z));
		}
	};

	std::vector<Pose> poses;

	PoseClusterer();

	PoseClusterer(std::vector<tf::Quaternion> quats);
	PoseClusterer(std::vector<tf::Vector3> rs);

	double radius(cv::Point3f query, cv::Point3f ref);

	Results clusterByAngle(double radiu, int max_results);

	std::vector<std::vector<int> > KDcluster(double radius, int max_results, std::vector<cv::Point3f> data);
	std::vector<std::vector<int> > BFcluster(double radius, std::vector<cv::Point3f> data, std::vector<cv::Point3f>& centers);

	cv::Point3f getClusterCenter(std::vector<cv::Point3f> data, std::vector<int> indexes);


	std::vector<float> getClusterCenterQuery(std::vector<cv::Point3f> data, std::vector<int> indexes);
};


#endif /* MANTIS_INCLUDE_MANTIS3_POSECLUSTERER_H_ */
