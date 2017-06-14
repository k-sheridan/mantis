/*
 * PoseClusterer.cpp
 *
 *  Created on: Jun 14, 2017
 *      Author: pauvsi
 */

#include <mantis3/PoseClusterer.h>

PoseClusterer::PoseClusterer() {
	// TODO Auto-generated constructor stub

}

PoseClusterer::PoseClusterer(std::vector<tf::Quaternion> quats){
	for(auto e : quats)
	{
		this->poses.push_back(PoseClusterer::Pose(e));
	}
}

PoseClusterer::PoseClusterer(std::vector<tf::Vector3> rs){
	for(auto e : rs)
	{
		this->poses.push_back(PoseClusterer::Pose(e));
	}
}

/*
 * clusters each pose by its angle the radius is the maximum distance between the angles
 * returns a vector of index clusters
 */
PoseClusterer::Results PoseClusterer::clusterByAngle(double radius, int max_results){
	std::vector<cv::Point3f> angles, centers;

	for(auto e : poses)
	{
		tf::Vector3 angle = e.getEuler();
		angles.push_back(cv::Point3f(angle.x(), angle.y(), angle.z()));
		//ROS_DEBUG_STREAM("angle: " << angles.back());
	}

	Results res;

	res.indexes = this->BFcluster(radius, angles, centers);
	res.centers = centers;
	return res;

}

std::vector<std::vector<int> > PoseClusterer::BFcluster(double rad, std::vector<cv::Point3f> data, std::vector<cv::Point3f>& centers){
	std::vector<bool> isNeighbor;
	//prep the neighbor vec
	for(auto e : data)
	{
		isNeighbor.push_back(false);
	}

	centers.clear();

	std::vector<std::vector<int> > out;

	for(int i = 0; i < data.size(); i++)
	{
		std::vector<int> indexes;
		//if not a neighbor
		if(!isNeighbor.at(i)){

			//find its neighbors
			for(int j = 0; j < data.size(); j++)
			{
				if(!isNeighbor.at(j)){
					double this_rad = this->radius(data.at(i), data.at(j));
					//ROS_DEBUG_STREAM(this_rad);
					if(this_rad <= rad*1.5) //TODO *2?
					{
						indexes.push_back(j);
					}
				}
			}

			//get the center of this cluster
			cv::Point3f center = this->getClusterCenter(data, indexes);

			ROS_DEBUG_STREAM("CLUSTER SIZE BFORE: " << indexes.size());

			indexes.clear();
			//find the centers neighbors
			for(int j = 0; j < data.size(); j++)
			{
				if(!isNeighbor.at(j)){
					double this_rad = this->radius(center, data.at(j));
					//ROS_DEBUG_STREAM(this_rad);
					if(this_rad <= rad)
					{
						indexes.push_back(j);
					}
				}
			}

			ROS_DEBUG_STREAM("CLUSTER SIZE: " << indexes.size());

			out.push_back(indexes);
			centers.push_back(center);

			//set the neighbors
			for(auto e : indexes)
			{
				isNeighbor.at(e) = true;
			}

		}
	}

	return out;
}

std::vector<std::vector<int> > PoseClusterer::KDcluster(double radius, int max_results, std::vector<cv::Point3f> data)
{
	std::vector<bool> isNeighbor;

	std::vector<std::vector<int> > out;

	cv::flann::Index kdtree(cv::Mat(data).reshape(1), cv::flann::KDTreeIndexParams(10));

	//prep the neighbor vec
	for(auto e : data)
	{
		isNeighbor.push_back(false);
	}

	for(int i = 0; i < data.size(); i++)
	{
		//if not a neighbor
		if(!isNeighbor.at(i))
		{
			std::vector<float> query;
			query.push_back(data.at(i).x);
			query.push_back(data.at(i).y);
			query.push_back(data.at(i).z);

			std::vector<int> indices;
			std::vector<float> dists;

			kdtree.radiusSearch(query, indices, dists, radius, max_results);

			//ROS_DEBUG_STREAM("BEFORE CLUSTER CENTERING: " << indices.size());
			indices.clear();
			dists.clear();

			kdtree.radiusSearch(getClusterCenterQuery(data, indices), indices, dists, radius, max_results);

			//ROS_DEBUG_STREAM("AFTER CLUSTER CENTERING: " << indices.size());

			for(auto e : indices)
			{
				ROS_DEBUG_STREAM("index: " << e);
			}

			for(auto e : dists)
			{
				ROS_DEBUG_STREAM("distance: " << e);
			}

			out.push_back(indices);

			ROS_DEBUG_STREAM("FINAL CLUSTER SIZE: " << out.back().size());

			for(auto e : indices)
			{
				isNeighbor.at(e) = true;
			}
		}
	}

	ROS_DEBUG_STREAM("FINAL CLUSTER COUNT: " << out.size());

	return out;

}


double PoseClusterer::radius(cv::Point3f query, cv::Point3f ref)
{
	double dx = query.x - ref.x;
	double dy = query.y - ref.y;
	double dz = query.z - ref.z;

	return sqrt(dx*dx + dy*dy + dz*dz);
}

cv::Point3f PoseClusterer::getClusterCenter(std::vector<cv::Point3f> data, std::vector<int> indexes)
{
	cv::Point3f center(0, 0, 0);
	for(auto e : indexes)
	{
		center += data.at(e);
	}

	return center / (double)indexes.size();
}

std::vector<float> PoseClusterer::getClusterCenterQuery(std::vector<cv::Point3f> data, std::vector<int> indexes)
{
	cv::Point3f center;
	std::vector<float> query;

	query.push_back(center.x);
	query.push_back(center.y);
	query.push_back(center.z);

	return query;
}
