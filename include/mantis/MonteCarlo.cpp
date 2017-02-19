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
}

tf::Transform MonteCarlo::generateRandomTransform()
{
	tf::Quaternion q = tf::Quaternion(this->rng.uniform(0.0, 1.0),   this->rng.uniform(0.0,1.0),	 this->rng.uniform(0.0, 1.0) ,	this->rng.uniform(0.0, 1.0));
	tf::Vector3 t = tf::Vector3(this->rng.uniform(MINX, MAXX), this->rng.uniform(MINY,MAXY), this->rng.uniform(MINZ,MAXZ));
	return(tf::Transform(q,t));
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
		}
		fin.push_back(transfer);
	}
}
