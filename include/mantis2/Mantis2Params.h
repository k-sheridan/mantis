/*
 * Mantis2Params.h
 *
 *  Created on: Jun 8, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS2_MANTIS2PARAMS_H_
#define MANTIS_INCLUDE_MANTIS2_MANTIS2PARAMS_H_


#define SUPER_DEBUG true

#if SUPER_DEBUG
bool imgReady = false;
cv::Mat final;
#endif

//defines
#define QUAD_STRECH_FACTOR 1.3
#define MASK_THICKNESS_FACTOR 0.25

#define GRID_SIZE 9
#define DEF_GRID_SPACING 0.32

#define MAX_QUAD_ERROR 12.0

#define POSE_SOLVE_METHOD cv::SOLVEPNP_ITERATIVE

#define HYPOTHESES_PUB_TOPIC "mantis/hypotheses"

#define BASE_FRAME "base_link"
#define WORLD_FRAME "world"

//params
double GRID_SPACING;
int FAST_THRESHOLD;
int CANNY_HYSTERESIS;
int RATE;
int POLYGON_EPSILON;
std::string QUAD_DETECT_CAMERA_TOPIC;
double SEARCH_RADIUS_MULTIPLIER;

std::vector<cv::Point3d> gridSquare;

std::vector<std::vector<cv::Point3d>> gridSquarePossibilities;

std::vector<cv::Point3d> generateGridSquare(double spacing){
	std::vector<cv::Point3d> square;
	square.push_back(cv::Point3d(spacing/2, spacing/2, 0));
	square.push_back(cv::Point3d(-spacing/2, spacing/2, 0));
	square.push_back(cv::Point3d(-spacing/2, -spacing/2, 0));
	square.push_back(cv::Point3d(spacing/2, -spacing/2, 0));
	return square;
}

std::vector<std::vector<cv::Point3d>> generatePossibleOrientations(double grid_spacing){
	std::vector<std::vector<cv::Point3d>> possibilities; // possibility possibilities
	std::vector<cv::Point3d> possibility;
	cv::Point3d temp;

	possibility.push_back(cv::Point3d(grid_spacing/2, grid_spacing/2, 0));
	possibility.push_back(cv::Point3d(-grid_spacing/2, grid_spacing/2, 0));
	possibility.push_back(cv::Point3d(-grid_spacing/2, -grid_spacing/2, 0));
	possibility.push_back(cv::Point3d(grid_spacing/2, -grid_spacing/2, 0));

	possibilities.push_back(possibility);

	std::rotate(possibility.rbegin(), possibility.rbegin() + 1, possibility.rend());
	possibilities.push_back(possibility);

	std::rotate(possibility.rbegin(), possibility.rbegin() + 1, possibility.rend());
	possibilities.push_back(possibility);

	std::rotate(possibility.rbegin(), possibility.rbegin() + 1, possibility.rend());
	possibilities.push_back(possibility);

	return possibilities;
}

std::vector<tf::Vector3> parseCoordinatesFromString(std::string str) {
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

void getParameters()
{
	ros::param::param <std::string> ("~quadDetectCameraTopic", QUAD_DETECT_CAMERA_TOPIC, "camera/image_color");

	ros::param::param <int> ("~rate", RATE, 10);

	ros::param::param <int> ("~fast_thresh", FAST_THRESHOLD, 60);

	ros::param::param <int> ("~canny_hysteresis", CANNY_HYSTERESIS, 50);

	ros::param::param <int> ("~polygon_epsilon", POLYGON_EPSILON, 10);

	ros::param::param <double> ("~neighborhood_search_radius_multiplier", SEARCH_RADIUS_MULTIPLIER, 0.1);
	ros::param::param <double> ("~grid_spacing", GRID_SPACING, DEF_GRID_SPACING);

	gridSquare = generateGridSquare(GRID_SPACING);
	gridSquarePossibilities = generatePossibleOrientations(GRID_SPACING);

	//WHITE_BGR = WHITE_INIT;
	//RED_BGR = RED_INIT;
	//GREEN_BGR = GREEN_INIT;
}

#endif /* MANTIS_INCLUDE_MANTIS2_MANTIS2PARAMS_H_ */
