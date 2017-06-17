/*
 * Mantis2Params.h
 *
 *  Created on: Jun 8, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS2_MANTIS2PARAMS_H_
#define MANTIS_INCLUDE_MANTIS2_MANTIS2PARAMS_H_


#define SUPER_DEBUG false

#if SUPER_DEBUG
bool imgReady = false;
cv::Mat final;
#endif

//defines
#define QUAD_STRECH_FACTOR 1.3
#define MASK_THICKNESS_FACTOR 0.25

#define GRID_SIZE 9
#define DEF_GRID_SPACING 0.32

#define MAX_QUAD_ERROR 0.5

#define MAX_ANGLE_DIFFERENCE 0.1

#define POSE_SOLVE_METHOD cv::SOLVEPNP_ITERATIVE

#define HYPOTHESES_PUB_TOPIC "mantis/hypotheses"

#define BASE_FRAME "base_link"
#define WORLD_FRAME "world"

#define PROJECTION_BIAS 1.1

//COLOR STUFF
#define WHITE cv::Vec3i(255, 255, 255)
#define RED cv::Vec3i(50, 85, 255)
#define GREEN cv::Vec3i(50, 255, 85)

#define MAX_WHITE_ERROR 20000
#define MAX_RED_ERROR 2000
#define MAX_GREEN_ERROR 20000

#define MASK_BLUR_SIGMA 3
#define MASK_BLUR_SIZE cv::Size(3, 3)
#define MASK_INIT_CLOSE_ITER 3
#define MASK_CLOSE_ASCEND_ITER 3
#define MASK_ERODE_ITER 3

#define COLOR_SEARCH_AREA cv::Size(2, 2)

// PARTICLE FILTER

#define ROT_SIGMA 0.03
#define TRANS_SIGMA 0.01

//params
double GRID_SPACING;
int FAST_THRESHOLD;
int CANNY_HYSTERESIS;
int RATE;
int POLYGON_EPSILON;
std::string QUAD_DETECT_CAMERA_TOPIC;
double SEARCH_RADIUS_MULTIPLIER;

std::vector<tf::Vector3> white_map;
std::vector<tf::Vector3> red_map;
std::vector<tf::Vector3> green_map;

tf::TransformListener* tf_listener;

cv::RNG rng(1);

std::vector<tf::Vector3> gridSquare;

std::vector<std::vector<tf::Vector3>> gridSquarePossibilities;

std::vector<tf::Vector3> generateGridSquare(double spacing){
	std::vector<tf::Vector3> square;
	square.push_back(tf::Vector3(spacing/2, spacing/2, 0));
	square.push_back(tf::Vector3(-spacing/2, spacing/2, 0));
	square.push_back(tf::Vector3(-spacing/2, -spacing/2, 0));
	square.push_back(tf::Vector3(spacing/2, -spacing/2, 0));
	return square;
}

std::vector<std::vector<tf::Vector3>> generatePossibleOrientations(double grid_spacing){
	std::vector<std::vector<tf::Vector3>> possibilities; // possibility possibilities
	std::vector<tf::Vector3> possibility;
	tf::Vector3 temp;

	possibility.push_back(tf::Vector3(grid_spacing/2, grid_spacing/2, 0));
	possibility.push_back(tf::Vector3(-grid_spacing/2, grid_spacing/2, 0));
	possibility.push_back(tf::Vector3(-grid_spacing/2, -grid_spacing/2, 0));
	possibility.push_back(tf::Vector3(grid_spacing/2, -grid_spacing/2, 0));

	possibilities.push_back(possibility);

	std::vector<tf::Vector3> possibility2;
	possibility2.push_back(tf::Vector3(grid_spacing/2, -grid_spacing/2, 0));
	possibility2.push_back(tf::Vector3(-grid_spacing/2, -grid_spacing/2, 0));
	possibility2.push_back(tf::Vector3(-grid_spacing/2, grid_spacing/2, 0));
	possibility2.push_back(tf::Vector3(grid_spacing/2, grid_spacing/2, 0));

	possibilities.push_back(possibility2);

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
	std::string whiteTemp;
	ros::param::param<std::string>("~whiteMap", whiteTemp, "");
	//ROS_DEBUG_STREAM(whiteTemp);
	white_map = parseCoordinatesFromString(whiteTemp);
	ROS_DEBUG_STREAM("size of white map "<< white_map.size());

	std::string redTemp;
	ros::param::param<std::string>("~redMap", redTemp, "");
	//ROS_DEBUG_STREAM(redTemp);
	red_map = parseCoordinatesFromString(redTemp);
	ROS_DEBUG_STREAM("size of red map"<< red_map.size());


	std::string greenTemp;
	ros::param::param<std::string>("~greenMap", greenTemp, "");
	//ROS_DEBUG_STREAM(greenTemp);
	green_map = parseCoordinatesFromString(greenTemp);
	ROS_DEBUG_STREAM("size of greenMap"<< green_map.size());

}

#endif /* MANTIS_INCLUDE_MANTIS2_MANTIS2PARAMS_H_ */
