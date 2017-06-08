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
#define QUAD_STRECH_FACTOR 1.31
#define MASK_THICKNESS_FACTOR 0.25

//params
int FAST_THRESHOLD;
int CANNY_HYSTERESIS;
int RATE;
int POLYGON_EPSILON;
std::string CAMERA_0_TOPIC;
double SEARCH_RADIUS_MULTIPLIER;

void getParameters()
{
	ros::param::param <std::string> ("~camera0Topic", CAMERA_0_TOPIC, "camera/image_color");

	ros::param::param <int> ("~rate", RATE, 10);

	ros::param::param <int> ("~fast_thresh", FAST_THRESHOLD, 60);

	ros::param::param <int> ("~canny_hysteresis", CANNY_HYSTERESIS, 50);

	ros::param::param <int> ("~polygon_epsilon", POLYGON_EPSILON, 10);

	ros::param::param <double> ("~neighborhood_search_radius_multiplier", SEARCH_RADIUS_MULTIPLIER, 0.1);

	//WHITE_BGR = WHITE_INIT;
	//RED_BGR = RED_INIT;
	//GREEN_BGR = GREEN_INIT;
}

#endif /* MANTIS_INCLUDE_MANTIS2_MANTIS2PARAMS_H_ */
