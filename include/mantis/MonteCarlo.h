/*
 * MonteCarlo.h
 *
 *  Created on: Nov 12, 2016
 *      Author: kevin
 */

#ifndef MANTIS_INCLUDE_MANTIS_MONTECARLO_H_
#define MANTIS_INCLUDE_MANTIS_MONTECARLO_H_

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include <random>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>

class MonteCarlo {
public:
	MonteCarlo();
	virtual ~MonteCarlo();

	tf::Transform generateRandomTransform();

private:
	std::mt199937 rng;
};

#endif /* MANTIS_INCLUDE_MANTIS_MONTECARLO_H_ */
