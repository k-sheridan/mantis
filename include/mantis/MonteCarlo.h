/*
 * MonteCarlo.h
 *
 *  Created on: Nov 12, 2016
 *      Author: kevin
 */

#ifndef MANTIS_INCLUDE_MANTIS_MONTECARLO_H_
#define MANTIS_INCLUDE_MANTIS_MONTECARLO_H_

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include "message_filters/subscriber.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include "error/MantisRequest.h"

#define DEFAULT_MINX  -10
#define DEFAULT_MINY  -10
#define DEFAULT_MINZ    0
#define DEFAULT_MAXX   10
#define DEFAULT_MAXY   10
#define DEFAULT_MAXZ    5


class MonteCarlo {
public:
	MantisRequest req;

	MonteCarlo();
	virtual ~MonteCarlo();

	tf::Transform generateRandomTransform();
	tf::TransformListener tfListener;

private:
	cv::RNG rng;
	float MINX, MINY, MINZ, MAXX, MAXY, MAXZ;

};

#include "error/computeError.h"

#endif /* MANTIS_INCLUDE_MANTIS_MONTECARLO_H_ */
