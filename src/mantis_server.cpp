#include "ros/ros.h"
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

#include <mantis/mantisService.h>

#include <mantis/MonteCarlo.h>

MonteCarlo* mc;

/*
 * run the particle filter
 */
bool runMantis(mantis::mantisService::Request& req, mantis::mantisService::Response& res)
{
	ROS_DEBUG("got request");

	Particle result = mc->runFilter(req);

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle nh;

	ros::ServiceServer server = nh.advertiseService("mantis_service", runMantis);

	mc = new MonteCarlo();

	ros::spin();

	delete mc; // clean up

	return 0;
}
