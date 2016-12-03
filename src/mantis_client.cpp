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

void detectLines();

struct Frame
{
	cv::Mat img;
	ros::Time t;
	cv::Mat K;
	cv::Mat D;
};

Frame frame0;

int rate;

std::string camera0Topic;

cv::Mat get3x3FromVector(boost::array<double, 9> vec)
{
	cv::Mat mat = cv::Mat(3, 3, CV_32F);
	for(int i = 0; i < 3; i++)
	{
		mat.at<float>(i, 0) = vec.at(3 * i + 0);
		mat.at<float>(i, 1) = vec.at(3 * i + 1);
		mat.at<float>(i, 2) = vec.at(3 * i + 2);
	}

	ROS_DEBUG_STREAM_ONCE("K = " << mat);
	return mat;
}

void cameraCallback0(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{
	cv::Mat temp = cv_bridge::toCvShare(img, "bgr8")->image.clone();

	frame0.K = get3x3FromVector(cam->K);
	frame0.D = cv::Mat(cam->D, false);

	cv::fisheye::undistortImage(temp,temp,frame0.K,frame0.D, frame0.K);

	frame0.img = temp;
	frame0.t = img->header.stamp;


	ROS_INFO_STREAM("intrinsic; " << frame0.K);
	ROS_INFO_STREAM("intrinsic; " << frame0.D);

	detectLines();

}

void detectLines()
{
	cv::Mat mono, cmono;
	cv::cvtColor(frame0.img, mono, CV_BGR2GRAY);

	cv::Canny(mono, cmono, 50, 200, 3);

	std::vector<cv::Vec4i> Lines;
	cv::HoughLinesP(cmono, Lines, 1, CV_PI/180, 50, 50, 10);
	for (size_t i = 0; i < Lines.size(); i++)
	{
	    cv::Vec4i l = Lines[i];
	    cv::line( mono, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
	}
	cv::imshow("show",mono);
	cv::waitKey(30);
}
void getParameters()
{
	ros::param::param <std::string> ("~camera0Topic", camera0Topic, "bottomCamera/image_color");

	ros::param::param <int> ("~rate", rate, 10);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	getParameters();

	ros::Rate loop_rate(rate);

	image_transport::ImageTransport it(n);

	image_transport::CameraSubscriber cameraSub0 = it.subscribeCamera(camera0Topic, 1, cameraCallback0);

	while(n.ok()){
		loop_rate.sleep();

		ros::spinOnce();
	}

	return 0;
}
