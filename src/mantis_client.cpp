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

#define SUPER_DEBUG true

struct Frame;
void run(Frame* f);

int FAST_THRESHOLD;
int CANNY_HYSTERESIS;
int RATE;
std::string CAMERA_0_TOPIC;

struct Frame
{
	cv::Mat img;
	ros::Time t;
	cv::Mat K;
	cv::Mat D;
	cv::Mat canny;
	std::vector<cv::KeyPoint> fast_corners;
};

Frame frame0;

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

	ROS_DEBUG("reading message");
	cv::Mat temp = cv_bridge::toCvShare(img, "bgr8")->image.clone();

	frame0.K = get3x3FromVector(cam->K);
	frame0.D = cv::Mat(cam->D, false);

	frame0.img = temp;
	frame0.t = img->header.stamp;


	//ROS_INFO_STREAM("intrinsic; " << frame0.K);
	//ROS_INFO_STREAM("distortion; " << frame0.D);

	run(&frame0);

}

void run(Frame* f)
{
	cv::Mat mono;

	cv::cvtColor(f->img, mono, CV_BGR2GRAY);

	cv::Canny(mono, f->canny, CANNY_HYSTERESIS, 3 * CANNY_HYSTERESIS, 3);
	//cv::dilate(f->canny, f->canny, cv::Mat(), cv::Point(-1, -1), 3);
	cv::GaussianBlur(f->canny, f->canny, cv::Size(3, 3), 3, 3);
	cv::FAST(f->canny, f->fast_corners, FAST_THRESHOLD, true);




#ifdef SUPER_DEBUG
	cv::Mat final;
	cv::cvtColor(f->canny, final, CV_GRAY2BGR);

	for(auto& e : f->fast_corners)
	{
		cv::drawMarker(final, e.pt, cv::Scalar(0, 255, 255), cv::MarkerTypes::MARKER_STAR, 3);
	}

	cv::imshow("debug", final);
	cv::waitKey(30);
#endif
}
void getParameters()
{
	ros::param::param <std::string> ("~camera0Topic", CAMERA_0_TOPIC, "camera/image_color");

	ros::param::param <int> ("~rate", RATE, 10);

	ros::param::param <int> ("~fast_thresh", FAST_THRESHOLD, 60);

	ros::param::param <int> ("~canny_hysteresis", CANNY_HYSTERESIS, 110);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	getParameters();

	ros::Rate loop_rate(RATE);

	image_transport::ImageTransport it(n);

	image_transport::CameraSubscriber cameraSub0 = it.subscribeCamera(CAMERA_0_TOPIC, 1, cameraCallback0);

	while(n.ok()){

		ROS_DEBUG("spinning once");
		ros::spinOnce();
		ROS_DEBUG("spun once");
		loop_rate.sleep();

	}

	return 0;
}
