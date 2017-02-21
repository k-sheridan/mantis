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
int POLYGON_EPSILON;
std::string CAMERA_0_TOPIC;
cv::Vec3b RED_BGR, GREEN_BGR, WHITE_BGR;
double COLOR_THRESHOLD;

struct Frame
{
	cv::Mat img;
	ros::Time t;
	cv::Mat K;
	cv::Mat D;
	cv::Mat canny;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<std::vector<cv::Point> > quads;
	std::vector<cv::Vec4i> contour_hierarchy;
};

Frame frame0;

enum Color{
	RED,
	GREEN,
	WHITE,
	OTHER
};

double dist2color(cv::Vec3b& test, cv::Vec3b& compare)
{
	double db = test[0]-compare[0];
	double dg = test[1]-compare[1];
	double dr = test[2]-compare[2];
	return sqrt(db*db+dg*dg+dr*dr);
}

/*
 * get the color of a pixel by searching a radius
 */
Color getPixelColor(Frame* f, cv::Point& pos)
{
	cv::Vec3b test = f->img.at<cv::Vec3b>(pos);

	if(dist2color(test, WHITE_BGR) < COLOR_THRESHOLD)
		return Color::WHITE;
	else if(dist2color(test, GREEN_BGR) < COLOR_THRESHOLD)
		return Color::GREEN;
	else if(dist2color(test, RED_BGR) < COLOR_THRESHOLD)
		return Color::RED;
	else
	{
		return Color::OTHER;
	}
}

/*
 * get the color of a pixel by searching around it for colors in the THRESH
 */
Color getCornerColor(Frame* f, cv::Point& pos, int rad)
{

}

/*
 * get the color of a quadrilateral
 * RED = quad near the RED line
 * GREEN = quad near the GREEN Line
 * WHITE = quad in the center of the grid
 * OTHER = outlier
 */
Color getQuadColor(Frame* f, std::vector<cv::Point> quad, int rad)
{

}

void convert2Binary(cv::Mat& img){

	for(int i = 0; i < img.rows; i++)
	{
		for(int j = 0; j < img.cols; j++)
		{
			if(img.at<uint8_t>(i, j))
			{
				img.at<uint8_t>(i, j) = 255;
			}
		}
	}

}

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
	f->quads.clear();

	cv::Mat mono;
	//cv::fisheye::undistortImage(f->img, f->img, f->K, f->D, f->K);
	cv::cvtColor(f->img, mono, CV_BGR2GRAY);

	cv::GaussianBlur(mono, mono, cv::Size(5, 5), 3, 3);
	cv::Canny(mono, f->canny, CANNY_HYSTERESIS, 3 * CANNY_HYSTERESIS, 3);
	cv::dilate(f->canny, f->canny, cv::Mat(), cv::Point(-1, -1), 1);

	cv::findContours(f->canny, f->contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point> approx;
	for(int i = 0; i < f->contours.size(); i++)
	{
		cv::approxPolyDP(f->contours.at(i),approx,POLYGON_EPSILON,true);
		if(approx.size() == 4)
		{
			f->quads.push_back(approx);
		}
	}


#ifdef SUPER_DEBUG
	cv::Mat final = f->img;


	cv::RNG rng(12345);
	for( int i = 0; i< f->quads.size(); i++ )
	{
		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		cv::drawContours( final, f->quads, i, color, 2, 8);
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

	ros::param::param <int> ("~canny_hysteresis", CANNY_HYSTERESIS, 50);

	ros::param::param <int> ("~polygon_epsilon", POLYGON_EPSILON, 10);

	ros::param::param <double> ("~color_threshold", COLOR_THRESHOLD, 5);
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
