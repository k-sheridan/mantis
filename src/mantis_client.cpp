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

#define WHITE_INIT cv::Vec3b(255, 255, 255)
#define RED_INIT cv::Vec3b(0, 0, 255)
#define GREEN_INIT cv::Vec3b(0, 255, 0)

struct Frame;
void run(Frame* f);

int FAST_THRESHOLD;
int CANNY_HYSTERESIS;
int RATE;
int POLYGON_EPSILON;
std::string CAMERA_0_TOPIC;
cv::Vec3b RED_BGR, GREEN_BGR, WHITE_BGR;
double COLOR_THRESHOLD;
double SEARCH_RADIUS_MULTIPLIER;

enum Color{
	RED,
	GREEN,
	WHITE,
	OTHER
};

struct Quadrilateral
{
	cv::Point2f center;
	std::vector<cv::Point> contour;
	double approximate_area;
	Color color;

	Quadrilateral(std::vector<cv::Point> _contour)
	{
		this->contour = _contour;
		ROS_ASSERT(_contour.size() == 4);
		ROS_DEBUG("computing delta vec");
		cv::Point first_delta = (this->contour.at(0) - this->contour.at(1));
		ROS_DEBUG_STREAM("delta is: " << first_delta);
		this->approximate_area = pow(sqrt(first_delta.x*first_delta.x + first_delta.y*first_delta.y), 2);
		ROS_DEBUG_STREAM("approximate area: " << approximate_area);
	}
};

struct Frame
{
	cv::Mat img;
	ros::Time t;
	cv::Mat K;
	cv::Mat D;
	cv::Mat canny;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<Quadrilateral> quads;
	std::vector<cv::Vec4i> contour_hierarchy;
};

Frame frame0;

cv::Point2f computeQuadCenter(std::vector<cv::Point>& contour)
{
	float x_sigma=0, y_sigma=0;
	for(auto& e : contour)
	{
		x_sigma += e.x;
		y_sigma += e.y;
	}

	return cv::Point2f(x_sigma / contour.size(), y_sigma / contour.size());
}

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
Color getPixelColor(Frame* f, cv::Point pos)
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
	Color theColor = Color::OTHER;
	for(int i = pos.y - rad; i <= pos.y + rad; i++)
	{
		for(int j = pos.x - rad; j < pos.x + rad; j++)
		{
			if(theColor != Color::GREEN || theColor != Color::RED)
			{
				if(theColor != Color::WHITE)
				{
					Color thisPixel = getPixelColor(f, cv::Point(j, i));
					if(thisPixel != Color::OTHER)
					{
						theColor = thisPixel;
					}
				}
				else
				{
					Color thisPixel = getPixelColor(f, cv::Point(j, i));
					if(thisPixel != Color::OTHER && thisPixel != Color::WHITE)
					{
						theColor = thisPixel;
					}
				}
			}
			else
			{
				break;
			}
		}
		if(theColor == Color::GREEN || theColor == Color::RED)
		{
			break;
		}
	}
	return theColor;
}

/*
 * get the color of a quadrilateral
 * RED = quad near the RED line
 * GREEN = quad near the GREEN Line
 * WHITE = quad in the center of the grid
 * OTHER = outlier
 */
Color getQuadColor(Frame* f, Quadrilateral quad, int rad)
{
	/*Color finalColor = Color::OTHER;

	for(std::vector<cv::Point>::iterator it = quad.contour.begin(); it != quad.contour.begin() + 4; it++)
	{
		Color thisCorner = getCornerColor(f, (*it), rad);
		if(thisCorner == Color::OTHER)
		{
			return Color::OTHER; // this is an invalid corner
		}
		else
		{

		}
	}*/

	return Color::WHITE;
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

	cv::GaussianBlur(mono, mono, cv::Size(3, 3), 3, 3);
	cv::Canny(mono, f->canny, CANNY_HYSTERESIS, 3 * CANNY_HYSTERESIS, 3);
	cv::dilate(f->canny, f->canny, cv::Mat(), cv::Point(-1, -1), 2);
	cv::erode(f->canny, f->canny, cv::Mat(), cv::Point(-1, -1), 1);

	cv::findContours(f->canny, f->contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point> approx;
	for(int i = 0; i < f->contours.size(); i++)
	{
		cv::approxPolyDP(f->contours.at(i),approx,POLYGON_EPSILON,true);
		if(approx.size() == 4)
		{
			Quadrilateral this_quad = Quadrilateral(approx);
			this_quad.color = getQuadColor(f, this_quad.contour, round(SEARCH_RADIUS_MULTIPLIER * this_quad.approximate_area));
			if(this_quad.color != Color::OTHER)
			{
				this_quad.center = computeQuadCenter(this_quad.contour);
				f->quads.push_back(this_quad);
			}
			else
			{
				ROS_DEBUG("removed quadrilateral because its color was wrong");
			}
		}
	}


#ifdef SUPER_DEBUG
	cv::Mat final = f->img;


	cv::RNG rng(12345);
	for( int i = 0; i< f->quads.size(); i++ )
	{
		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		std::vector<std::vector<cv::Point> > cont;
		cont.push_back(f->quads.at(i).contour);
		cv::drawContours( final, cont, 0, color, 2, 8);
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

	ros::param::param <double> ("~color_search_radius_multiplier", SEARCH_RADIUS_MULTIPLIER, 0.001);

	//WHITE_BGR = WHITE_INIT;
	//RED_BGR = RED_INIT;
	//GREEN_BGR = GREEN_INIT;
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
