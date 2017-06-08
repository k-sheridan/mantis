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

#include <eigen3/Eigen/Eigen>

#define SUPER_DEBUG false

#define QUAD_STRECH_FACTOR 1.31
#define MASK_THICKNESS_FACTOR 0.25

#define BLUR_SIGMA 10

//HSV mins and maxes
#define WHITE_MIN cv::Scalar(0, 0, 80)

#define RED_MIN1 cv::Scalar(0, 170, 170)
#define RED_MIN2 cv::Scalar(165, 170, 170)

#define GREEN_MIN cv::Scalar(70, 70, 50)

#define WHITE_MAX cv::Scalar(360, 20, 100)

#define RED_MAX1 cv::Scalar(10, 255, 255)
#define RED_MAX2 cv::Scalar(180, 255, 255)

#define GREEN_MAX cv::Scalar(140, 100, 100)

struct Frame;
cv::Mat run(Frame* f);

int FAST_THRESHOLD;
int CANNY_HYSTERESIS;
int RATE;
int POLYGON_EPSILON;
std::string CAMERA_0_TOPIC;
//cv::Vec3b RED_BGR, GREEN_BGR, WHITE_BGR;
double COLOR_THRESHOLD;
double SEARCH_RADIUS_MULTIPLIER;

ros::ServiceClient particle_filter_service;

#if SUPER_DEBUG
cv::Mat final;
bool imgReady = false;
#endif
/*
enum Color{
	RED,
	GREEN,
	WHITE,
	OTHER
};*/

struct StateEstimate{
	Eigen::Vector3d pos;
	Eigen::Quaterniond quat;
};

struct Measurement{
	cv::Point pos;
	cv::Scalar color;

	Measurement(cv::Point _pos, cv::Scalar _color)
	{
		pos = _pos;
		color = _color;
	}
};

struct Quadrilateral
{
	cv::Point2f center; // the centroid of this quad
	std::vector<cv::Point> contour; // the raw detected corners of this quad
	std::vector<cv::Point> test_points; // test points of the quad for color

	double side_length; // the approximate area of this quad
	//Color color; // the color of this quad
	bool neighbor; // has this quad been found to be the neighbor of another already

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

	std::vector<cv::Point> computeTestPoints()
																	{
		std::vector<cv::Point> test;
		for(auto e : contour)
		{
			cv::Point pt = cv::Point((cv::Point2f(e) - center) * QUAD_STRECH_FACTOR + center);
			test.push_back(pt);
		}
		return test;
																	}

	Quadrilateral(std::vector<cv::Point> _contour)
	{
		neighbor = false;
		this->contour = _contour;
		ROS_ASSERT(_contour.size() == 4);
		//ROS_DEBUG("computing delta vec");
		cv::Point first_delta = (this->contour.at(0) - this->contour.at(1));
		//ROS_DEBUG_STREAM("delta is: " << first_delta);
		this->side_length = sqrt(first_delta.x*first_delta.x + first_delta.y*first_delta.y);
		ROS_DEBUG_STREAM("side length: " << side_length);
		this->center = this->computeQuadCenter(_contour);
		//this->color = Color::OTHER;

		//test points
		test_points = computeTestPoints();
	}
};

struct Frame
{
	cv::Mat img;
	sensor_msgs::Image img_msg;
	sensor_msgs::CameraInfo cam_info_msg;
	cv::Mat canny;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<Quadrilateral> quads;
	std::vector<cv::Vec4i> contour_hierarchy;
};

Frame frame1;

bool checkPixel(cv::Mat sample, cv::Scalar min, cv::Scalar max)
{
	//ROS_DEBUG_STREAM(min.val[0] << " " << min.val[1]);
	//ROS_DEBUG_STREAM((int)sample.data[0] << " " << (int)sample.data[1]);
	if(!(sample.data[0] >= min.val[0] && sample.data[0] <= max.val[0]))
	{
		return false;
	}
	if(!(sample.data[1] >= min.val[1] && sample.data[1] <= max.val[1]))
	{
		return false;
	}
	if(!(sample.data[2] >= min.val[2] && sample.data[2] <= max.val[2]))
	{
		return false;
	}
	return true;
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


int removeDuplicateQuads(std::vector<Quadrilateral>& quads)
{
	std::vector<Quadrilateral> keepers;
	std::vector<cv::Point2f> original_pts;
	for(auto& e : quads)
	{
		original_pts.push_back(e.center);
	}

	cv::flann::KDTreeIndexParams indexParams;
	cv::flann::Index kdtree(cv::Mat(original_pts).reshape(1), indexParams);

	int neighbors = 0;

	for(auto& e : quads)
	{
		if(!e.neighbor)
		{
			std::vector<float> query;
			query.push_back(e.center.x);
			query.push_back(e.center.y);

			std::vector<int> indices;
			std::vector<float> dists;

			kdtree.radiusSearch(query, indices, dists, SEARCH_RADIUS_MULTIPLIER * e.side_length, 4);

			for(std::vector<int>::iterator it = indices.begin() + 1; it != indices.end(); it++)
			{
				quads.at(*it).neighbor = true; // this will remove this from the list
				neighbors++;
			}
		}
	}

	ROS_DEBUG_STREAM("found " << neighbors << " neighbors");

	ROS_DEBUG_STREAM("size before: " << quads.size());
	quads.erase(std::remove_if(quads.begin(), quads.end(), [](Quadrilateral x){return x.neighbor;}), quads.end());
	ROS_DEBUG_STREAM("size after: " << quads.size());

	return neighbors;

}

cv::Mat createGridMask(std::vector<Quadrilateral> quads, int rows, int cols)
{
	cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8U);

	for(auto e : quads)
	{
		int thickness = e.side_length * MASK_THICKNESS_FACTOR; // TODO make thickness a func of sidelength
		ROS_ASSERT(e.test_points.size() == 4);
		std::vector<std::vector<cv::Point> > cont;
		cont.push_back(e.test_points);
		cv::drawContours( mask, cont, 0, cv::Scalar(255, 255, 255), thickness, 8);
	}

	return mask;
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

StateEstimate runParticleFilter(cv::Mat measurement)
{
	mantis::mantisService srv;

	srv.request.camera_info.push_back(frame1.cam_info_msg);
	cv_bridge::CvImage cv_image = cv_bridge::CvImage(frame1.img_msg.header, frame1.img_msg.encoding, measurement);
	srv.request.image.push_back(*cv_image.toImageMsg());

	particle_filter_service.waitForExistence();

	if(particle_filter_service.call(srv))
	{
		ROS_DEBUG_STREAM("response: " << srv.response);
	}
	else
	{
		ROS_ERROR("failed to call the particle filter service");
	}

	StateEstimate est;

	est.pos.x() = srv.response.pose.position.x;
	est.pos.y() = srv.response.pose.position.y;
	est.pos.z() = srv.response.pose.position.z;

	est.quat.x() = srv.response.pose.orientation.x;
	est.quat.y() = srv.response.pose.orientation.y;
	est.quat.z() = srv.response.pose.orientation.z;
	est.quat.w() = srv.response.pose.orientation.w;

	return est;
}

void cameraCallback0(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{

	ROS_INFO("reading message");
	cv::Mat temp = cv_bridge::toCvShare(img, img->encoding)->image.clone();

	//frame1.K = get3x3FromVector(cam->K);
	//frame1.D = cv::Mat(cam->D, false);
	frame1.cam_info_msg = *cam;
	frame1.img_msg = *img;
	frame1.img = temp;


	//ROS_INFO_STREAM("intrinsic; " << frame0.K);
	//ROS_INFO_STREAM("distortion; " << frame0.D);

	cv::Mat measurement1 = run(&frame1);

	StateEstimate est = runParticleFilter(measurement1);
}

cv::Mat run(Frame* f)
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
			f->quads.push_back(Quadrilateral(approx));
		}
	}

	removeDuplicateQuads(f->quads);

	cv::Mat mask, measurement;
	mask = createGridMask(f->quads, f->img.rows, f->img.cols);
	f->img.copyTo(measurement, mask); // create the data to be used

	//cv::GaussianBlur(measurement, measurement, cv::Size(21, 21), 21, 21);
	cv::GaussianBlur(measurement, measurement, cv::Size(0, 0), BLUR_SIGMA, BLUR_SIGMA);

	//cv::fisheye::undistortImage(measurement, measurement, f->K, f->D, cv::Mat::eye(3, 3, CV_32F));
	//cv::fisheye::undistortImage(measurement, measurement, f->K, f->D, f->K);


#if SUPER_DEBUG
	final = measurement;
	/*
	cv::Mat temp;
	f->img.copyTo(temp);
	final = cv::Mat::zeros(temp.rows, temp.cols, CV_8U);
	temp.copyTo(final, createGridMask(f->quads, temp.rows, temp.cols));*/
	//temp.copyTo(final);
	//final = createGridMask(f->quads, temp.rows, temp.cols);

	/*final = cv::Mat::zeros(final.rows, final.cols, final.type());

	for(auto t : f->quads)
	{
		for(auto e : t.test_points)
		{
			cv::drawMarker(final, e, temp.at<cv::Vec3b>(e));
		}
	}*/

	/*
	cv::RNG rng(12345);
	for( int i = 0; i< f->quads.size(); i++ )
	{
		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		std::vector<std::vector<cv::Point> > cont;
		cont.push_back(f->quads.at(i).contour);
		cv::drawContours( final, cont, 0, color, 2, 8);
		switch (f->quads.at(i).color) {
		case Color::WHITE:
			ROS_DEBUG_STREAM("drawing white marker");
			//cv::drawMarker(final, f->quads.at(i).center, cv::Scalar(255, 255, 255));
			break;
		case Color::GREEN:
			ROS_DEBUG_STREAM("drawing green marker");
			//cv::drawMarker(final, f->quads.at(i).center, cv::Scalar(0, 255, 0));
			break;
		case Color::RED:
			ROS_DEBUG_STREAM("drawing red marker");
			cv::drawMarker(final, f->quads.at(i).center, cv::Scalar(0, 0, 255));
			break;
		default:
			ROS_DEBUG_STREAM("drawing other marker");
			//cv::drawMarker(final, f->quads.at(i).center, cv::Scalar(255, 0, 255));
			break;
		}
		//cv::drawMarker(final, f->quads.at(i).center, cv::Scalar(255, 255, 255), cv::MarkerTypes::MARKER_STAR);
	}*/

	imgReady = true;
	cv::imshow("debug", final);
	cv::waitKey(30);
#endif

	return measurement;
}
void getParameters()
{
	ros::param::param <std::string> ("~camera0Topic", CAMERA_0_TOPIC, "camera/image_color");

	ros::param::param <int> ("~rate", RATE, 10);

	ros::param::param <int> ("~fast_thresh", FAST_THRESHOLD, 60);

	ros::param::param <int> ("~canny_hysteresis", CANNY_HYSTERESIS, 50);

	ros::param::param <int> ("~polygon_epsilon", POLYGON_EPSILON, 10);

	ros::param::param <double> ("~color_threshold", COLOR_THRESHOLD, 60);

	ros::param::param <double> ("~neighborhood_search_radius_multiplier", SEARCH_RADIUS_MULTIPLIER, 0.1);

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

	ROS_DEBUG("creating service");

	particle_filter_service = n.serviceClient<mantis::mantisService>("mantis_service"); // create a client

	ROS_DEBUG("created services");

	while(n.ok()){

		//ROS_DEBUG("spinning once");
		ros::spinOnce();
		//ROS_DEBUG("spun once");
		loop_rate.sleep();

#if SUPER_DEBUG
		if(imgReady){
		cv::imshow("debug", final);
		cv::waitKey(30);}
#endif
	}

	return 0;
}

/*
 *
 * get the color of a pixel by searching around it for colors in the THRESH

Color getCornerColor(Frame* f, cv::Point& pos)
{
	Color theColor = Color::OTHER;


	cv::Rect roi(pos.x, pos.y, 1, 1);
	cv::Mat sample;
	f->img(roi).copyTo(sample);
	cv::Mat hsv;
	cv::cvtColor(sample, hsv, CV_BGR2HSV);

	ROS_DEBUG_STREAM("hsv: " << hsv);


	//red
	if(checkPixel(hsv, RED_MIN1, RED_MAX1))
	{
		ROS_DEBUG("found red corner");
		return Color::RED;
	}
	if(checkPixel(hsv, RED_MIN2, RED_MAX2))
	{
		ROS_DEBUG("found red corner");
		return Color::RED;
	}




	return theColor;
}


 * get the color of a quadrilateral
 * RED = quad near the RED line
 * GREEN = quad near the GREEN Line
 * WHITE = quad in the center of the grid
 * OTHER = outlier

Color getQuadColor(Frame* f, Quadrilateral quad)
{
	Color finalColor = Color::WHITE;

	for(std::vector<cv::Point>::iterator it = quad.contour.begin(); it != quad.contour.begin() + 4; it++)
	{
		//cv::Point test = (cv::Point2f((*it)) - quad.center) * QUAD_STRECH_FACTOR + quad.center;
		Color thisCorner = getCornerColor(f, (*it));
		if(thisCorner == Color::OTHER)
		{
			return Color::OTHER; // this is an invalid corner
		}
		else if(thisCorner == Color::GREEN || thisCorner == Color::RED)
		{
			return thisCorner;
		}
	}

	return finalColor;
}
 */
