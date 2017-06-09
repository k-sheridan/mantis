/*
 * QuadDetection.h
 *
 *  Created on: Jun 8, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS_QUADDETECTION_H_
#define MANTIS_INCLUDE_MANTIS_QUADDETECTION_H_


struct Quadrilateral
{
	cv::Point2f center; // the centroid of this quad
	std::vector<cv::Point> contour; // the raw detected corners of this quad
	std::vector<cv::Point> test_points; // test points of the quad for pose

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
		//ROS_DEBUG_STREAM("side length: " << side_length);
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

	//ROS_DEBUG_STREAM("found " << neighbors << " neighbors");

	//ROS_DEBUG_STREAM("size before: " << quads.size());
	quads.erase(std::remove_if(quads.begin(), quads.end(), [](Quadrilateral x){return x.neighbor;}), quads.end());
	//ROS_DEBUG_STREAM("size after: " << quads.size());

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

int detectQuadrilaterals(Frame* f)
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

#if SUPER_DEBUG
	final = f->img.clone();
	/*
	cv::Mat temp;
	f->img.copyTo(temp);
	final = cv::Mat::zeros(temp.rows, temp.cols, CV_8U);
	temp.copyTo(final, createGridMask(f->quads, temp.rows, temp.cols));*/
	//temp.copyTo(final);
	//final = createGridMask(f->quads, temp.rows, temp.cols);

	//final = cv::Mat::zeros(final.rows, final.cols, final.type());

	for(auto t : f->quads)
	{
		for(auto e : t.test_points)
		{
			cv::drawMarker(final, e, cv::Scalar(125, 125, 125));
		}
	}


	//cv::RNG rng(12345);
	for( int i = 0; i< f->quads.size(); i++ )
	{
		cv::Scalar color = cv::Scalar( 255,0,0 );
		std::vector<std::vector<cv::Point> > cont;
		cont.push_back(f->quads.at(i).contour);
		cv::drawContours( final, cont, 0, color, 2, 8);
		/*
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
		}*/
		//cv::drawMarker(final, f->quads.at(i).center, cv::Scalar(255, 255, 255), cv::MarkerTypes::MARKER_STAR);
	}

	imgReady = true;
	cv::imshow("debug", final);
	cv::waitKey(30);
#endif

	return f->quads.size();

}

#endif /* MANTIS_INCLUDE_MANTIS_QUADDETECTION_H_ */
