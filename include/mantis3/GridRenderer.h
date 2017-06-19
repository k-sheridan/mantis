/*
 * GridRenderer.h
 *
 *  Created on: Jun 18, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_GRIDRENDERER_H_
#define MANTIS_INCLUDE_MANTIS3_GRIDRENDERER_H_

#include <ros/ros.h>

#include <GL/glew.h>
#include <GL/glut.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/core/opengl.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#define RENDER_DEBUG = true


class GridRenderer {
public:

	int grid_size;
	double grid_spacing;
	double inner_line_thickness;
	double outer_line_thickness;

	GLuint fb, color, depth;

	cv::Vec3f WHITE, RED, GREEN;

	cv::Mat_<float> K;
	cv::Size size;
	tf::Transform c2w;

	GridRenderer(cv::Size sz);
	virtual ~GridRenderer();

	void setIntrinsic(cv::Mat_<float> K);

	void setSize(cv::Size sz){
		size = sz;
	}

	void setC2W(tf::Transform tf)
	{
		c2w = tf;
	}

	cv::Mat tfTransform2GLViewMat(tf::Transform trans);

	unsigned char* getPixelData( int x1, int y1, int x2, int y2 );

	void generateGrid();

	void setColors(cv::Vec3i w, cv::Vec3i g, cv::Vec3i r);

	cv::Mat renderGrid();


};

#endif /* MANTIS_INCLUDE_MANTIS3_GRIDRENDERER_H_ */
