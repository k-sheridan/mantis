/*
 * CoPlanarPoseEstimator.h
 *
 *  Created on: Jun 12, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_COPLANARPOSEESTIMATOR_H_
#define MANTIS_INCLUDE_MANTIS3_COPLANARPOSEESTIMATOR_H_

#include "RobustPlanarPose/RPP.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include "Mantis3Types.h"

class CoPlanarPoseEstimator {
public:
	CoPlanarPoseEstimator();
	//virtual ~CoPlanarPoseEstimator();

	tf::Transform estimatePose(std::vector<cv::Point2d> img_pts, std::vector<tf::Vector3> obj_pts, double& img_err);

};

#endif /* MANTIS_INCLUDE_MANTIS3_COPLANARPOSEESTIMATOR_H_ */
