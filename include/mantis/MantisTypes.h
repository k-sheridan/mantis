/*
 * MantisTypes.h
 *
 *  Created on: Jun 6, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS_MANTISTYPES_H_
#define MANTIS_INCLUDE_MANTIS_MANTISTYPES_H_

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

struct Particle{
	tf::Transform trans;
	double weight;
};



#endif /* MANTIS_INCLUDE_MANTIS_MANTISTYPES_H_ */
