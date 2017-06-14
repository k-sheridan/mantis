/*
 * HypothesisGeneration.h
 *
 *  Created on: Jun 12, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_HYPOTHESISGENERATION_H_
#define MANTIS_INCLUDE_MANTIS3_HYPOTHESISGENERATION_H_

#include "CoPlanarPoseEstimator.h"

#include "Mantis3Params.h"

#include "HypothesisEvaluation.h"


Hypothesis generateCentralHypothesis(Quadrilateral quad, bool& pass);

/*
 * give this undistorted quads in pixel space
 */
std::vector<Hypothesis> generateHypotheses(std::vector<Quadrilateral> quads, MantisImage img)
{
	std::vector<Hypothesis> hyps;

	for(auto e : quads)
	{
		bool pass = false;

		Hypothesis hyp = generateCentralHypothesis(e, pass);

		if(pass)
		{
			hyps.push_back(hyp);
		}

#if SUPER_DEBUG
		visualizeHypothesis(img.img.clone(), hyp, e, img.K, img.D);

		ROS_DEBUG_STREAM("estimate position: " << hyp.getPosition().x() << ", " << hyp.getPosition().y() << ", " << hyp.getPosition().z());

		ros::Duration sleep(1);
		sleep.sleep();
#endif
	}

	return hyps;
}

/*
 * generates a hypothesis with the assumption that the quad is at
 * the center of the grid
 */
Hypothesis generateCentralHypothesis(Quadrilateral quad, bool& pass)
{
	Hypothesis hyp;
	CoPlanarPoseEstimator pe;

	//ROS_DEBUG("estimating pose");
	for(auto e : gridSquarePossibilities)
	{
		hyp.setC2W(pe.estimatePose(quad.test_points, e));
		if(hyp.getPosition().z() >= 0)
		{
			ROS_DEBUG_STREAM("FOUND GOOD POSE");
			break;
		}
		else
		{
			ROS_DEBUG_STREAM("FOUND BAD POSE");
		}
	}

	//ROS_DEBUG("estimated pose");

	pass = true;

	//hyp.setW2C(tf::Transform(tf::Quaternion(0, 0, 0, 1)));

	return hyp;
}


#endif /* MANTIS_INCLUDE_MANTIS3_HYPOTHESISGENERATION_H_ */
