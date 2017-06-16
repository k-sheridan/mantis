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


std::vector<Hypothesis> generateCentralHypotheses(Quadrilateral quad, bool& pass);

/*
 * give this undistorted quads in pixel space
 */
std::vector<Hypothesis> generateHypotheses(std::vector<Quadrilateral> quads, MantisImage img)
				{
	std::vector<Hypothesis> hyps;

	for(auto e : quads)
	{
		bool pass = false;

		std::vector<Hypothesis> central = generateCentralHypotheses(e, pass);

		if(pass)
		{
			hyps.insert(hyps.end(), central.begin(), central.end());
		}

#if SUPER_DEBUG
		//visualizeHypothesis(img.img.clone(), central.front(), e, img.K, img.D);

		//ROS_DEBUG_STREAM("ERROR: " << evaluateHypothesis(central.front(), img));

		//ROS_DEBUG_STREAM("estimate position: " << central.front().getPosition().x() << ", " << central.front().getPosition().y() << ", " << central.front().getPosition().z());

		//ros::Duration sleep(0.1);
		//sleep.sleep();
#endif
	}

	return hyps;
				}

/*
 * generates a hypothesis with the assumption that the quad is at
 * the center of the grid
 */
std::vector<Hypothesis> generateCentralHypotheses(Quadrilateral quad, bool& pass){
	Hypothesis hyp;
	std::vector<Hypothesis> central;
	CoPlanarPoseEstimator pe;

	double error;

	//ROS_DEBUG("estimating pose");
	for(auto e : gridSquarePossibilities)
	{
		hyp.setC2W(pe.estimatePose(quad.test_points, e, error));
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

	//remove for high error
	if(error > MAX_QUAD_ERROR)
	{
		ROS_DEBUG_STREAM("failed with error: " << error);
		pass = false;
		return central;
	}
	else
	{
		pass = true;
	}

	tf::Transform rotZ = tf::Transform(tf::Quaternion(0, 0, 1/sqrt(2), 1/sqrt(2)));

	central.push_back(hyp);
	hyp.setW2C(rotZ * central.back().getW2C());
	central.push_back(hyp);
	hyp.setW2C(rotZ * central.back().getW2C());
	central.push_back(hyp);
	hyp.setW2C(rotZ * central.back().getW2C());
	central.push_back(hyp);



	//ROS_DEBUG("estimated pose");


	//hyp.setW2C(tf::Transform(tf::Quaternion(0, 0, 0, 1)));

	return central;
}

Hypothesis shiftHypothesis(Hypothesis& hyp, tf::Vector3 delta){
	Hypothesis newHyp;

	tf::Transform newW2C = hyp.getW2C();
	newW2C.getOrigin() += delta;

	newHyp.setW2C(newW2C);

	return newHyp;
}

/*
 * shifts the hypotheses to all possible positions
 */
std::vector<Hypothesis> computeAllShiftedHypothesesFAST(Hypothesis hyp){

	std::vector<Hypothesis> final;

	for(double x = -((double)GRID_SIZE/2.0)*GRID_SPACING + ((double)GRID_SPACING/2.0); x < ((double)GRID_SIZE/2.0)*GRID_SPACING; x += GRID_SPACING)
	{
		for(double y = -((double)GRID_SIZE/2.0)*GRID_SPACING + ((double)GRID_SPACING/2.0); y < ((double)GRID_SIZE/2.0)*GRID_SPACING; y += GRID_SPACING)
		{

			final.push_back(shiftHypothesis(hyp, tf::Vector3(x, y, 0)));

		}
	}

	return final;
}


#endif /* MANTIS_INCLUDE_MANTIS3_HYPOTHESISGENERATION_H_ */
