/*
 * PoseAdjustment.h
 *
 *  Created on: Jun 14, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_POSEADJUSTMENT_H_
#define MANTIS_INCLUDE_MANTIS3_POSEADJUSTMENT_H_

#include "mantis3/Mantis3Params.h"

Hypothesis generateRandomHypothesis(Hypothesis hyp){
	tf::Matrix3x3 rot;
	rot.setRPY(rng.gaussian(ROT_SIGMA), rng.gaussian(ROT_SIGMA), rng.gaussian(ROT_SIGMA));
	tf::Transform rand = tf::Transform(rot, tf::Vector3(rng.gaussian(TRANS_SIGMA), rng.gaussian(TRANS_SIGMA), rng.gaussian(TRANS_SIGMA)));

	Hypothesis out = hyp;

	out.setW2C(hyp.getW2C() * rand);

	return out;
}

/*
 * runs a particle filter like algorithm with a best particle re-sampling step
 * may be replaced with direct image alignment methods ie Lucas-Kanade-Tomasi
 */
Hypothesis optimizeHypothesisWithParticleFilter(Hypothesis in, MantisImage img, int particles = 50, int iterations = 10)
{
	Hypothesis current_best = in;
	evaluateOneHypothesis(current_best, img, true);
	//visualizeHypothesis(img.img.clone(), current_best, img.K, img.D);

	ROS_DEBUG_STREAM("ITERATION 0 ERROR : " << current_best.error);

	for(int i = 0; i < iterations; i++)
	{
		Hypothesis sampleParticle = current_best;

		for(int j = 0; j < particles; j++)
		{
			Hypothesis temp = generateRandomHypothesis(sampleParticle);

			evaluateOneHypothesis(temp, img, true);

			if(temp.error < current_best.error)
			{
				current_best = temp;
			}
		}

		ROS_DEBUG_STREAM("ITERATION " << i+1 << " ERROR : " << current_best.error);
		//visualizeHypothesis(img.img.clone(), current_best, img.K, img.D);

	}

	return current_best;

}



#endif /* MANTIS_INCLUDE_MANTIS3_POSEADJUSTMENT_H_ */
