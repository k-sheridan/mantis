/*
 * PoseEstimator.h
 *
 *  Created on: Jun 8, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS2_POSEESTIMATOR_H_
#define MANTIS_INCLUDE_MANTIS2_POSEESTIMATOR_H_


std::vector<std::vector<cv::Point3d>> generatePossibleOrientations(double grid_spacing)
{
	std::vector<std::vector<cv::Point3d>> possibilities; // possibility possibilities
	std::vector<cv::Point3d> possibility;
	cv::Point3d temp;

	possibility.push_back(cv::Point3d(grid_spacing/2, grid_spacing/2, 0));
	possibility.push_back(cv::Point3d(-grid_spacing/2, grid_spacing/2, 0));
	possibility.push_back(cv::Point3d(-grid_spacing/2, -grid_spacing/2, 0));
	possibility.push_back(cv::Point3d(grid_spacing/2, -grid_spacing/2, 0));

	possibilities.push_back(possibility);

	std::rotate(possibility.rbegin(), possibility.rbegin() + 1, possibility.rend());
	possibilities.push_back(possibility);

	std::rotate(possibility.rbegin(), possibility.rbegin() + 1, possibility.rend());
	possibilities.push_back(possibility);

	std::rotate(possibility.rbegin(), possibility.rbegin() + 1, possibility.rend());
	possibilities.push_back(possibility);

	return possibilities;
}

tf::Transform cvRvecTvec2tfTransform(cv::Mat rvec, cv::Mat tvec)
{
	/*cv::Mat rot;
	cv::Rodrigues(rvec, rot);
	ROS_DEBUG_STREAM("rot: " << rot);*/

	tf::Transform trans;

	trans.getBasis().setRPY(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
	trans.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

	//ROS_DEBUG_STREAM("quat: " << trans.getRotation().w() << ", " << trans.getRotation().x() << ", " << trans.getRotation().y() << ", " << trans.getRotation().z());

	return trans;
}

/*
 * finds all hypotheses from the detected quads, then determines if there has been a
 * re-observation from the predicted prior
 *
 * quads - detected quads in this frame
 * prior - the old best extrapolated hypotheses
 */
std::vector<Hypothesis> computeHypotheses(std::vector<Quadrilateral> quads, std::vector<Hypothesis> prior){

}

//estimate the orientation of the camera with respect to a centered planar square
//
Hypothesis computeHypothesis(std::vector<cv::Point2d> img_pts, std::vector<cv::Point3d> object_pts, cv::Mat K)
{
	/*
	cv::Mat H = cv::findHomography(img_pts, object_pts);
	std::vector<cv::Mat> Rs;
	std::vector<cv::Mat> ts;
	cv::decomposeHomographyMat(H, K, Rs, ts, cv::noArray());
	 */
	cv::Mat rvec;
	cv::Mat tvec;
	cv::solvePnP(object_pts, img_pts, K, cv::noArray(), rvec, tvec);
	//ROS_DEBUG_STREAM("rvec: " << rvec);
	//ROS_DEBUG_STREAM("tvec: " << tvec);

	Hypothesis hyp;

	hyp.setC2W(cvRvecTvec2tfTransform(rvec, tvec));

	return hyp;
}

std::vector<Hypothesis> computeAllCentralHypothesis(Quadrilateral quad, std::vector<std::vector<cv::Point3d>> possibilities, cv::Mat K)
						{
	//setup 2d points
	std::vector<cv::Point2d> img_pts;
	for(auto e : quad.test_points)
	{
		img_pts.push_back(cv::Point2d(e.x, e.y));
	}

	std::vector<Hypothesis> hyps;

	for(auto e : possibilities)
	{
		hyps.push_back(computeHypothesis(img_pts, e, K));
	}

	return hyps;
						}

#endif /* MANTIS_INCLUDE_MANTIS2_POSEESTIMATOR_H_ */
