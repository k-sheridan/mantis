/*
 * PoseEstimator.h
 *
 *  Created on: Jun 8, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS2_POSEESTIMATOR_H_
#define MANTIS_INCLUDE_MANTIS2_POSEESTIMATOR_H_

double computeHypothesisQuadReprojectionError(std::vector<cv::Point2d> img_pts, std::vector<cv::Point3d> object_pts, cv::Mat K, cv::Mat rvec, cv::Mat tvec)
{
	std::vector<cv::Point2d> reprojTest;
	cv::projectPoints(object_pts, rvec, tvec, K, cv::noArray(), reprojTest);
	double error = 0;
	for(int i = 0; i < reprojTest.size(); i++)
	{
		error += (reprojTest.at(i)-img_pts.at(i)).ddot((reprojTest.at(i)-img_pts.at(i)));
	}
	return sqrt(error);
}

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
	cv::Mat_<double> rot;
	cv::Rodrigues(rvec, rot);
	/*ROS_DEBUG_STREAM("rot: " << rot);
	ROS_DEBUG_STREAM("rvec: " << rvec);*/
	//ROS_DEBUG_STREAM("tvec " << tvec);

	tf::Transform trans;

	trans.getBasis().setValue(rot(0), rot(1), rot(2), rot(3), rot(4), rot(5), rot(6), rot(7), rot(8));
	trans.setOrigin(tf::Vector3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));

	//ROS_DEBUG_STREAM("rot: " << trans.getRotation().w() << ", " << trans.getRotation().x() << ", " << trans.getRotation().y() << ", " << trans.getRotation().z());
	/*double x, y, z;
	trans.getBasis().getRPY(x, y, z);
	ROS_DEBUG_STREAM("tf rvec " << x <<", "<<y<<", "<<z);*/
	ROS_DEBUG_STREAM(trans.getOrigin().x() << ", " << trans.getOrigin().y() << ", " << trans.getOrigin().z());

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
	cv::solvePnP(object_pts, img_pts, K, cv::noArray(), rvec, tvec, false, POSE_SOLVE_METHOD);

	Hypothesis hyp;

	hyp.setC2W(cvRvecTvec2tfTransform(rvec, tvec));
	//hyp.setW2C(cvRvecTvec2tfTransform(rvec, tvec));

#if SUPER_DEBUG
	//ROS_DEBUG_STREAM("solution ");
	//ROS_DEBUG_STREAM("		rvec: " << rvec);
	//ROS_DEBUG_STREAM("		tvec: " << tvec);
	ROS_DEBUG_STREAM("Hypothesis Error: " << computeHypothesisQuadReprojectionError(img_pts, object_pts, K, rvec, tvec));

	//test both reprojections
	cv::Mat test = cv::Mat::zeros(300, 300, CV_8UC3);
	std::vector<cv::Point2d> reproj;
	cv::projectPoints(object_pts, rvec, tvec, K, cv::noArray(), reproj);
	for(auto e : reproj)
	{
		cv::drawMarker(test, e, cv::Scalar(0, 255, 0), cv::MARKER_DIAMOND);
	}
	for(auto e : object_pts)
	{
		cv::drawMarker(test, hyp.projectPoint(tf::Vector3(e.x, e.y, e.z), K), cv::Scalar(255, 255, 255), cv::MARKER_SQUARE);
	}

	cv::imshow("hypo check", test);
	cv::waitKey(30);

	ros::Duration sleep(2);
	sleep.sleep();

#endif

	return hyp;
}

/*
 * this method computes one hypothesis then rotates it 3 more times
 */
std::vector<Hypothesis> computeAllCentralHypothesisFAST(Quadrilateral quad, std::vector<cv::Point3d> obj_pts, cv::Mat K)
										{
	//setup 2d points
	std::vector<cv::Point2d> img_pts;
	for(auto e : quad.test_points)
	{
		img_pts.push_back(cv::Point2d(e.x, e.y));
	}

	tf::Transform rotZ = tf::Transform(tf::Quaternion(0, 0, 1/sqrt(2), 1/sqrt(2)));

	std::vector<Hypothesis> hyps;

	hyps.push_back(computeHypothesis(img_pts, obj_pts, K));
	Hypothesis hyp;
	hyp.setW2C(rotZ * hyps.back().getW2C());
	hyps.push_back(hyp);
	hyp.setW2C(rotZ * hyps.back().getW2C());
	hyps.push_back(hyp);
	hyp.setW2C(rotZ * hyps.back().getW2C());
	hyps.push_back(hyp);


	return hyps;
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
