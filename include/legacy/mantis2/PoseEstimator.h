/*
 * PoseEstimator.h
 *
 *  Created on: Jun 8, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS2_POSEESTIMATOR_H_
#define MANTIS_INCLUDE_MANTIS2_POSEESTIMATOR_H_

std::vector<Hypothesis> computeAllCentralHypothesisFAST(Quadrilateral quad, std::vector<cv::Point3d> obj_pts, cv::Mat K);
std::vector<Hypothesis> computeAllQuadHypothesesFAST(Quadrilateral quad, std::vector<cv::Point3d> obj_pts, cv::Mat K);
std::vector<Hypothesis> computeAllCentralHypothesis(Quadrilateral quad, std::vector<std::vector<cv::Point3d>> possibilities, cv::Mat K);
std::vector<Hypothesis> computeAllQuadHypotheses(Quadrilateral quad, std::vector<std::vector<cv::Point3d>> possibilities, cv::Mat K);

double computeHypothesisQuadReprojectionError(std::vector<cv::Point2d> img_pts, std::vector<cv::Point3d> object_pts, cv::Mat K, cv::Mat rvec, cv::Mat tvec){
	std::vector<cv::Point2d> reprojTest;
	cv::projectPoints(object_pts, rvec, tvec, K, cv::noArray(), reprojTest);
	double error = 0;
	for(int i = 0; i < reprojTest.size(); i++)
	{
		error += (reprojTest.at(i)-img_pts.at(i)).ddot((reprojTest.at(i)-img_pts.at(i)));
	}
	return sqrt(error);
}

double computeHypothesisTFReprojectionError(std::vector<cv::Point2d> img_pts, std::vector<cv::Point3d> object_pts, cv::Mat K, Hypothesis hyp)
{
	double error = 0;
	for(int i = 0; i < object_pts.size(); i++)
	{
		cv::Point2d reproj = hyp.projectPoint(tf::Vector3(object_pts.at(i).x, object_pts.at(i).y, object_pts.at(i).z), K);

		error += (reproj-img_pts.at(i)).ddot((reproj-img_pts.at(i)));
	}
	return sqrt(error);
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
	//ROS_DEBUG_STREAM(trans.getOrigin().x() << ", " << trans.getOrigin().y() << ", " << trans.getOrigin().z());

	return trans;
}

/*
 * finds all hypotheses from the detected quads, then determines if there has been a
 * re-observation from the predicted prior
 *
 * quads - detected quads in this frame
 * prior - the old best extrapolated hypotheses
 */
std::vector<Hypothesis> computeHypotheses(std::vector<Quadrilateral> quads, std::vector<Hypothesis> prior, cv::Mat K){

	std::vector<Hypothesis> hypotheses;

	//first compute all possible Hypotheses
	for(auto e : quads)
	{
		std::vector<Hypothesis> more = computeAllQuadHypothesesFAST(e, gridSquare, K);
		//std::vector<Hypothesis> more = computeAllQuadHypotheses(e, gridSquarePossibilities, K);

		//FOR DEBUG ONLY!
		//std::vector<Hypothesis> more = computeAllCentralHypothesis(e, gridSquarePossibilities, K);

		hypotheses.insert(hypotheses.begin(), more.begin(), more.end());
	}

	return hypotheses;
}

//estimate the orientation of the camera with respect to a centered planar square
//
Hypothesis computeHypothesis(std::vector<cv::Point2d> img_pts, std::vector<cv::Point3d> object_pts, cv::Mat K, double& error)
{
	/*
	cv::Mat H = cv::findHomography(img_pts, object_pts);
	std::vector<cv::Mat> Rs;
	std::vector<cv::Mat> ts;
	cv::decomposeHomographyMat(H, K, Rs, ts, cv::noArray());
	 */

	cv::Mat_<double> rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	cv::Mat_<double> tvec = (cv::Mat_<double>(3, 1) << 0, 0, 1);

	cv::solvePnP(object_pts, img_pts, K, cv::noArray(), rvec, tvec, true, POSE_SOLVE_METHOD);

	Hypothesis hyp;

	hyp.setC2W(cvRvecTvec2tfTransform(rvec, tvec));
	//hyp.setW2C(cvRvecTvec2tfTransform(rvec, tvec));

	/*//if the pose is underground reevaluate with no initial guess
	if(hyp.getPosition().z() < 0)
	{
		ROS_DEBUG("trying without initial guess");
		cv::solvePnP(object_pts, img_pts, K, cv::noArray(), rvec, tvec, false, POSE_SOLVE_METHOD);
		hyp.setC2W(cvRvecTvec2tfTransform(rvec, tvec));
	}*/

	error = computeHypothesisQuadReprojectionError(img_pts, object_pts, K, rvec, tvec);

	ROS_DEBUG_STREAM("Hypothesis Error: " << error << " POS: " << hyp.getPosition().x()<<", "<<hyp.getPosition().y()<<", "<<hyp.getPosition().z());
	ROS_DEBUG_STREAM("TF error: " << computeHypothesisTFReprojectionError(img_pts, object_pts, K, hyp));

#if SUPER_DEBUG
	/*//ROS_DEBUG_STREAM("solution ");
	//ROS_DEBUG_STREAM("		rvec: " << rvec);
	//ROS_DEBUG_STREAM("		tvec: " << tvec);

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
	sleep.sleep();*/

#endif

	//set one observation;
	hyp.observations = 1;

	return hyp;
}

/*
 * this method computes one hypothesis then rotates it 3 more times
 */
std::vector<Hypothesis> computeAllCentralHypothesisFAST(Quadrilateral quad, std::vector<cv::Point3d> obj_pts, cv::Mat K){
	//setup 2d points
	std::vector<cv::Point2d> img_pts;
	for(auto e : quad.test_points)
	{
		img_pts.push_back(cv::Point2d(e.x, e.y));
	}

	tf::Transform rotZ = tf::Transform(tf::Quaternion(0, 0, 1/sqrt(2), 1/sqrt(2)));

	std::vector<Hypothesis> hyps;

	double error;
	hyps.push_back(computeHypothesis(img_pts, obj_pts, K, error));

	if(error > MAX_QUAD_ERROR)
	{
		hyps.clear();

		ROS_DEBUG_STREAM("error too high returned vector with size " << hyps.size());
		return hyps;
	}

	Hypothesis hyp;
	hyp.setW2C(rotZ * hyps.back().getW2C());
	hyps.push_back(hyp);
	hyp.setW2C(rotZ * hyps.back().getW2C());
	hyps.push_back(hyp);
	hyp.setW2C(rotZ * hyps.back().getW2C());
	hyps.push_back(hyp);


	return hyps;
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
std::vector<Hypothesis> computeAllQuadHypothesesFAST(Quadrilateral quad, std::vector<cv::Point3d> obj_pts, cv::Mat K){

	std::vector<Hypothesis> final, central;

	central = computeAllCentralHypothesisFAST(quad, obj_pts, K); // get the central hypotheses

	if(central.size() == 0)
	{
		return final;
	}

	//testing
	final.push_back(central.at(0));

	/*for(double x = -((double)GRID_SIZE/2.0)*GRID_SPACING + ((double)GRID_SPACING/2.0); x < ((double)GRID_SIZE/2.0)*GRID_SPACING; x += GRID_SPACING)
	{
		for(double y = -((double)GRID_SIZE/2.0)*GRID_SPACING + ((double)GRID_SPACING/2.0); y < ((double)GRID_SIZE/2.0)*GRID_SPACING; y += GRID_SPACING)
		{
			for(auto& e : central)
			{
				final.push_back(shiftHypothesis(e, tf::Vector3(x, y, 0)));
			}
		}
	}*/

	return final;
}

std::vector<Hypothesis> computeAllQuadHypotheses(Quadrilateral quad, std::vector<std::vector<cv::Point3d>> possibilities, cv::Mat K){

	std::vector<Hypothesis> final, central;

	central = computeAllCentralHypothesis(quad, possibilities, K); // get the central hypotheses

	for(double x = -((double)GRID_SIZE/2.0)*GRID_SPACING + ((double)GRID_SPACING/2.0); x < ((double)GRID_SIZE/2.0)*GRID_SPACING; x += GRID_SPACING)
	{
		for(double y = -((double)GRID_SIZE/2.0)*GRID_SPACING + ((double)GRID_SPACING/2.0); y < ((double)GRID_SIZE/2.0)*GRID_SPACING; y += GRID_SPACING)
		{
			for(auto& e : central)
			{
				final.push_back(shiftHypothesis(e, tf::Vector3(x, y, 0)));
			}
		}
	}

	return final;
}

std::vector<Hypothesis> computeAllCentralHypothesis(Quadrilateral quad, std::vector<std::vector<cv::Point3d>> possibilities, cv::Mat K){
	//setup 2d points
	std::vector<cv::Point2d> img_pts;
	for(auto e : quad.test_points)
	{
		img_pts.push_back(cv::Point2d(e.x, e.y));
	}

	std::vector<Hypothesis> hyps;

	double error;

	for(auto e : possibilities)
	{
		hyps.push_back(computeHypothesis(img_pts, e, K, error));
		if(error > MAX_QUAD_ERROR)
		{
			hyps.pop_back();
			ROS_DEBUG_STREAM("removed hypo with too high error");
		}
	}

	return hyps;
}

#endif /* MANTIS_INCLUDE_MANTIS2_POSEESTIMATOR_H_ */
