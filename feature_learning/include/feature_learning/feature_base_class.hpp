/*********************************************************************
*
*  Copyright (c) 2013, Computational Learning and Motor Control Laboratory
*  University of Southern California
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************
/**
 * \author Bharath Sankaran
 *
 * @b header file for the base feature class for feature learning via MaxEnt
 */

#ifndef FEATURE_BASE_CLASS_HPP
#define FEATURE_BASE_CLASS_HPP

// CPP includes
#include <iostream>
#include <vector>
#include <functional>
#include <queue>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iterator>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Polygon.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

//Graph Module includes
#include "graph_module/EGraph.h"
#include <graph_module/graph_module.hpp>


namespace feature_learning {

class feature_class{

protected:

	cv::Mat local_image_,local_segment_;

public:

	// constructor for the feature class
	feature_class();

	// destructor for the feature class
	~feature_class();

	// Getting the input from the user, function overloaded for
	// raw color image and superpixel segment image
	void inputImage(cv::Mat input);

	void inputImage(cv::Mat input,cv::Mat segment);

	// Computing the features required for learning
	template <class Derived>
	void computeColorHist(Eigen::MatrixBase<Derived> &out_mat );

	template <class Derived>
	void computeTextonMap(Eigen::MatrixBase<Derived> &out_mat );

	template <class Derived>
	void computeEntropyMap(Eigen::MatrixBase<Derived> &out_mat );

	template <class Derived>
	void computePushFeature(Eigen::MatrixBase<Derived> &out_mat );

	template <class Derived>
	void computeGraspPatch(Eigen::MatrixBase<Derived> &out_mat );

	// Code snippet example
	//	cv::Mat_<float> a = Mat_<float>::ones(2,2);
	//	Eigen::Matrix<float,Dynamic,Dynamic> b;
	//	cv2eigen(a,b);
	// OR
	// MatrixXd m = Eigen::Map<MatrixXd>(reinterpret_cast<double*>(a.data),a.rows,a.cols);

};

}

#endif
