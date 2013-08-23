/*********************************************************************
*
*  Copyright (c) 2012, Computational Learning and Motor Control Laboratory
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
 * @b OpenCV wrapper for Pedro felzenbswalb's Graph based Segmentation
 */
#pragma once
#ifndef SEGMENT
#define SEGMENT

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <graph_based_segmentation/image.h>
#include <graph_based_segmentation/misc.h>
#include <graph_based_segmentation/pnmfile.h>
#include "graph_based_segmentation/segment-image.h"
#include "graph_based_segmentation/GraphSegment.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;

typedef struct {
    int x;
    int y;
}Pixel;

namespace graph_based_segmentation {

class graph_segment{

protected:

	ros::NodeHandle nh_;

	cv::Mat input_;

	double sigma_,k_;

	int min_size_,num_ccs_;

public:

	graph_segment();

	graph_segment(cv::Mat input);

	graph_segment(ros::NodeHandle &nh);

	~graph_segment();

	cv::Mat convertNativetocvMat(image<rgb>* input);

	image<rgb>* convertcvMattoNative(cv::Mat input);

	cv::Mat getSegmentedImage();

	void setCVImage(cv::Mat input){input_ = input;}

	bool serviceCallback(GraphSegment::Request &request, GraphSegment::Response& response);

	cv::Mat returnCVImage(const sensor_msgs::Image & img);

	sensor_msgs::ImagePtr returnRosImage(const sensor_msgs::Image rgb_image);

private:

	ros::NodeHandle nh_priv_;

	ros::ServiceServer graph_segment_srv_;

};

}

#endif

