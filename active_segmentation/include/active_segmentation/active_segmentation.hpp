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
 * @b computes connectivity graph for static segmented connected components
 */

#ifndef ACTIVE_SEGMENTATION_HPP
#define ACTIVE_SEGMENTATION_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "static_segmentation/StaticSegment.h"
#include <geometry_msgs/Polygon.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include "graph_module/EGraph.h"

// boost graph library includes
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/undirected_graph.hpp>

using namespace std;

namespace active_segmentation {

class active_segment{

protected:

	ros::NodeHandle nh_;

	cv::Mat input_;

	geometry_msgs::Polygon polygon_;

	int number_of_vertices_;

	std::string static_service_,rgb_topic_,camera_topic_,window_thread_;

	tf::TransformListener listener_;

	sensor_msgs::CameraInfo cam_info_;

	static_segmentation::StaticSegment staticsegment_srv_;

	graph_module cluster_graph_;

public:

	active_segment(ros::NodeHandle &nh);

	//overlaod constructor for non ROS Declaration
	active_segment(cv::Mat input, graph_module::EGraph graph);

	~active_segment();

	void convertToGraph();

	cv::Mat returnCVImage(const sensor_msgs::Image & img);

	std::pair<double,double> findCentroid(int index);

	cv::Mat constructVisGraph(cv::Mat input_image, graph_module graph);

	void addLine(cv::Mat &image, cv::Point2f start, cv::Point2f end);

	void addCircle(cv::Mat &image, cv::Point2f center);

	void controlGraph();

private:

	ros::NodeHandle nh_priv_;

};

}

#endif
