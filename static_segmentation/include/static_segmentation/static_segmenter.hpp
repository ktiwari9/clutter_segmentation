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
 * @b Combines tabletop segmenter and Felzenswalbs graph based segmenter
 */
#pragma once
#ifndef STATIC_SEGMENTER_HPP
#define STATIC_SEGMENTER_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include "static_segmentation/StaticSegment.h"
#include "static_segmentation/StaticSeg.h"
#include "tabletop_segmenter/TabletopSegmentation.h"
#include "graph_based_segmentation/GraphSegment.h"
#include "graph_module/EGraph.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>
#include <usc_utilities/assert.h>
#include <graph_module/graph_module.hpp>
#include <geometry_msgs/Point.h>

namespace static_segmentation {

struct graph_node{

	int index_;
	cv::MatND hist_;
	double x_,y_;
};

struct find_node{

	int id;
	find_node(int id) : id(id){}
	bool operator()(const graph_node& g) const
	{
		return g.index_ == id;
	}
};

class static_segment{

public:
	typedef std::vector<graph_node> local_graph;
	typedef std::vector<graph_node>::iterator local_graph_it;

protected:

	ros::NodeHandle nh_;

	cv::Mat input_;

	std::string tabletop_service_,graph_service_,rgb_topic_,camera_topic_;

	tabletop_segmenter::TabletopSegmentation tabletop_srv_;

	graph_based_segmentation::GraphSegment graphsegment_srv_;

	tf::TransformListener listener_;

	sensor_msgs::CameraInfo cam_info_;

	double threshold_;

	local_graph node_list_;

	std::vector<local_graph> graph_list_;

	local_graph old_node_list_;

	std::vector<local_graph> old_graph_list_;

	std::vector<cv::Point2f> graph_centroid_;

	//local_graph_it node_it_;


public:

	static_segment(ros::NodeHandle &nh);

	~static_segment();

	bool serviceCallback(StaticSegment::Request &request, StaticSegment::Response &response);

	std::vector<StaticSeg> computeCGraph(sensor_msgs::ImagePtr &return_image);

	void getMasksFromClusters(const std::vector<sensor_msgs::PointCloud2> &clusters,
			const sensor_msgs::CameraInfo &cam_info,
			std::vector<sensor_msgs::Image> &masks);

	cv::Mat returnCVImage(const sensor_msgs::Image & img);

	geometry_msgs::Point32 createPoint32(double x, double y, double z);

	cv::MatND computePatchFeature(cv::Mat input, cv::Mat mask);

	graph_module::EGraph buildEGraph(std::vector<graph_node> node_list, cv::Mat segment);

	void addEdge(local_graph_it it_1, local_graph_it it_2, graph::ros_graph& graph);

	void updateOldNodeList(std::vector<graph_module::EGraph> in_graph);

	void updateNewNodeList();

	double compareDescriptor(cv::MatND hist_orig, cv::MatND hist_new);

private:

	ros::NodeHandle nh_priv_;

	ros::ServiceServer static_segment_srv_;

};

}

#endif

