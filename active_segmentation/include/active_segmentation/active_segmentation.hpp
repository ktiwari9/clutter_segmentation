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
// CPP includes
#include <iostream>
#include <vector>
#include <functional>
#include <queue>
// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Polygon.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
// Static Segment includes
#include "static_segmentation/StaticSegment.h"
#include "static_segmentation/StaticSeg.h"
#include "static_segmentation/static_segmenter.hpp"
//Graph Module includes
#include "graph_module/EGraph.h"
#include <graph_module/graph_module.hpp>
// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// Arm trajectory client
#include <arm_controller_interface/joint_trajectory_client.h>

namespace active_segmentation {

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

struct local_graph{

	graph::ros_graph graph_;
	geometry_msgs::Point centroid_;
};

struct compare_graph : public std::binary_function<local_graph, local_graph, bool>
{
    bool operator()(const local_graph lhs, const local_graph rhs) const
    {
        //return lhs.graph_.graph_.size() < rhs.graph_.graph_.size();
    	return lhs.graph_.number_of_vertices_ < rhs.graph_.number_of_vertices_;
    }
};

/*
 * Temporary container class which helps iterate over the priority queue
 */
template <class T, class S, class C>
    S& Container(std::priority_queue<T, S, C>& q) {
        struct HackedQueue : private std::priority_queue<T, S, C> {
            static S& Container(std::priority_queue<T, S, C>& q) {
                return q.*&HackedQueue::c;
            }
        };
    return HackedQueue::Container(q);
}


class active_segment{

private:

	ros::NodeHandle nh_priv_;

public:

	typedef std::priority_queue<local_graph,std::vector<local_graph>,compare_graph> graph_queue;

	geometry_msgs::Point push_loc_;

	arm_controller_interface::HeadJointTrajectoryClient head_joint_trajectory_client_;

	bool first_call_;

	cv::Mat input_,prev_input_,segment_,prev_segment_;

protected:

	ros::NodeHandle nh_;

	graph::ros_graph cluster_graph_;

	int number_of_vertices_;

	std::string static_service_,rgb_topic_,camera_topic_,window_thread_,left_camera_topic_,tabletop_service_;

	tf::TransformListener listener_;

	sensor_msgs::CameraInfo cam_info_;

	image_geometry::PinholeCameraModel left_cam_;

	static_segmentation::StaticSegment staticsegment_srv_;

	graph_queue graph_list_;

	std::vector<local_graph> graph_iter_,cluster_iter_;

	std::vector<static_segmentation::StaticSeg> graph_msg_;

	bool tracking_,got_table_;

	pcl::ModelCoefficients::Ptr table_coefficients_;

	// Optical flow parameters
	float pyramid_scale_ = 0.5; // Pyramid scales

	int levels_ = 3, win_size_ = 11,of_iter_ = 5;
	// Number of Pyramid levels,window size, number of iterations

	double poly_pixel_ = 6,poly_sigma_ = 1.25;
	//size of pixel neighbourhood and std dev of gaussian

public:

	active_segment(ros::NodeHandle &nh);

	//overload constructor for non ROS Declaration
	active_segment(cv::Mat input, cv::Mat segment, graph_module::EGraph graph);

	~active_segment();

	void initGraphHelpers();

	bool convertToGraph();

	cv::Mat returnCVImage(const sensor_msgs::Image & img);

	std::pair<double,double> findCentroid(int index);

	cv::Mat constructVisGraph(cv::Mat input_image);

	void constructVisGraph();

	void addLine(cv::Mat &image, cv::Point2f start, cv::Point2f end);

	void addCircle(cv::Mat &image, cv::Point2f center);

	void controlGraph();

	void TrackandUpdate();

	void getPushPoint(pcl::PointCloud<pcl::PointXYZ> push_ray,
			geometry_msgs::Point &push_loc);

	cv::MatND computePatchFeature(cv::Mat input, cv::Mat mask);

	cv::MatND getFeatureVector(cv::Mat input, cv::Mat mask, int index);

	void updateGraphList(cv::Mat flow);

	// Matches an edge from the old subgraph with an edge in the new subgraph
	bool matchEdges(std::pair<int,int> old_edge,std::pair<int,int> new_edge);

	void updateWithNewGraph();

	void buildGraphFromMsg(graph_queue& graph_list);

	int findNearestNeighbour(geometry_msgs::Point vert,std::vector<local_graph> new_graph_iter);

	bool matchGraphs(local_graph base_graph,local_graph match_graph);

	void projectVertex3DBASE(graph::Vertex_ros point,pcl::PointCloud<pcl::PointXYZ> &ray);

};

}

#endif
