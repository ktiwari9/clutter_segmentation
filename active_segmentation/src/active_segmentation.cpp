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

#include "graph_module/EGraph.h"
#include "active_segmentation/active_segmentation.hpp"

namespace active_segmentation {

active_segment::active_segment(cv::Mat input, cv::Mat segment, graph_module::EGraph graph){

	segment_ = segment;
	input_ = input;
	graph_msg_ = graph;
}

active_segment::active_segment(ros::NodeHandle & nh):
			nh_(nh),nh_priv_("~"){

	nh_priv_.param<std::string>("static_service",static_service_,std::string("/static_segment_srv"));
	nh_priv_.param<std::string>("window_thread",window_thread_,std::string("Display window"));
	nh_priv_.param<bool>("tracking",tracking_,false);
	nh_priv_.param<std::string>("left_camera_topic",left_camera_topic_,std::string("/Honeybee/left/camera_info"));

	cv::namedWindow( window_thread_.c_str(), CV_WINDOW_AUTOSIZE );// Create a window for display.
	cv::startWindowThread();

}



active_segment::~active_segment(){ cv::destroyWindow(window_thread_.c_str());}

cv::Mat active_segment::returnCVImage(const sensor_msgs::Image & img) {

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img);
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		exit(0);
	}

	return cv_ptr->image;
}

cv::Mat active_segment::constructVisGraph(cv::Mat input, graph::ros_graph graph){

	cv::Mat draw(input);

	int count=0;
	ROS_INFO("Finding Max Vertex");
	graph::Vertex_ros push_vertex = cluster_graph_.findMaxVertex();

	ROS_DEBUG("Max Vertex index %d",push_vertex.index_);
	for(graph::ros_graph::IGraph_it iter_= graph.graph_.begin(); iter_!=graph.graph_.end(); ++iter_){

		// First point
		graph::Edge_ros edge = *iter_;
		cv::Point center_1(edge.edge_.first.x_,edge.edge_.first.y_);
		// Second point
		cv::Point center_2(edge.edge_.second.x_,edge.edge_.second.y_);

		// Display shenanigans
		cv::circle(draw,center_1, 5, cv::Scalar(128,0,128), -1);
		cv::circle(draw,center_2, 5, cv::Scalar(128,0,128), -1);
		cv::line(draw,center_1,center_2,cv::Scalar(128,0,0),1,0);

		if(push_vertex.index_ != edge.edge_.first.index_)
			cv::putText(draw, boost::lexical_cast<string>(edge.edge_.first.index_), center_1,
				CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
		else
			cv::putText(draw, " Max Vertex: "+boost::lexical_cast<string>(edge.edge_.first.index_), center_1,
				CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

		if(push_vertex.index_ != edge.edge_.second.index_)
			cv::putText(draw, boost::lexical_cast<string>(edge.edge_.second.index_), center_2,
				CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
		else
			cv::putText(draw, " Max Vertex: "+ boost::lexical_cast<string>(edge.edge_.second.index_), center_2,
				CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

		count++;
	}


	ROS_INFO("Number of Edges %d",count);
	if(input_.empty())
		cv::imwrite("/tmp/full_graph.png",draw);

	return draw;
}

void active_segment::constructVisGraph(){

	cv::Mat disp_image = constructVisGraph(input_,cluster_graph_);
	if(!disp_image.empty())
		cv::imshow(window_thread_.c_str(), disp_image);
}

bool active_segment::pushAndTrack(){


	return true;

}

void active_segment::controlGraph(){

	// receive a new graph from
	if(convertToGraph()){

		ROS_INFO("Calling Static Segment service");
		//visualize graph
		ROS_INFO("Displaying Graph");
		constructVisGraph();

		// find the max node
		ROS_INFO("Pushing max vertex");
		graph::Vertex_ros v1 = cluster_graph_.findMaxVertex();

		// Project Vertex to 3D
		cv::Point2d push_2d(v1.x_,v1.y_);
		cv::Point3d push_3d = left_cam_.projectPixelTo3dRay(push_2d); // getting push location in 3D

		//Now push and track;

	}
}

bool active_segment::convertToGraph(){

	//Call static segmentation service
	bool call_succeeded = false;

	if(tracking_){
		// Do something with the tracking result
	}
	else{
		staticsegment_srv_.request.call = staticsegment_srv_.request.EMPTY;
	}

	while(!call_succeeded){
		if(ros::service::call(static_service_,staticsegment_srv_)){
			call_succeeded = true;

			//getting camera info
			sensor_msgs::CameraInfo::ConstPtr cam_info =
				    ros::topic::waitForMessage<sensor_msgs::CameraInfo>(left_camera_topic_, nh_, ros::Duration(5.0));

			left_cam_.fromCameraInfo(cam_info); // Getting left camera info

			input_ = returnCVImage(staticsegment_srv_.response.graph_image);

			graph_msg_ = staticsegment_srv_.response.out_graph;

			bool result = cluster_graph_.buildGraph(graph_msg_);

			cv::imwrite("/tmp/response_img.png",input_);
		}
	}

	if (staticsegment_srv_.response.result == staticsegment_srv_.response.FAILURE)
	{
		ROS_ERROR("Segmentation service returned error");
		return false;
	}

	ROS_INFO("Segmentation service succeeded. Returned Segmented Graph %d",staticsegment_srv_.response.result);

	// DEBUG of projection
	if(staticsegment_srv_.response.result == staticsegment_srv_.response.SUCCESS){
		ROS_INFO("Static Segment service call succeeded");
		return true;
	}

	return false;
}
}

int run_active_segmentation(int argc, char **argv){

	ros::init(argc, argv, "active_segment");
	ros::NodeHandle nh;
	active_segmentation::active_segment ss(nh);

	while(nh.ok()){

		ss.controlGraph();
		ros::spinOnce();
	}

	return 0;
}

int main(int argc, char **argv){

	return run_active_segmentation(argc,argv);

}
