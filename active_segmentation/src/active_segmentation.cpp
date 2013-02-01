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

active_segment::active_segment(cv::Mat input, geometry_msgs::Polygon polygon):
			cluster_graph_(polygon.points.size()){

	input_ = input;
	number_of_vertices_ = polygon.points.size();
	polygon_ = polygon;
}

active_segment::active_segment(ros::NodeHandle & nh):
			nh_(nh),nh_priv_("~"),cluster_graph_(){

	nh_priv_.param<std::string>("static_service",static_service_,std::string("/static_segment_srv"));
	nh_priv_.param<std::string>("window_thread",window_thread_,std::string("Display window"));

	cv::namedWindow( window_thread_.c_str(), CV_WINDOW_AUTOSIZE );// Create a window for display.
	cv::startWindowThread();

}



active_segment::~active_segment(){ cv::destroyWindow(window_thread_.c_str());}

std::pair<double,double> active_segment::findCentroid(int index){

	std::pair<double,double> centroid;
	for(int i = 0;i<number_of_vertices_;i++){
		if(input_.at<int>(polygon_.points[i].x,polygon_.points[i].y) == index){
			centroid.first = polygon_.points[i].x;
			centroid.second = polygon_.points[i].y;
		}
	}

	return centroid;
}

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

cv::Mat active_segment::constructVisGraph(cv::Mat input, graph_module graph){

	graph_module::Graph_it iter;

	cv::Mat draw(input);

	int count=0;
	for(iter = graph.graph_.begin(); iter!=graph.graph_.end(); iter++){

		cv::Point2f start(iter->edge_.first.x_,iter->edge_.first.y_);
		cv::Point2f end(iter->edge_.second.x_,iter->edge_.second.y_);
		ROS_INFO_STREAM("Edge 1: "<<start.x<<","<<start.y<<" Edge 2:"<<end.x<<","<<end.y);
		addLine(draw,start,end);
		count++;
	}


	ROS_INFO("Number of Edges %d",count);
	cv::imwrite("/tmp/full_graph.png",draw);

	return draw;
}

void active_segment::addLine(cv::Mat &image, cv::Point2f start, cv::Point2f end){

	  int thickness = 2;
	  int lineType = 8;
	  // add points and line for each start and end image
	  addCircle(image,start);
	  addCircle(image, end);
	  cv::line(image, start, end, cv::Scalar( 0, 0, 0 ), thickness, lineType );
	  cv::imwrite("/tmp/inter_full_graph.png",image);
}

void active_segment::addCircle(cv::Mat& image, cv::Point2f center){

	cv::circle(image, center, 4, cv::Scalar(128,0,0), -1,8,0);

}

void active_segment::controlGraph(){

	// receive a new graph from
	ROS_INFO("Commencing graph conversion");
	convertToGraph();


	//visualize graph
	ROS_INFO("Displaying Graph");
	cv::Mat view_image = constructVisGraph(input_,cluster_graph_);

	cv::imshow(window_thread_.c_str(), view_image);

	// find the max node
	ROS_INFO("Finding max vertex");
	//Vertex v1 = cluster_graph_.findMaxVertex();

	// do something to move the vertex

}

void active_segment::convertToGraph(){

	//Call static segmentation service
	bool call_succeeded = false;

	while(!call_succeeded){
		if(ros::service::call(static_service_,staticsegment_srv_)){
			call_succeeded = true;
			input_ = returnCVImage(staticsegment_srv_.response.graph_image);
			polygon_ = staticsegment_srv_.response.c_graph.polygon;
			number_of_vertices_ = polygon_.points.size();
			cv::imwrite("/tmp/response_img.png",input_);
		}
	}

	if (staticsegment_srv_.response.result == staticsegment_srv_.response.FAILURE)
	{
		ROS_ERROR("Segmentation service returned error");
		exit(0);
	}

	ROS_INFO("Segmentation service succeeded. Returned Segmented Graph %d",staticsegment_srv_.response.result);

	// create an index map
	std::vector<int> index_map;
	std::vector<int>::iterator index_map_it;

	ROS_INFO("NUmber of vertices %d",number_of_vertices_);

	for(int vals=0;vals<number_of_vertices_;vals++){
		int test_value = (int)input_.at<uchar>((int)polygon_.points[vals].x,(int)polygon_.points[vals].y);
		index_map.push_back(test_value);
	}

	//DEBUG TEST:
	for(int vals=0;vals<index_map.size();vals++)
		ROS_INFO("Index_map values %d",index_map[vals]);

	// DEBUG of projection
	if(staticsegment_srv_.response.result == staticsegment_srv_.response.SUCCESS){

		int count = 0;

		// now loop through image and construct graph from connected components
		int rows = input_.rows, cols = input_.cols;

		for(int i = 0 ; i < rows ; i++)
			for(int j = 0 ; j < cols; j++){

				// if location is not background or not a polygon point
				index_map_it = std::find(index_map.begin(),index_map.end(),(int)input_.at<uchar>(i,j));

				if((int)input_.at<uchar>(i,j) > 0 && index_map_it!=index_map.end()){
					//ROS_INFO("index value : %d",input_.at<int>(i,j));

					// checking for the first pixel in the image
					if(i == 0 && j == 0)
						continue;

					// checking first row and West value
					if(j != 0){
						if(input_.at<int>(i,j) == input_.at<int>(i,j-1)){
							continue;
						}
						else{
							// checking if prev value is not background to avoid connecting with background
							index_map_it = std::find(index_map.begin(),index_map.end(),(int)input_.at<uchar>(i,j-1));

							if((int)input_.at<uchar>(i,j-1) > 0 && index_map_it!=index_map.end()){
								//ROS_INFO("index value prev j-1 : %d",input_.at<int>(i,j-1));

								// first check edge
								Vertex v1,v2;
								v1.index_ = (int)input_.at<uchar>(i,j-1);
								v2.index_ = (int)input_.at<uchar>(i,j);

								if(!cluster_graph_.findEdge(v1,v2)){
									ROS_INFO_STREAM("Adding Edge Between Indices: "<<v1.index_<<" and "<<v2.index_);
									// compute centroids of edges
									std::pair<double,double> c_1 = findCentroid((int)input_.at<uchar>(i,j-1));
									std::pair<double,double> c_2 = findCentroid((int)input_.at<uchar>(i,j));
									v1.x_ = c_1.first; v1.y_ = c_1.second;
									v2.x_ = c_2.first; v2.y_ = c_2.second;

									ROS_INFO_STREAM("Adding Edge Between Vertex: "<<v1.x_<<","<<v1.y_<<" and "<<v2.x_<<","<<v2.y_);
									// inserting edge - initializing all edge weights to zero
									cluster_graph_.addEdge(v1,v2,1);
									count++;
								}
							}
						}
					}

					//checking all other rows (North Value)
					if(i != 0){

						if(input_.at<int>(i,j) == input_.at<int>(i-1,j)){
							continue;
						}
						else{
							// checking if prev value is not background to avoid connecting with background
							index_map_it = std::find(index_map.begin(),index_map.end(),(int)input_.at<uchar>(i-1,j));

							if((int)input_.at<uchar>(i-1,j) > 0 && index_map_it!=index_map.end()){
								//ROS_INFO("index value prev i-1 : %d",input_.at<int>(i-1,j));

								// first check edge
								Vertex v1,v2;
								v1.index_ = (int)input_.at<uchar>(i-1,j);
								v2.index_ = (int)input_.at<uchar>(i,j);

								if(!cluster_graph_.findEdge(v1,v2)){
									ROS_INFO_STREAM("Adding Edge Between Indices: "<<v1.index_<<" and "<<v2.index_);

									// compute centroids of edges
									std::pair<double,double> c_1 = findCentroid((int)input_.at<uchar>(i-1,j));
									std::pair<double,double> c_2 = findCentroid((int)input_.at<uchar>(i,j));
									v1.x_ = c_1.first; v1.y_ = c_1.second;
									v2.x_ = c_2.first; v2.y_ = c_2.second;
									ROS_INFO_STREAM("Adding Edge Between Vertex: "<<v1.x_<<","<<v1.y_<<" and "<<v2.x_<<","<<v2.y_);
									// inserting edge - initializing all edge weights to zero
									cluster_graph_.addEdge(v1,v2,1);
									count++;
								}
							}
						}
					}
				}
			}
		ROS_INFO("Number of accumulated Edges %d",count);
	}
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
