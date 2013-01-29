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

#include "active_segmentation/graph_module.hpp"
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

}



active_segment::~active_segment(){}

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

	// DEBUG of projection
	if(staticsegment_srv_.response.result == staticsegment_srv_.response.SUCCESS){

		// now loop through image and construct graph from connected components
		int rows = input_.rows, cols = input_.cols;

		for(int i = 0 ; i < rows ; i++)
			for(int j = 0 ; j < cols; j++){

				// if location is not background
				if(input_.at<int>(i,j) > 0){

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
							if(input_.at<int>(i,j-1) > 0){

								// first check edge
								Vertex v1,v2;
								v1.index_ = input_.at<int>(i,j-1);
								v2.index_ = input_.at<int>(i,j);

								if(!cluster_graph_.findEdge(v1,v2)){

									// compute centroids of edges
									std::pair<double,double> c_1 = findCentroid(input_.at<int>(i,j-1));
									std::pair<double,double> c_2 = findCentroid(input_.at<int>(i,j));
									v1.x_ = c_1.first; v1.y_ = c_1.second;
									v2.x_ = c_2.first; v2.y_ = c_2.second;

									// inserting edge - initializing all edge weights to zero
									cluster_graph_.addEdge(v1,v2,1);
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
							if(input_.at<int>(i-1,j) > 0){

								// first check edge
								Vertex v1,v2;
								v1.index_ = input_.at<int>(i-1,j);
								v2.index_ = input_.at<int>(i,j);

								if(!cluster_graph_.findEdge(v1,v2)){

									// compute centroids of edges
									std::pair<double,double> c_1 = findCentroid(input_.at<int>(i-1,j));
									std::pair<double,double> c_2 = findCentroid(input_.at<int>(i,j));
									v1.x_ = c_1.first; v1.y_ = c_1.second;
									v2.x_ = c_2.first; v2.y_ = c_2.second;

									// inserting edge - initializing all edge weights to zero
									cluster_graph_.addEdge(v1,v2,1);
								}
							}
						}
					}
				}
			}
	}
}

}

int main(int argc, char **argv){


	ros::init(argc, argv, "active_segment");
	ros::NodeHandle nh;

	//	cv::Mat input = cv::imread(argv[1]);
	//	geometry_msgs::Polygon polygon;// = argv[2]; // change this dummy assignment later

	//	active_segmentation::active_segment ss(input, polygon);
	active_segmentation::active_segment ss(nh);
	ss.convertToGraph();

	return 0;

}
