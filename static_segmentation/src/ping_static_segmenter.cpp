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
 * @b Service test node for graph based segmentation
 */
// Author(s): Matei Ciocarlie

#include <ros/ros.h>

#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include<static_segmentation/StaticSegment.h>
#include <geometry_msgs/Polygon.h>

#include<graph_module/EGraph.h>
#include<graph_module/graph_module.hpp>

bool dumpGraphImage(const sensor_msgs::Image & img,
		 const char *name, graph_module::EGraph& input_graph)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(img);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}


	cv::Mat input = cv_ptr->image;

	graph::ros_graph new_graph;

	bool result = new_graph.buildGraph(input_graph);

	if(result){

		for(new_graph.iter_=new_graph.graph_.begin();new_graph.iter_!=new_graph.graph_.end(); new_graph.iter_++){

			// First point
			graph::Edge_ros edge = *new_graph.iter_;
			cv::Point center_1(edge.edge_.first.x_,edge.edge_.first.y_);
			cv::circle(input,center_1, 5, cv::Scalar(128,0,0), -1);
			// Second point
			cv::Point center_2(edge.edge_.second.x_,edge.edge_.second.y_);
			cv::circle(input,center_2, 5, cv::Scalar(128,0,0), -1);

		}

		ROS_INFO("Saving Image");

		cv::imwrite(name, input);


	}

	return true;
}



/*! Simply pings the graph_based_segmentation segmentation and recognition services and prints out the result.*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_static_segment_node");
  ros::NodeHandle nh;

  std::string service_name("/static_segment_srv");
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);


  static_segmentation::StaticSegment segmentation_srv;

  segmentation_srv.request.call = segmentation_srv.request.EMPTY;

  if (!ros::service::call(service_name, segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    exit(0);
  }
  if (segmentation_srv.response.result == segmentation_srv.response.FAILURE)
  {
    ROS_ERROR("Segmentation service returned error");
    exit(0);
  }

  ROS_INFO("Segmentation service succeeded. Returned Segmented Graph %d",segmentation_srv.response.result);

  // DEBUG of projection
  if(segmentation_srv.response.result == segmentation_srv.response.SUCCESS)
	 ROS_INFO("Response Success");
	   dumpGraphImage(segmentation_srv.response.graph_image,"/tmp/segmented_image.png",segmentation_srv.response.out_graph);

  return true;
}

