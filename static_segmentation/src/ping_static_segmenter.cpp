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

bool dumpGraphImage(const sensor_msgs::Image & img,
		 const char *name, geometry_msgs::Polygon& input_graph)
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

  for(int i=0;i<input_graph.points.size();i++){

	  cv::Point center(input_graph.points[i].x,input_graph.points[i].y);
	  cv::circle(input,center, 5, cv::Scalar(128,0,0), -1);

  }

  cv::imwrite(name, input);


  //test what's in the image
  //std::cout<<cv_ptr->image<<std::endl;

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

  ROS_INFO("Segmentation service succeeded. Returned Segmented Graph");

  // DEBUG of projection
  if(segmentation_srv.response.result == segmentation_srv.response.SUCCESS)
	  dumpGraphImage(segmentation_srv.response.graph_image,"segmented_image.png",
			  segmentation_srv.response.c_graph.polygon);

  return true;
}

