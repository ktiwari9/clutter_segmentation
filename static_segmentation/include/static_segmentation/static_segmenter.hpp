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
#ifndef STATIC_SEGMENTATION_H
#define STATIC_SEGMENTATION_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tabletop_segmenter/TabletopSegmentation.h>
#include <graph_based_segmentation/GraphSegment.h>
#include "static_segmentation/StaticSegment.h"
#include <geometry_msgs/Polygon.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "pcl_ros/transforms.h"

using namespace std;

typedef struct {
    int x;
    int y;
}Pixel;

namespace static_segmentation{

class static_segment{

protected:

	ros::NodeHandle nh_;

	cv::Mat input_;

	std::string tabletop_service_,graph_service_,rgb_topic_,camera_topic_;

	tabletop_segmenter::TabletopSegmentation tabletop_srv_;

	graph_based_segmentation::GraphSegment graphsegment_srv_;

	tf::TransformListener listener_;

	sensor_msgs::CameraInfo cam_info_;

public:

	static_segment(ros::NodeHandle &nh);

	~static_segment();

	bool serviceCallback(StaticSegment::Request &request, StaticSegment::Response &response);

	geometry_msgs::Polygon computeCGraph();

	void getMasksFromClusters(const std::vector<sensor_msgs::PointCloud2> &clusters,
			const sensor_msgs::CameraInfo &cam_info,
			std::vector<sensor_msgs::Image> &masks);

	cv::Mat returnCVImage(const sensor_msgs::Image & img);

private:

	ros::NodeHandle nh_priv_;

	ros::ServiceServer static_segment_srv_;

};

}

#endif

