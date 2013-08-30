/*********************************************************************
*
*  Copyright (c) 2013, Computational Learning and Motor Control Laboratory
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
 * @b header file for the extract feature class for feature extraction from the feature_base_class
 */

#ifndef EXTRACT_FEATURES_HPP
#define EXTRACT_FEATURES_HPP

#include <string>
#include <vector>
#include <opencv2/highgui/highgui.hpp>

#include "feature_learning/feature_base_class.hpp"
#include "feature_learning/ExtractFeatures.h"
#include <learn_appearance/texton_hist.h>
#include <pcl/surface/convex_hull.h>
#include <sensor_msgs/CameraInfo.h>
#include "pcl_ros/transforms.h"
//Grasp Template Includes
#include <grasp_template/heightmap_sampling.h>
#include <pcl/features/usc.h> // Unique shape context feature
#include <pcl/features/3dsc.h> // Unique shape context feature
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
//Tabletopsegmenter includes
#include "tabletop_segmenter/TabletopSegmentation.h"

// Other includes
#include <graph_based_segmentation/segment.h>
#include <rosbag/bag.h>
#include <visualization_msgs/Marker.h>

namespace feature_learning {

class extract_features {

public:

	std::string filename_;

protected:

	ros::NodeHandle nh_;
	tf::TransformListener listener_;

	ros::Publisher vis_pub_;
	// Visualization Markers
	visualization_msgs::Marker marker_;

	bool initialized_;

	std::string tabletop_service_;
	tabletop_segmenter::TabletopSegmentation tabletop_srv_;

	static const double BOX_WIDTH_X = 0.50; // in m
	static const double BOX_LENGTH_Y = 0.50; // in m
	static const double BOX_HEIGHT_Z = 0.35; // in m

	std::string topicFeatureInputCloud() const {return "/XTION/rgb/points";};
	std::string topicFeatureCameraInfo() const {return "/Honeybee/left/camera_info'";};
	std::string topicFeatureCameraInput() const {return "/Honeybee/left/image_rect_color";};
	std::string topicFeatureTable() const {return "/grasp_demo_table";};
	std::string topicFeatureGripperPose() const {return "/grasp_demo_gripper_pose";};
	std::string topicFeatureViewpoint() const {return "/grasp_demo_viewpoint_transform";};

public:

	extract_features(std::string filename);

	extract_features(ros::NodeHandle& nh);

	~extract_features();

	void setInitialized(bool initialized){initialized_ = initialized;}

	bool initialized(std::string filename);

	std::vector<std::vector<cv::Point> > getHoles(cv::Mat input);

	void testfeatureClass(cv::Mat image, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
			const geometry_msgs::PoseStamped &viewpoint,const geometry_msgs::Pose& surface,
			const geometry_msgs::PoseStamped& gripper_pose, const image_geometry::PinholeCameraModel& model,
			const std::string filename);

	void preProcessCloud(cv::Mat input_segment,const image_geometry::PinholeCameraModel& model,
			pcl::PointCloud<pcl::PointXYZ> &processed_cloud);

	bool serviceCallback(ExtractFeatures::Request& request, ExtractFeatures::Response& response);

	void getMasksFromClusters(const std::vector<sensor_msgs::PointCloud2> &clusters,
            const sensor_msgs::CameraInfo &cam_info, std::vector<sensor_msgs::Image> &masks);

private:

	rosbag::Bag bag_;

	ros::NodeHandle nh_priv_;

	ros::ServiceServer extract_feature_srv_;

	// Data required for feature extraction
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_,processed_cloud_;
	cv::Mat input_image_, mask_image_;
	geometry_msgs::PoseStamped view_point_pose_, gripper_pose_;
	geometry_msgs::Pose surface_pose_;
	image_geometry::PinholeCameraModel left_cam_;
	sensor_msgs::CameraInfo cam_info_;
	sensor_msgs::ImagePtr ros_image_;
	pcl::ModelCoefficients::Ptr table_coefficients_;

	geometry_msgs::PointStamped action_point_;


};

}

#endif
