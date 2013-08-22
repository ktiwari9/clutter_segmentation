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
 * @b source file for the feature_class definition for feature learning via MaxEnt
 */



#include "feature_learning/extract_features.hpp"
#include <usc_utilities/assert.h>


using namespace grasp_template;
using namespace graph_based_segmentation;

namespace feature_learning{

extract_features::extract_features(ros::NodeHandle& nh):
		nh_(nh), nh_priv_("~"),input_cloud_(new pcl::PointCloud<pcl::PointXYZ>){

	extract_feature_srv_ = nh_.advertiseService(nh_.resolveName("extract_features_srv"),&extract_features::serviceCallback, this);
}

extract_features::extract_features(std::string filename){

	filename_ = filename;
}

extract_features::~extract_features(){}

std::vector<std::vector<cv::Point> > extract_features::getHoles(cv::Mat input){

	// first find contours aka holes
	std::vector<std::vector<cv::Point> > contours_unordered;
	cv::Mat image_gray;
	cv::cvtColor( input, image_gray, CV_BGR2GRAY );

	// convert to BW
	cv::Mat image_bw = image_gray > 128;
	int thresh = 100;
	cv::Mat canny_output;
	cv::Canny(image_bw, canny_output, thresh, thresh*2, 3 );
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(canny_output.clone(), contours_unordered, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
	// Now find the holes

	cv::Mat singleLevelHoles = cv::Mat::zeros(image_bw.size(), image_bw.type());
	 cv::Mat multipleLevelHoles = cv::Mat::zeros(image_bw.size(), image_bw.type());
	for(std::vector<cv::Vec4i>::size_type i = 0; i < contours_unordered.size();i++)
	{
		if(hierarchy[i][3] != -1)
			cv::drawContours(singleLevelHoles, contours_unordered, i, cv::Scalar::all(255), CV_FILLED, 8, hierarchy);
	}

	cv::bitwise_not(image_bw, image_bw);
	cv::bitwise_and(image_bw, singleLevelHoles, multipleLevelHoles);

	// Now get the final holes
	std::vector<std::vector<cv::Point> > holes;
	cv::findContours(multipleLevelHoles.clone(), holes, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
	return holes;
}

void extract_features::preProcessCloud(cv::Mat input_segment,const image_geometry::PinholeCameraModel& model,
		pcl::PointCloud<pcl::PointXYZ> &processed_cloud){

	// Local Declarations
	processed_cloud.clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Mean and covariance declarations
	Eigen::Matrix3f covariance;
	Eigen::Vector4f cloud_mean;

	// Now transform point cloud base frame
	ROS_VERIFY(listener_.waitForTransform("/BASE",input_cloud_->header.frame_id,
					ros::Time::now(), ros::Duration(5.0)));

	ROS_VERIFY(pcl_ros::transformPointCloud("/BASE",*input_cloud_,*input_cloud_,listener_));

	// Getting mean of point cloud to estimate the nominal cutting plane
	pcl::computeMeanAndCovarianceMatrix(*input_cloud_,covariance,cloud_mean);

	// Now compute an xy plane with the mean point and height
	pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
	plane_coefficients->values.resize (4);
	plane_coefficients->values[0] = plane_coefficients->values[1] = 0;
	plane_coefficients->values[2] = cloud_mean(2) ;
	plane_coefficients->values[3] = 0;

	// Getting holes in the image
	std::vector<std::vector<cv::Point> > hole_contours = getHoles(input_segment);

	// Computing means around contour points
	size_t max_size = 0;
	for(size_t i = 0; i < hole_contours.size(); i++)
	{
		cv::Mat mean_value;
		cv::reduce(hole_contours[i], mean_value, CV_REDUCE_AVG, 1);
		cv::Point2d mean_point(mean_value.at<float>(0,0), mean_value.at<float>(0,1));
		cv::Point3d push_3d = model.projectPixelTo3dRay(mean_point);

		pcl::PointCloud<pcl::PointXYZ> ray;
		ray.push_back(pcl::PointXYZ(0,0,0));
		ray.push_back(pcl::PointXYZ((float)push_3d.x,(float)push_3d.y,(float)push_3d.z));

		ROS_DEBUG("Creating Ray in base frame");
		ROS_VERIFY(listener_.waitForTransform("/BASE",model.cam_info_.header.frame_id,
				ros::Time::now(), ros::Duration(5.0)));

		ROS_VERIFY(pcl_ros::transformPointCloud("/BASE", ray,
					ray, listener_));

		// Now get intersection of Ray and cloud XY plane
		/*
		 * TODO: This may only work if plane calculation is correct, so verify this
		 */

		float t;
		t = (plane_coefficients->values[3] + plane_coefficients->values[0]*ray.points[0].x +
				plane_coefficients->values[1]*ray.points[0].y+ plane_coefficients->values[2]*ray.points[0].z);
		t /= (plane_coefficients->values[0]*ray.points[1].x +
				plane_coefficients->values[1]*ray.points[1].y+ plane_coefficients->values[2]*ray.points[1].z);

		pcl::PointXYZ push_point;
		push_point.x = t*ray.points[1].x;push_point.y = t*ray.points[1].y; push_point.z = t*ray.points[1].z;

		/*-----------------------------------------------------------------------------------------------*/
		// Now compute the box filter around this centroid and count the number of points.
		// TODO: This is a hack but how do I justify it?? Why the most number of points and not the least
		// number of points.
		/*-----------------------------------------------------------------------------------------------*/

		// Getting the transform offsets
		tf::Transform template_center_transform;
		template_center_transform.setIdentity();
		//template_center_transform.getOrigin().setY() //--> I don't think I need this

		// Creating Box filter
		pcl::CropBox cropBox;
		cropBox.setInputCloud(input_cloud_);
		// TODO: Do I use the centroid as reference or should I use the table height like Peter does??
		Eigen::Vector4f bag_min(-(BOX_WIDTH_X/(double)2.0),
				template_center_transform.getOrigin().getY() -(BOX_LENGTH_Y/(double)2.0),
				0.0,
				1.0);
		Eigen::Vector4f bag_max((BOX_WIDTH_X/(double)2.0),
				template_center_transform.getOrigin().getY() +(BOX_LENGTH_Y/(double)2.0),
				0.0 + BOX_HEIGHT_Z,
				1.0);
		Eigen::Vector3f eigen_rotation(0.0, 0.0, 0.0);
		Eigen::Vector3f eigen_translation(push_point.x,push_point.y,0.0);
		cropBox.setRotation(eigen_rotation);
		cropBox.setTranslation(eigen_translation);
		cropBox.setInputCloud(input_cloud_);
		filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
		cropBox.filter(*filtered_cloud);

		if(max_size < filtered_cloud->size())
		{
			max_size = filtered_cloud->size();
			processed_cloud.swap(*filtered_cloud);
			action_point_.point.x = push_point.x;action_point_.point.y = push_point.y;action_point_.point.z = push_point.z;
			action_point_.header = "/BASE";
		}
	}

}

void extract_features::testfeatureClass(cv::Mat image, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		const geometry_msgs::PoseStamped &viewpoint,const geometry_msgs::Pose& surface,
		const geometry_msgs::PoseStamped& gripper_pose, const image_geometry::PinholeCameraModel& model,
		const std::string filename){

	pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud;
	preProcessCloud(image,model,*processed_cloud);

	feature_class<pcl::PointXYZ> feature;
	Eigen::MatrixXf final_feature;
	feature.initialized_ = feature.initializeFeatureClass(image,processed_cloud,viewpoint,surface,gripper_pose);

	feature.computeFeature(final_feature); // TODO: Write this feature to disk?? How??
	// Use bag writer to write before and after bags with topics and name the Eigen matrix as the same thing
	std::stringstream eigen_filename;
	eigen_filename<<filename<<".txt";
	ofstream ofs(eigen_filename.str().c_str(),ios::out | ios::trunc);
	if(ofs)
	{
		// instructions
		ofs << final_feature;
		ofs.close();
	}
	else  // sinon
	{
		std::cerr << "Erreur à l'ouverture !" << std::endl;
	}


}

bool extract_features::serviceCallback(ExtractFeatures::Request& request, ExtractFeatures::Response& response){

	view_point_pose_ = request.view_point;
	gripper_pose_ = request.gripper_pose;
	surface_pose_ = request.surface_pose;

	if(initialized_){

		testfeatureClass(input_image_,input_cloud_,view_point_pose_,surface_pose_,gripper_pose_,left_cam_,filename_);

		action_point_.header.stamp = ros::Time::now();

		sensor_msgs::PointCloud2Ptr ros_cloud;
		pcl::toROSMsg(*input_cloud_,*ros_cloud);
		try
		{
			bag_.write(topicFeatureInputCloud(), ros::Time::now(), ros_cloud);
			bag_.write(topicFeatureCameraInput(), ros::Time::now(), ros_image_);
			bag_.write(topicFeatureCameraInfo(), ros::Time::now(), left_cam_);
			bag_.write(topicFeatureTable(), ros::Time::now(), surface_pose_);
			bag_.write(topicFeatureGripperPose(), ros::Time::now(), gripper_pose_);
			bag_.write(topicFeatureViewpoint(), ros::Time::now(), view_point_pose_);
		}
		catch (rosbag::BagIOException ex)
		{
			ROS_DEBUG("grasp_template_planning::DemoWriter: Problem when writing demonstration to file >%s< : %s.",
					bag_.getFileName().c_str(), ex.what());

		}

		response.action_location = action_point_;
		response.result = ExtractFeatures::Response::SUCCESS;
		return true;
	}
	else{

		response.result = ExtractFeatures::Response::FAILURE;
		return false;
	}


}

bool extract_features::initialized(std::string filename){

	sensor_msgs::Image::ConstPtr input_image = ros::topic::waitForMessage<sensor_msgs::Image>("/Honeybee/left/image_rect_color", nh_, ros::Duration(5.0));
	sensor_msgs::PointCloud2ConstPtr ros_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/Honeybee/left/image_rect_color",nh_, ros::Duration(5.0));
	pcl::fromROSMsg (*ros_cloud, *input_cloud_);

	graph_segment convertor;
	input_image_ = convertor.returnCVImage(*input_image);
	convertor.input_ = input_image_;
	ros_image_ = convertor.returnRosImage(*input_image);

	// getting camera info
	sensor_msgs::CameraInfo::ConstPtr cam_info =
			ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/Honeybee/left/camera_info", nh_, ros::Duration(10.0));
	left_cam_.fromCameraInfo(cam_info);

	try
	{
		bag_.open(filename, rosbag::bagmode::Write);

	}
	catch (rosbag::BagIOException ex)
	{
		ROS_DEBUG("grasp_template_planning::DemoWriter: Problem when opening demo file >%s< : %s.",
				filename.c_str(), ex.what());
	}

	if(left_cam_.D_.empty() || input_image_.empty() || input_cloud_->empty())
		return false;
	else
		return true;
}

} //namespace

int main(int argc, char **argv) {
	ros::init(argc, argv, "extract_feature_srv");
	ros::NodeHandle nh;
	if(argc < 2)
	{
		std::cout<<" Please provide a filename sequence to save to"<<std::endl;
		return -1;
	}
	std::string filename(argv[1]);

	feature_learning::extract_features feature(nh);
	feature.initialized_ = feature.initialized(filename);

	ros::spin();
	return 0;
}


