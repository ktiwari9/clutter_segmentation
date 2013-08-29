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
		nh_(nh), nh_priv_("~"),input_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
		processed_cloud_(new pcl::PointCloud<pcl::PointXYZ>){

	nh_priv_.param<std::string>("tabletop_service",tabletop_service_,std::string("/tabletop_segmentation"));
	extract_feature_srv_ = nh_.advertiseService(nh_.resolveName("extract_features_srv"),&extract_features::serviceCallback, this);
}

extract_features::extract_features(std::string filename){

	filename_ = filename;
}

extract_features::~extract_features(){}

std::vector<std::vector<cv::Point> > extract_features::getHoles(cv::Mat input){

	cv::Mat img_gray;
	cv::cvtColor(mask_image_,img_gray,CV_BGR2GRAY);
	std::vector<std::vector<cv::Point> > contours_unordered;
	// closing all contours via dialation
	cv::Mat bw_new;
	// to fill cluster masks
	cv::Mat element = cv::getStructuringElement(2, cv::Size(5,5));

	/// Apply the dilation operation
	cv::dilate(img_gray.clone(), bw_new, element);
	cv::imwrite("/tmp/holesBW.jpg",bw_new);

	cv::Mat new_canny = cv::Mat::zeros(bw_new.size(), CV_8UC1);
	new_canny = ~(bw_new > 0);

	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(new_canny.clone(), contours_unordered, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
	// Now find the holes
	cv::Mat singleLevelHoles = cv::Mat::zeros(new_canny.size(), new_canny.type());
	cv::Mat multipleLevelHoles = cv::Mat::zeros(new_canny.size(), new_canny.type());
	std::cout<<"feature_learning::extract_features: drawing contours in input image"<<std::endl;
	for(std::vector<cv::Vec4i>::size_type i = 0; i < contours_unordered.size();i++)
	{
		if(hierarchy[i][3] != -1)
			cv::drawContours(singleLevelHoles, contours_unordered, i, cv::Scalar::all(255), CV_FILLED, 8, hierarchy);
	}

	cv::bitwise_not(new_canny, new_canny);
	cv::bitwise_and(new_canny, singleLevelHoles, multipleLevelHoles);
	cv::imwrite("/tmp/singleLevelHoles.jpg",singleLevelHoles);
	cv::imwrite("/tmp/multipleLevelHoles.jpg",multipleLevelHoles);

	// Now get the final holes
	std::vector<std::vector<cv::Point> > holes;
	cv::findContours(singleLevelHoles.clone(), holes, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
	ROS_INFO("feature_learning::extract_features: returning contours in input image");
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
//	ROS_INFO("feature_learning::extract_features: Pre-processing cloud");
//	listener_.waitForTransform("/BASE",input_cloud_->header.frame_id,
//				input_cloud_->header.stamp, ros::Duration(5.0));
//	std::cout<<input_cloud_->header.stamp<<std::endl << "Timestamp"<<std::endl;

	// Getting mean of point cloud to estimate the nominal cutting plane
	ROS_INFO("feature_learning::extract_features: Input cloud size %d ",input_cloud_->size());
	pcl::computeMeanAndCovarianceMatrix(*input_cloud_,covariance,cloud_mean);

	// Now compute an xy plane with the mean point and height
	pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
	plane_coefficients->values.resize (4);
	plane_coefficients->values[0] = plane_coefficients->values[1] = 0;
	plane_coefficients->values[2] = cloud_mean(2) ;
	plane_coefficients->values[3] = 0;

	// Getting holes in the image
	ROS_INFO("feature_learning::extract_features: Getting holes in input image");
	std::vector<std::vector<cv::Point> > hole_contours = getHoles(input_segment);
	if(hole_contours.empty())
		return;
	cv::Mat contour_mask = cv::Mat::zeros(input_segment.size(), CV_8UC1);
	cv::drawContours(contour_mask, hole_contours, -1, cv::Scalar::all(255), CV_FILLED,8);
	cv::imwrite("/tmp/countour_image.jpg",contour_mask);
	// Computing means around contour points
	size_t max_size = 0;
	ROS_INFO("Number of contours returned %d",hole_contours.size());
	for(size_t i = 0; i < hole_contours.size(); i++)
	{
		if(hole_contours[i].size() < 100)
			continue;
		cv::Mat sub_contour_mask = cv::Mat::zeros(input_segment.size(), CV_8UC1);
		std::vector<std::vector<cv::Point> > temp_contours;
		temp_contours.push_back(hole_contours[i]);
		ROS_DEBUG("feature_learning::extract_features: Computing the mean of every contour");
		cv::drawContours(sub_contour_mask, temp_contours, -1, cv::Scalar::all(255), CV_FILLED,8);
	    cv::Moments m_c = cv::moments(sub_contour_mask, false);
	    cv::Point2f sub_mask_center(m_c.m10/m_c.m00, m_c.m01/m_c.m00);
		cv::Point2d mean_point(sub_mask_center.x, sub_mask_center.y);
		ROS_DEBUG("feature_learning::extract_features: Projecting pixel to 3D");
		cv::Point3d push_3d = model.projectPixelTo3dRay(mean_point);

		pcl::PointCloud<pcl::PointXYZ> ray;
		ray.push_back(pcl::PointXYZ(0,0,0));
		ray.push_back(pcl::PointXYZ((float)push_3d.x,(float)push_3d.y,(float)push_3d.z));
		ray.header.frame_id =  model.tfFrame();
		ray.header.stamp = ros::Time::now();

		ROS_DEBUG("feature_learning::extract_features: Creating Ray in base frame");
		ROS_VERIFY(listener_.waitForTransform("/BASE",model.tfFrame(),
				ray.header.stamp, ros::Duration(5.0)));

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
		pcl::CropBox<pcl::PointXYZ> cropBox;
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
		ROS_INFO("Filtered cloud size %d",filtered_cloud->size());
		if(max_size < filtered_cloud->size())
		{
			max_size = filtered_cloud->size();
			processed_cloud.swap(*filtered_cloud);
			action_point_.point.x = push_point.x;action_point_.point.y = push_point.y;action_point_.point.z = push_point.z;
			action_point_.header.frame_id = "/BASE";
			ROS_INFO("feature_learning::extract_features: Found a actionable point cloud with size %d",processed_cloud.size());

		}
	}

	ROS_INFO("feature_learning::extract_features: Got an action point");

}

void extract_features::testfeatureClass(cv::Mat image, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		const geometry_msgs::PoseStamped &viewpoint,const geometry_msgs::Pose& surface,
		const geometry_msgs::PoseStamped& gripper_pose, const image_geometry::PinholeCameraModel& model,
		const std::string filename){

	feature_class feature;
	Eigen::MatrixXf final_feature;
	feature.initialized_ = feature.initializeFeatureClass(image,cloud,viewpoint,surface,gripper_pose);

	ROS_INFO("feature_learning::extract_features: Starting feature computation process , Initialized %d",feature.initialized_);
	feature.computeFeature(final_feature);
	// Use bag writer to write before and after bags with topics and name the Eigen matrix as the same thing
	std::stringstream eigen_filename;
	eigen_filename<<filename<<".txt";
	ofstream ofs(eigen_filename.str().c_str(),ios::out | ios::trunc);
	if(ofs)
	{
		// instructions
		ofs << std::endl;
		ofs << final_feature;
		ofs.close();
	}
	else  // sinon
	{
		std::cerr << "Erreur à l'ouverture !" << std::endl;
	}


}

bool extract_features::serviceCallback(ExtractFeatures::Request& request, ExtractFeatures::Response& response){

	ROS_INFO("feature_learning::extract_features: Executing service, initialized %d",initialized_);
	view_point_pose_ = request.view_point;
	gripper_pose_ = request.gripper_pose;
	surface_pose_ = request.surface_pose;
	ROS_INFO("feature_learning::extract_features: Got gripper pose, surface pose and viewpoint");
	if(initialized_){
		ROS_INFO("feature_learning::extract_features: Computing features");
		testfeatureClass(input_image_,processed_cloud_,view_point_pose_,surface_pose_,gripper_pose_,left_cam_,filename_);

		action_point_.header.stamp = ros::Time::now();

		sensor_msgs::PointCloud2Ptr ros_cloud;
		pcl::toROSMsg(*input_cloud_,*ros_cloud);
		ROS_INFO("feature_learning::extract_features: Writing bag");
		try
		{
			bag_.write(topicFeatureInputCloud(), ros::Time::now(), ros_cloud);
			bag_.write(topicFeatureCameraInput(), ros::Time::now(), ros_image_);
			bag_.write(topicFeatureCameraInfo(), ros::Time::now(), cam_info_);
			bag_.write(topicFeatureTable(), ros::Time::now(), surface_pose_);
			bag_.write(topicFeatureGripperPose(), ros::Time::now(), gripper_pose_);
			bag_.write(topicFeatureViewpoint(), ros::Time::now(), view_point_pose_);
		}
		catch (rosbag::BagIOException ex)
		{
			ROS_DEBUG("feature_learning::extract_features: Problem when writing demonstration to file >%s< : %s.",
					bag_.getFileName().c_str(), ex.what());

		}

		ROS_INFO("feature_learning::extract_features: returning success");
		response.action_location = action_point_;
		response.result = ExtractFeatures::Response::SUCCESS;
		return true;
	}
	else{

		ROS_INFO("feature_learning::extract_features: returning failed");
		response.result = ExtractFeatures::Response::FAILURE;
		return false;
	}


}

void extract_features::getMasksFromClusters(const std::vector<sensor_msgs::PointCloud2> &clusters,
                                          const sensor_msgs::CameraInfo &cam_info, std::vector<sensor_msgs::Image> &masks) {

  masks.resize(clusters.size());

  Eigen::Matrix4f P;
  int rows = 3, cols = 4;
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
      P(r, c) = cam_info.P[r * cols + c];

  P(3, 0) = 0;
  P(3, 1) = 0;
  P(3, 2) = 0;
  P(3, 3) = 1;

  //std::cout << "Transformation Matrix " << std::endl << P << std::endl;

  for (size_t i = 0; i < clusters.size(); ++i) {

    sensor_msgs::PointCloud2 cloud_proj;

    sensor_msgs::Image mask;
    mask.height = cam_info.height;
    mask.width = cam_info.width;
    //mask.encoding = enc::TYPE_32FC1;
    mask.encoding = sensor_msgs::image_encodings::MONO8;
    mask.is_bigendian = false;
    mask.step = mask.width;
    size_t size = mask.step * mask.height;
    mask.data.resize(size);

    pcl_ros::transformPointCloud(P, clusters[i], cloud_proj);

    for (unsigned int j = 0; j < cloud_proj.width; j++) {

      float x, y, z;

      memcpy(&x,
             &cloud_proj.data[j * cloud_proj.point_step
                              + cloud_proj.fields[0].offset], sizeof(float));
      memcpy(&y,
             &cloud_proj.data[j * cloud_proj.point_step
                              + cloud_proj.fields[1].offset], sizeof(float));
      memcpy(&z,
             &cloud_proj.data[j * cloud_proj.point_step
                              + cloud_proj.fields[2].offset], sizeof(float));

      if (round(y / z) >= 0 && round(y / z) < mask.height
          && round(x / z) >= 0 && round(x / z) < mask.width) {
        int i = round(y / z) * mask.step + round(x / z);
        mask.data[i] = 255;
      }
    }

    masks[i] = mask;
  }
}


bool extract_features::initialized(std::string filename){

	ROS_INFO("feature_learning::extract_features: Initializing extract features");
	sensor_msgs::Image::ConstPtr input_image = ros::topic::waitForMessage<sensor_msgs::Image>("/Honeybee/left/image_rect_color", nh_, ros::Duration(5.0));
	sensor_msgs::PointCloud2ConstPtr ros_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/XTION/rgb/points",nh_, ros::Duration(5.0));
	pcl::fromROSMsg (*ros_cloud, *input_cloud_);
	ROS_INFO("feature_learning::extract_features: Got input image and pointcloud");

	ROS_VERIFY(listener_.waitForTransform("/BASE",input_cloud_->header.frame_id,
			input_cloud_->header.stamp, ros::Duration(5.0)));

	ROS_VERIFY(pcl_ros::transformPointCloud("/BASE",*input_cloud_,*input_cloud_,listener_));

	graph_segment convertor;
	input_image_ = convertor.returnCVImage(*input_image);
	convertor.setCVImage(input_image_);
	ros_image_ = convertor.returnRosImage(*input_image);
	ROS_INFO("feature_learning::extract_features: Converted image into ros image");


	// getting camera info
	sensor_msgs::CameraInfo::ConstPtr cam_info =
			ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/Honeybee/left/camera_info", nh_, ros::Duration(10.0));
	left_cam_.fromCameraInfo(cam_info);
	cam_info_ = *cam_info;
	ROS_INFO("feature_learning::extract_features: got camera info");

	// Now getting the table top info

	// Calling tabletop segmentation service
	if (!ros::service::call(tabletop_service_, tabletop_srv_)) {
		ROS_ERROR("Call to tabletop segmentation service failed");
		return false;
	}
	if (tabletop_srv_.response.result != tabletop_srv_.response.SUCCESS
			&& tabletop_srv_.response.result != tabletop_srv_.response.SUCCESS_NO_RGB) {

		ROS_ERROR("Segmentation service returned error %d", tabletop_srv_.response.result);
		return false;
	}

	//convert clusters to honeybee frame
	ROS_INFO("Transforming clusters to bumblebee frame");

	std::vector<sensor_msgs::PointCloud2> clusters;
	for (int i = 0; i < (int) tabletop_srv_.response.clusters.size(); i++) {

		// transforming every cluster in the service
		sensor_msgs::PointCloud2 transform_cloud;

		try {
			tabletop_srv_.response.clusters[i].header.stamp = ros::Time(0);// TODO: <---- Change this later
			pcl_ros::transformPointCloud(input_image->header.frame_id,
					tabletop_srv_.response.clusters[i], transform_cloud,
					listener_);
		} catch (tf::TransformException& ex) {
			ROS_ERROR("Failed to transform cloud from frame %s into frame %s", tabletop_srv_.response.clusters[0].header.frame_id.c_str(),
					input_image->header.frame_id.c_str());
		}

		clusters.push_back(transform_cloud);
	}

	// Now getting transformed masks from transformed clusters
	std::vector<sensor_msgs::Image> bbl_masks;
	getMasksFromClusters(clusters, cam_info_, bbl_masks);

	// Now combine masks and add to static segmenter
	cv::Mat init = convertor.returnCVImage(bbl_masks[0]);
	mask_image_ = cv::Mat::zeros(init.size(), CV_8UC1);

	for (int i = 0; i < (int) bbl_masks.size(); i++) {

		cv::Mat mask = convertor.returnCVImage(bbl_masks[i]);
		cv::add(mask, mask_image_, mask_image_);
	}

	preProcessCloud(input_image_,left_cam_,*processed_cloud_);

	if(processed_cloud->size() < 10)
	{
		ROS_ERROR("feature_learning::extract_features: FEATURE COMPUTATION FAILED");
		return;
	}



	try
	{
		bag_.open(filename, rosbag::bagmode::Write);

	}
	catch (rosbag::BagIOException ex)
	{
		ROS_DEBUG("feature_learning::extract_features: Problem when opening demo file >%s< : %s.",
				filename.c_str(), ex.what());
	}

	if(left_cam_.distortionCoeffs().empty() || input_image_.empty() || input_cloud_->empty())
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
	feature.setInitialized(feature.initialized(filename));

	ros::spin();
	return 0;
}


