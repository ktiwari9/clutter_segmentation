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


//pcl publisher
#include <pcl17_ros/point_cloud.h>

#ifdef _OPENMP
#include <omp.h>
#endif


using namespace graph_based_segmentation;

int writer_counter = 1;

namespace feature_learning{

extract_features::extract_features(ros::NodeHandle& nh):
										nh_(nh), nh_priv_("~"),input_cloud_(new pcl17::PointCloud<pcl17::PointXYZ>), input_rgb_cloud_(new pcl17::PointCloud<PoinRGBType>),holes_(false),
										processed_cloud_(new pcl17::PointCloud<pcl17::PointXYZ>),table_coefficients_(new pcl17::ModelCoefficients ()){

	nh_priv_.param<std::string>("tabletop_service",tabletop_service_,std::string("/tabletop_segmentation"));
	nh_priv_.param<std::string>("input_cloud_topic",input_cloud_topic_,std::string("/tabletop_segmentation"));
	nh_priv_.param<std::string>("input_camera_info",input_camera_info_,std::string("/tabletop_segmentation"));
	nh_priv_.param<std::string>("input_image_topic",input_image_topic_,std::string("/tabletop_segmentation"));
	nh_priv_.param<std::string>("base_frame",base_frame_,std::string("/tabletop_segmentation"));
	nh_priv_.param<std::string>("svm_load",svm_filename_,std::string("saved_svm_function.dat"));

	extract_feature_srv_ = nh_.advertiseService(nh_.resolveName("extract_features_srv"),&extract_features::serviceCallback, this);
	vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/intersection_marker", 1);
	m_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/template_markers", 1);
	pcd_pub_ = nh_.advertise<pcl17::PointCloud<PointType> >("/template_patches", 1);
	edge_cloud_ = nh_.advertise<pcl17::PointCloud<PoinRGBType> >("/template_basis", 1);

	marker_.header.stamp = ros::Time();
	marker_.ns = "extract_features";
	marker_.id = 0;
	marker_.type = visualization_msgs::Marker::SPHERE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.scale.x = 0.05;
	marker_.scale.y = 0.05;
	marker_.scale.z = 0.05;
	marker_.color.a = 1.0;
	marker_.color.r = 0.0;
	marker_.color.g = 1.0;
	marker_.color.b = 0.0;
	marker_.pose.orientation.x = 0.0;
	marker_.pose.orientation.y = 0.0;
	marker_.pose.orientation.z = 0.0;
	marker_.pose.orientation.w = 1.0;

	ROS_INFO("feature_learning::extract_features: Loading SVM files");

	//	ifstream fin(svm_filename_.c_str(),ios::binary);
	//    deserialize(learned_pfunct_, fin);

	ROS_INFO("feature_learning::extract_features: Loaded SVM files");

}

extract_features::extract_features(){}

extract_features::~extract_features(){}

visualization_msgs::Marker extract_features::getMarker(int i){

	visualization_msgs::Marker local_marker;

	local_marker.header.stamp = ros::Time();
	local_marker.ns = "extract_features";
	local_marker.id = i;
	local_marker.type = visualization_msgs::Marker::SPHERE;
	local_marker.action = visualization_msgs::Marker::ADD;
	local_marker.scale.x = 0.05;
	local_marker.scale.y = 0.05;
	local_marker.scale.z = 0.05;
	local_marker.color.a = 1.0;
	local_marker.color.r = 0.0;
	local_marker.color.g = 1.0;
	local_marker.color.b = 0.0;
	local_marker.pose.orientation.x = 0.0;
	local_marker.pose.orientation.y = 0.0;
	local_marker.pose.orientation.z = 0.0;
	local_marker.pose.orientation.w = 1.0;

	return local_marker;

}

std::vector<std::vector<cv::Point> > extract_features::getHoles(cv::Mat input){

	cv::Mat img_gray = mask_image_;
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
	ROS_INFO("feature_learning::extract_features: drawing contours in input image");
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

pcl17::PointCloud<pcl17::PointXYZ> extract_features::preProcessCloud_edges(cv::Mat input_segment,const image_geometry::PinholeCameraModel& model,
		pcl17::PointCloud<pcl17::PointXYZ> &processed_cloud){

	holes_ = false;
	ROS_INFO("feature_learning::extract_features: Initializing edge computation features");
	// Local Declarations
	processed_cloud.clear();
	pcl17::PointCloud<PointNT>::Ptr cloud_normals (new pcl17::PointCloud<PointNT>);
	pcl17::search::KdTree<PoinRGBType>::Ptr tree (new pcl17::search::KdTree<PoinRGBType>);
	ROS_INFO("feature_learning::extract_features: Starting Normal Estimation %d",input_cloud_->points.size());

	pcl17::NormalEstimationOMP<PoinRGBType, PointNT> ne;
	//pcl17::NormalEstimation<PointType, PointNT> ne;
	ne.setInputCloud (input_rgb_cloud_);
	ne.setNumberOfThreads(6);


	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	ne.setSearchMethod (tree);

	// Output datasets

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);
	ROS_INFO("feature_learning::extract_features: Computing Normals");

	// Compute the features
	ne.compute (*cloud_normals);


	ROS_INFO("feature_learning::extract_features: Initializing organized edge detection");

	pcl17::OrganizedEdgeFromNormals<PoinRGBType,PointNT, pcl17::Label> oed;
	oed.setInputCloud (input_rgb_cloud_);
	oed.setInputNormals (cloud_normals);
	oed.setDepthDisconThreshold (0.02); // 2cm
	oed.setMaxSearchNeighbors (50);
	pcl17::PointCloud<pcl17::Label> labels;
	std::vector<pcl17::PointIndices> label_indices;
	ROS_INFO("feature_learning::extract_features: Computing organized edges");
	oed.compute (labels, label_indices);
	ROS_INFO("feature_learning::extract_features: Computed labels size:%d", labels.points.size());

	// Now cluster the edges and return them to the user
	pcl17::PointCloud<PointType> edge_list;
	pcl17::PointCloud<PoinRGBType> edges;

	int counter = 0;
	ROS_INFO("feature_learning::extract_features: Clustering edges, Number of Edges found: %d",label_indices.size());
	marker_array_.markers.clear();

#pragma omp parallel for
	for(size_t i = 0; i < label_indices.size() ; i++)
	{
		pcl17::PointCloud<PoinRGBType>::Ptr edge_points (new pcl17::PointCloud<PoinRGBType>);
	        pcl17::search::KdTree<PoinRGBType>::Ptr local_tree (new pcl17::search::KdTree<PoinRGBType>);
		pcl17::copyPointCloud (*input_rgb_cloud_, label_indices[i].indices, *edge_points);
		ROS_INFO("feature_learning::extract_features: Clustering edges of Type %d, with %d points",i,label_indices[i].indices.size());
		ROS_INFO("feature_learning::extract_features: Corresponding cloud size %d",edge_points->points.size());
		std::vector<pcl17::PointIndices> cluster_indices;
		pcl17::EuclideanClusterExtraction<PoinRGBType> ec;
		ec.setClusterTolerance (0.01); // 2cm
		ec.setMinClusterSize (50);
		ec.setMaxClusterSize (1500);
		ec.setSearchMethod (local_tree);
		ec.setInputCloud (edge_points);
		ec.extract (cluster_indices);
		ROS_INFO("feature_learning::extract_features: % d clusters found for edges of type %d",cluster_indices.size(),i);

		for (std::vector<pcl17::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			pcl17::PointCloud<PoinRGBType>::Ptr cloud_cluster (new pcl17::PointCloud<PoinRGBType>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				cloud_cluster->points.push_back (edge_points->points[*pit]); //*

			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;

			Eigen::Vector4f centroid;
#pragma omp critical
			edges += *cloud_cluster;
			pcl17::compute3DCentroid(*cloud_cluster,centroid);
			PointType center_point;
			center_point.x = centroid[0];center_point.y = centroid[1];center_point.z = centroid[2];
#pragma omp critical
			edge_list.push_back(center_point);

			visualization_msgs::Marker location_marker = getMarker(counter);
			counter++;

			location_marker.header = input_cloud_->header;
			location_marker.header.stamp = ros::Time();
			location_marker.pose.position.x = center_point.x; location_marker.pose.position.y = center_point.y; location_marker.pose.position.z = center_point.z;
#pragma omp critical
			marker_array_.markers.push_back(location_marker);
		}
	}
	ROS_INFO("feature_learning::extract_features: Publishing edges and markers");

	edges.header = input_rgb_cloud_->header;
	edges.header.stamp = ros::Time();
	edge_cloud_.publish(edges.makeShared());
	m_array_pub_.publish(marker_array_);
	return edge_list;
}

pcl17::PointCloud<pcl17::PointXYZ> extract_features::preProcessCloud_holes(cv::Mat input_segment,const image_geometry::PinholeCameraModel& model,
		pcl17::PointCloud<pcl17::PointXYZ> &processed_cloud){

	// Local Declarations
	holes_ = true;
	pcl17::PointCloud<PointType> edge_list;

	processed_cloud.clear();
	pcl17::PointCloud<pcl17::PointXYZ>::Ptr filtered_cloud (new pcl17::PointCloud<pcl17::PointXYZ>);
	pcl17::PassThrough<pcl17::PointXYZ> pass;

	// Mean and covariance declarations
	Eigen::Matrix3f covariance;
	Eigen::Vector4f cloud_mean;

	// Getting mean of point cloud to estimate the nominal cutting plane
	ROS_INFO("feature_learning::extract_features: Input cloud size %d ",input_cloud_->size());
	pcl17::computeMeanAndCovarianceMatrix(*input_cloud_,covariance,cloud_mean);

	// Getting holes in the image
	ROS_INFO("feature_learning::extract_features: Getting holes in input image");
	std::vector<std::vector<cv::Point> > hole_contours = getHoles(input_segment);
	if(hole_contours.empty())
		return edge_list;
	cv::Mat contour_mask = cv::Mat::zeros(input_segment.size(), CV_8UC1);
	cv::drawContours(contour_mask, hole_contours, -1, cv::Scalar::all(255), CV_FILLED,8);
	cv::imwrite("/tmp/countour_image.jpg",contour_mask);
	// Computing means around contour points
	size_t max_size = 0;
	ROS_INFO("feature_learning::extract_features: Number of contours returned %d",hole_contours.size());

	PointType push_point;

	marker_array_.markers.clear();
	int counter = 0;
#pragma omp parallel for
	for(size_t i = 0; i < hole_contours.size(); i++)
	{
		if(hole_contours[i].size() < 50)
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
		center_points_.push_back(mean_point);
		cv::Point3d push_3d = model.projectPixelTo3dRay(mean_point);

		pcl17::PointCloud<pcl17::PointXYZ> ray;
		ray.push_back(pcl17::PointXYZ(0,0,0));
		ray.push_back(pcl17::PointXYZ((float)push_3d.x,(float)push_3d.y,(float)push_3d.z));
		ray.header.frame_id =  model.tfFrame();
		ray.header.stamp = ros::Time::now();

		ROS_DEBUG("feature_learning::extract_features: Creating Ray in base frame");
		ROS_INFO_STREAM("Model frame "<<model.tfFrame());

		try {
			ROS_VERIFY(listener_.waitForTransform(base_frame_,model.tfFrame(),ray.header.stamp, ros::Duration(10.0)));
			ROS_VERIFY(pcl17_ros::transformPointCloud(base_frame_, ray,ray, listener_));
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}

		push_point.x = ray.points[1].x; push_point.y = ray.points[1].y; push_point.z = ray.points[1].z;
		if(push_point.z < table_height_)
			push_point.z = table_height_;

		visualization_msgs::Marker location_marker = getMarker(counter);
		counter++;

		ROS_INFO("feature_learning::extract_features: Marker Frame: %s",input_cloud_->header.frame_id.c_str());
		location_marker.header = input_cloud_->header;
		location_marker.header.stamp = ros::Time();
		location_marker.pose.position.x = push_point.x; location_marker.pose.position.y = push_point.y; location_marker.pose.position.z = push_point.z;

#pragma omp critical
		{
		marker_array_.markers.push_back(location_marker);
		edge_list.push_back(push_point);
		}
	}

	ROS_INFO("feature_learning::extract_features: Publishing Markers");
	m_array_pub_.publish(marker_array_);
	return edge_list;

}

extract_features::FeatureVector extract_features::convertEigenToFeature(const Eigen::MatrixXf& feature){

	FeatureVector datum;
	for(int j = 0 ; j < feature.cols(); j++)
	{
		datum(j) = static_cast<double>(feature(0,j));
	}

	return datum;
}
void extract_features::trainfeatureClass(cv::Mat image, const pcl17::PointCloud<PointType>::Ptr &cloud,
		const image_geometry::PinholeCameraModel& model, const PointType& center, int index){

	feature_class feature;
	Eigen::MatrixXf final_feature;

	// Now only get the image around the current template
	cv::Point2d  uv_image;

	// Getting the centroid of the template
	geometry_msgs::Point centroid;
	centroid.x = center.x; centroid.y = center.y; centroid.z = center.z;

	if(!holes_){

		ROS_DEBUG("feature_learning::extract_features: Converting centroid to camera frame");

		pcl17::PointCloud<pcl17::PointXYZ> ray;
		ray.push_back(pcl17::PointXYZ(0,0,0));
		ray.push_back(pcl17::PointXYZ(center));
		ray.header.frame_id =  base_frame_;
		ray.header.stamp = ros::Time::now();

		try {
			ROS_VERIFY(listener_.waitForTransform(model.tfFrame(),base_frame_,ray.header.stamp, ros::Duration(10.0)));
			ROS_VERIFY(pcl17_ros::transformPointCloud(model.tfFrame(), ray,ray, listener_));
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}

		cv::Point3d push_3d;
		push_3d.x = static_cast<double>(ray.points[1].x);
		push_3d.y = static_cast<double>(ray.points[1].y);
		push_3d.z = static_cast<double>(ray.points[1].z);

		model.project3dToPixel(push_3d,uv_image);
		ROS_INFO("feature_learning::extract_features: Image start x:%f start y:%f ",uv_image.x,uv_image.y);
	}
	else
		uv_image.x = center_points_[index].x; uv_image.y = center_points_[index].y;

	ROS_INFO("feature_learning::extract_features: Image size rows:%d cols:%d ",image.rows,image.cols);

	if(((uv_image.x + 60) < image.rows) && ((uv_image.y + 60) < image.cols))
	{
		cv::Rect faceRect(uv_image.x - 60 ,uv_image.y - 60, 120, 120);
		image(faceRect).copyTo(image);
	}
	else{
		cv::Rect faceRect(uv_image.x - 50 ,uv_image.y - 50, 100, 100);
		image(faceRect).copyTo(image);
	}

	std::stringstream temp_filename;
	temp_filename<<"/tmp/sampleRect_"<<index<<".jpg";
	cv::imwrite(temp_filename.str(),image);


	ROS_INFO("feature_learning::extract_features: Initializing feature class for given template of size %d",cloud->points.size());
	feature.initialized_ = feature.initializeFeatureClass(image,cloud,centroid);

	ROS_INFO("feature_learning::extract_features: Starting feature computation process , Initialized %d",feature.initialized_);
	feature.computeFeature(final_feature);
	ROS_INFO("feature_learning::extract_features: Feature computation complete");
	// Use bag writer to write before and after bags with topics and name the Eigen matrix as the same thing
	std::stringstream eigen_filename;
	eigen_filename<<filename_<<".txt";
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
	writer_counter++;
	ROS_INFO_STREAM("feature_learning::extract_features: Updated counter number "<< writer_counter<<" written to "<<eigen_filename.str());


}

double extract_features::testfeatureClass(cv::Mat image, const pcl17::PointCloud<PointType>::Ptr &cloud,
		const image_geometry::PinholeCameraModel& model, const PointType& center, int index){

	feature_class feature;
	Eigen::MatrixXf final_feature;

	// Now only get the image around the current template
	cv::Point2d  uv_image;

	// Getting the centroid of the template
	geometry_msgs::Point centroid;
	centroid.x = center.x; centroid.y = center.y; centroid.z = center.z;

	if(!holes_){

		ROS_DEBUG("feature_learning::extract_features: Converting centroid to camera frame");

		pcl17::PointCloud<pcl17::PointXYZ> ray;
		ray.push_back(pcl17::PointXYZ(0,0,0));
		ray.push_back(pcl17::PointXYZ(center));
		ray.header.frame_id =  base_frame_;
		ray.header.stamp = ros::Time::now();

		try {
			ROS_VERIFY(listener_.waitForTransform(model.tfFrame(),base_frame_,ray.header.stamp, ros::Duration(10.0)));
			ROS_VERIFY(pcl17_ros::transformPointCloud(model.tfFrame(), ray,ray, listener_));
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
		}

		cv::Point3d push_3d;
		push_3d.x = static_cast<double>(ray.points[1].x);
		push_3d.y = static_cast<double>(ray.points[1].y);
		push_3d.z = static_cast<double>(ray.points[1].z);

		model.project3dToPixel(push_3d,uv_image);
		ROS_INFO("feature_learning::extract_features: Image size rows:%f cols:%f ",uv_image.x,uv_image.y);
	}
	else
		uv_image.x = center_points_[index].x; uv_image.y = center_points_[index].y;


	if(((uv_image.x + 60) < image.rows) && ((uv_image.y + 60) < image.cols))
	{
		cv::Rect faceRect(uv_image.x - 60 ,uv_image.y - 60, 120, 120);
		image(faceRect).copyTo(image);
	}
	else{
		cv::Rect faceRect(uv_image.x - 50 ,uv_image.y - 50, 100, 100);
		image(faceRect).copyTo(image);
	}

	std::stringstream temp_filename;
	temp_filename<<"/tmp/sampleRect_test_"<<index<<".jpg";
	cv::imwrite(temp_filename.str(),image);


	ROS_INFO("feature_learning::extract_features: Initializing feature class for given template of size %d",cloud->points.size());
	feature.initialized_ = feature.initializeFeatureClass(image,cloud,centroid);

	ROS_INFO("feature_learning::extract_features: Starting feature computation process , Initialized %d",feature.initialized_);
	feature.computeFeature(final_feature);
	ROS_INFO("feature_learning::extract_features: Feature computation complete");

	// Getting the centroid of the template
	action_point_.point = centroid;
	action_point_.header.frame_id = base_frame_;
	action_point_.header.stamp = input_cloud_->header.stamp;

	FeatureVector new_sample = convertEigenToFeature(final_feature);

	double label = learned_pfunct_(new_sample);

	return label;

}

std::vector<pcl17::PointCloud<pcl17::PointXYZ> > extract_features::extract_templates(const pcl17::PointCloud<pcl17::PointXYZ> &centroids){

	//	pcl17::PointCloud<PointType>::Ptr filtered_cloud (new pcl17::PointCloud<PointType>);
	//	pcl17::PassThrough<PointType> pass;

	std::vector<pcl17::PointCloud<PointType> > output_template_list;

	pcl17::PointCloud<PointType> template_cloud;
	ROS_INFO("feature_learning::extract_features: Starting cloud resizing %d",input_cloud_->points.size());
	std::vector<cv::Point2d> new_points;

	if(holes_)
	{
		new_points = center_points_;
		center_points_.clear();
	}
	ROS_INFO("feature_learning::extract_features: Entering pragma parallel loop ");

#pragma omp parallel for
	for(size_t t = 0;  t < centroids.points.size(); t++)
	{
		// Trying a pass through filter
		//		filtered_cloud.reset(new pcl17::PointCloud<PointType>);
		pcl17::PointCloud<PointType>::Ptr filtered_cloud (new pcl17::PointCloud<PointType>);
		pcl17::PassThrough<PointType> pass;

		pass.setInputCloud (input_cloud_);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (centroids.points[t].z - (BOX_HEIGHT_Z*2), centroids.points[t].z + (BOX_HEIGHT_Z*2));
		pass.filter (*filtered_cloud);

		pass.setInputCloud (filtered_cloud);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (centroids.points[t].y - (BOX_LENGTH_Y/2), centroids.points[t].y + (BOX_LENGTH_Y/2));
		pass.filter (*filtered_cloud);

		pass.setInputCloud (filtered_cloud);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (centroids.points[t].x - (BOX_WIDTH_X/2), centroids.points[t].x + (BOX_WIDTH_X/2));
		pass.filter (*filtered_cloud);

		if(filtered_cloud->size() == 0)
			continue;
		ROS_INFO("feature_learning::extract_features: critical pragma parallel loop ");

#pragma omp critical
		{
			if(holes_)
				center_points_.push_back(new_points[t]);

			output_template_list.push_back(*filtered_cloud);
			template_cloud += *filtered_cloud;
		}
	}

	template_cloud.header = input_cloud_->header;
	pcd_pub_.publish(template_cloud.makeShared());

	return output_template_list;
}

void extract_features::publishManipulationMarker(){

	visualization_msgs::Marker location_marker = getMarker(1);
	marker_array_.markers.clear(); // Clear current marker array
	location_marker.type = visualization_msgs::Marker::CUBE;
	location_marker.header = action_point_.header;
	location_marker.color.b = 1.0;
	location_marker.header.stamp = ros::Time();
	location_marker.pose.position.x = action_point_.point.x; location_marker.pose.position.y = action_point_.point.y; location_marker.pose.position.z = action_point_.point.z;
	marker_array_.markers.push_back(location_marker);
	m_array_pub_.publish(marker_array_);

}

bool extract_features::serviceCallback(ExtractFeatures::Request& request, ExtractFeatures::Response& response){

	// registering filenname for recording : TODO: check if this can avoid the multiple call problem
	if(!request.filename.empty())
		filename_ = request.filename;

	setInitialized(initialized());

	ROS_INFO("feature_learning::extract_features: Executing service, initialized %d",initialized_);

	if(initialized_)
	{
		ROS_INFO("feature_learning::extract_features: Updating all the data for processing");
		bool updated = updateTopics();
		if(updated)
		{
			bool test = static_cast<bool>(request.action);

			ROS_INFO("feature_learning::extract_features: Computing features");
		//	pcl17::PointCloud<PointType> cluster_centers = preProcessCloud_holes(input_image_,left_cam_,*processed_cloud_);
		    pcl17::PointCloud<PointType> cluster_centers = preProcessCloud_edges(input_image_,left_cam_,*processed_cloud_);

			if(cluster_centers.empty())
			{
				ROS_INFO("feature_learning::extract_features: Empty Cluster Centers");
				response.result = ExtractFeatures::Response::FAILURE;
				return true;
			}
			else
			{
				ROS_INFO("feature_learning::extract_features: Extracting templates from Cluster Centers");
				std::vector<pcl17::PointCloud<PointType> > templates = extract_templates(cluster_centers);
				ROS_INFO("feature_learning::extract_features: %d templates extracted ", templates.size());

				if(test)
				{
					// Getting the data storage templates
					double max_prob = 0; //geometry_msgs::PointStamped best_action_point;
					for(size_t i = 0; i < templates.size(); i++)
					{
						pcl17::PointCloud<PointType>::Ptr temp_cloud(new pcl17::PointCloud<PointType>(templates[static_cast<int>(i)]));
						ROS_INFO("feature_learning::extract_features: Extracting features from template of size %d",temp_cloud->points.size());
						if(temp_cloud->points.size() == 0)
							continue;

						double class_prob = testfeatureClass(input_image_,temp_cloud,left_cam_,cluster_centers.points[static_cast<int>(i)],
								static_cast<int>(i));

						if(class_prob > max_prob)
						{
							response.action_location = action_point_;
							response.result = ExtractFeatures::Response::SUCCESS;
						}

					}

					ROS_INFO("feature_learning::extract_features: Action point header frame: %s",action_point_.header.frame_id.c_str());

					publishManipulationMarker();

					if(max_prob == 0)
						response.result = ExtractFeatures::Response::FAILURE;
					return true;
				}
				else // Training Pipeline
				{

					//int random_index = returnRandIndex(templates.size());
					ROS_INFO("feature_learning::extract_features: Running Parallel feature processing");
#pragma omp parallel for
					for(size_t random_index = 0; random_index < templates.size(); random_index++)
					{
						pcl17::PointCloud<PointType>::Ptr temp_cloud(new pcl17::PointCloud<PointType>(templates[random_index]));

						ROS_INFO("feature_learning::extract_features: Extracting features from template of size %d",temp_cloud->points.size());
						if(temp_cloud->points.size() == 0)
							ROS_INFO("feature_learning::extract_features: %d template failed",random_index);
						else
						{
							trainfeatureClass(input_image_,temp_cloud,left_cam_,cluster_centers.points[random_index],random_index);
							geometry_msgs::PointStamped centroid;
							centroid.point.x = static_cast<double>(cluster_centers.points[random_index].x);
							centroid.point.y = static_cast<double>(cluster_centers.points[random_index].y);
							centroid.point.z = static_cast<double>(cluster_centers.points[random_index].z);
							centroid.header.frame_id = base_frame_;
							centroid.header.stamp = ros::Time();

							ROS_INFO("feature_learning::extract_features: returning success");
#pragma omp critical
							{
								response.training_centers.push_back(centroid);
								response.indicies.push_back(random_index);
							}
						}
					}
					if(response.indicies.size() >= 1)
						response.result = ExtractFeatures::Response::SUCCESS;
					else
						response.result = ExtractFeatures::Response::FAILURE;

					return true;
				}
			}
		}
		else
		{
			ROS_INFO("feature_learning::extract_features: Topics not updated");
			response.result = ExtractFeatures::Response::FAILURE;
			return true;
		}
	}
	else
	{
		ROS_INFO("feature_learning::extract_features: returning failed");
		response.result = ExtractFeatures::Response::FAILURE;
		return true;
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

		pcl17_ros::transformPointCloud(P, clusters[i], cloud_proj);

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

bool extract_features::updateTopics(){

	ROS_INFO("feature_learning::extract_features: Initializing extract features");
	sensor_msgs::Image::ConstPtr input_image = ros::topic::waitForMessage<sensor_msgs::Image>(input_image_topic_, nh_, ros::Duration(5.0));
	ROS_INFO("feature_learning::extract_features: Got input Image");

	sensor_msgs::PointCloud2ConstPtr ros_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(input_cloud_topic_,nh_, ros::Duration(5.0));
	ROS_INFO("feature_learning::extract_features: Got input pointcloud from topic %s",input_cloud_topic_.c_str());
	//input_rgb_cloud_
	//pcl17::fromROSMsg (*ros_cloud, *input_cloud_);
	pcl17::fromROSMsg (*ros_cloud, *input_rgb_cloud_);
	input_rgb_cloud_->header = ros_cloud->header;
	ROS_INFO("feature_learning::extract_features: Converted input pointcloud to ros message between frames");

	try {
		ROS_VERIFY(listener_.waitForTransform(base_frame_,input_rgb_cloud_->header.frame_id,input_rgb_cloud_->header.stamp, ros::Duration(10.0)));
		ROS_VERIFY(pcl17_ros::transformPointCloud(base_frame_,*input_rgb_cloud_,*input_rgb_cloud_,listener_));
	} catch (tf::TransformException ex) {
		ROS_ERROR("feature_learning::extract_features: %s",ex.what());
	}

	graph_segment convertor;
	input_image_ = convertor.returnCVImage(*input_image);
	convertor.setCVImage(input_image_);
	ros_image_ = convertor.returnRosImage(*input_image);
	ROS_INFO("feature_learning::extract_features: Converted image into ros image");


	// getting camera info
	sensor_msgs::CameraInfo::ConstPtr cam_info =
			ros::topic::waitForMessage<sensor_msgs::CameraInfo>(input_camera_info_, nh_, ros::Duration(10.0));
	left_cam_.fromCameraInfo(cam_info);
	cam_info_ = *cam_info;
	ROS_INFO("feature_learning::extract_features: got camera info");

	// Now getting the table top info

	// Calling tabletop segmentation service
	if (!ros::service::call(tabletop_service_, tabletop_srv_)) {
		ROS_ERROR("feature_learning::extract_features: Call to tabletop segmentation service failed");
		return false;
	}
	if (tabletop_srv_.response.result != tabletop_srv_.response.SUCCESS
			&& tabletop_srv_.response.result != tabletop_srv_.response.SUCCESS_NO_RGB) {

		ROS_ERROR("feature_learning::extract_features: Segmentation service returned error %d", tabletop_srv_.response.result);
		return false;
	}

	// transforming table points from table frame to "/BASE FRAME"

	tf::Transform table_tf;
	tf::poseMsgToTF(tabletop_srv_.response.table.pose.pose,table_tf);
	Eigen::Matrix4f table_transform;
	sensor_msgs::PointCloud2 transform_table_cloud;


	pcl17_ros::transformPointCloud(tabletop_srv_.response.table.pose.header.frame_id,
			table_tf,tabletop_srv_.response.table.table_points,
			transform_table_cloud);

	try {
		ROS_VERIFY(listener_.waitForTransform(base_frame_, transform_table_cloud.header.frame_id,transform_table_cloud.header.stamp, ros::Duration(10.0)));
		ROS_VERIFY(pcl17_ros::transformPointCloud(base_frame_, transform_table_cloud,transform_table_cloud, listener_));
	} catch (tf::TransformException ex) {
		ROS_ERROR("feature_learning::extract_features: %s",ex.what());
	}

	pcl17::PointCloud<pcl17::PointXYZ>::Ptr table_cloud_pcl(new pcl17::PointCloud<pcl17::PointXYZ>());
	pcl17::fromROSMsg(transform_table_cloud, *table_cloud_pcl);

	pcl17::PointIndices::Ptr inliers (new pcl17::PointIndices ());
	// Create the segmentation object
	pcl17::SACSegmentation<pcl17::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl17::SACMODEL_PLANE);
	seg.setMethodType (pcl17::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);

	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (table_cloud_pcl);
	seg.segment (*inliers, *table_coefficients_);

	pcl17::PointCloud<pcl17::PointXYZ>::Ptr table_base_points(new pcl17::PointCloud<pcl17::PointXYZ>());
	pcl17::copyPointCloud (*table_cloud_pcl, *inliers, *table_base_points);

	Eigen::Vector4f table_centroid;
	pcl17::compute3DCentroid(*table_base_points,table_centroid);
	table_height_ = table_centroid[2];

	//convert clusters to honeybee frame
	ROS_INFO("feature_learning::extract_features: Transforming clusters to color camera frame");

	std::vector<sensor_msgs::PointCloud2> clusters;
	for (int i = 0; i < (int) tabletop_srv_.response.clusters.size(); i++) {

		// transforming every cluster in the service
		sensor_msgs::PointCloud2 transform_cloud;

		try {
			tabletop_srv_.response.clusters[i].header.stamp = ros::Time(0);// TODO: <---- Change this later
			pcl17_ros::transformPointCloud(input_image->header.frame_id,
					tabletop_srv_.response.clusters[i], transform_cloud,
					listener_);
		} catch (tf::TransformException& ex) {
			ROS_ERROR("feature_learning::extract_features: Failed to transform cloud from frame %s into frame %s", tabletop_srv_.response.clusters[0].header.frame_id.c_str(),
					input_image->header.frame_id.c_str());
		}

		clusters.push_back(transform_cloud);
	}

	// Getting table points
	tf::Transform table_tf_b;
	ROS_INFO("feature_learning::extract_features: Cluster headers are frame %s ", tabletop_srv_.response.clusters[0].header.frame_id.c_str());
	pcl17::PointCloud<pcl17::PointXYZ>::Ptr cluster_clouds(new pcl17::PointCloud<pcl17::PointXYZ>());

	for(size_t cloud_count = 0;cloud_count< tabletop_srv_.response.clusters.size(); cloud_count++)
	{
		sensor_msgs::PointCloud2 cluster_points;

		pcl17_ros::transformPointCloud(tabletop_srv_.response.clusters[cloud_count].header.frame_id,
				table_tf_b,tabletop_srv_.response.clusters[cloud_count],
				cluster_points);

		try {
			ROS_VERIFY(listener_.waitForTransform(base_frame_,cluster_points.header.frame_id,
					cluster_points.header.stamp, ros::Duration(10.0)));
			ROS_VERIFY(pcl17_ros::transformPointCloud(base_frame_, cluster_points,
					cluster_points, listener_));
		} catch (tf::TransformException ex) {
			ROS_ERROR("feature_learning::extract_features: %s",ex.what());
		}

		pcl17::PointCloud<pcl17::PointXYZ>::Ptr temp_clouds(new pcl17::PointCloud<pcl17::PointXYZ>());

		pcl17::fromROSMsg(cluster_points, *temp_clouds);
		*cluster_clouds += *temp_clouds;
	}
	cluster_clouds->header.frame_id = base_frame_;
	cluster_clouds->header.stamp = ros::Time::now();
	ROS_INFO("feature_learning::extract_features: Clusters transformed to frame %s ", cluster_clouds->header.frame_id.c_str());
	*input_cloud_ = *cluster_clouds;

	pcl17::io::savePCDFileASCII ("/tmp/converted_cluster_clouds.pcd", *cluster_clouds);

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

	if(left_cam_.distortionCoeffs().empty() || input_image_.empty() || input_cloud_->empty())
		return false;
	else
		return true;
}


bool extract_features::initialized(){

	return true;
}

} //namespace

int main(int argc, char **argv) {
	ros::init(argc, argv, "extract_feature_srv");
	ros::NodeHandle nh;

	feature_learning::extract_features feature(nh);

	ros::spin();
	return 0;
}


