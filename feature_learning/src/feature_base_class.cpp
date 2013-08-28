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


#include "feature_learning/feature_base_class.hpp"



namespace feature_learning{

feature_class::feature_class():
			local_cloud_(new pcl::PointCloud<feature_class::PointType>),initialized_(false){
	// Empty Constructor
}

feature_class::~feature_class(){}
// Empty Destructor

void feature_class::inputImage(cv::Mat input){
	local_image_ = input;
}

void feature_class::inputImage(cv::Mat input, cv::Mat segment){

	local_image_ = input;
	local_segment_ = segment;
}

void feature_class::inputCloud(const PointCloudPtr &input_cloud_ptr){
	local_cloud_.reset();
	local_cloud_ = input_cloud_ptr;

}

bool feature_class::initializeFeatureClass(cv::Mat image, const PointCloudPtr &cloud,
		const geometry_msgs::PoseStamped &viewpoint,const geometry_msgs::Pose& surface,
		const geometry_msgs::PoseStamped& gripper_pose){

	// creating input for initial values
	inputImage(image);
	inputCloud(cloud);

	bool verify = initializeGraspPatchParams(viewpoint,surface,gripper_pose);
	if(verify)
		return true;
	else
		return false;
}

bool feature_class::initializeGraspPatchParams(const geometry_msgs::PoseStamped &viewpoint,
		const geometry_msgs::Pose &surface, const geometry_msgs::PoseStamped &gripper_pose){


	// Getting view point translation
	view_point_translation_.x() = viewpoint.pose.position.x;
	view_point_translation_.y() = viewpoint.pose.position.y;
	view_point_translation_.z() = viewpoint.pose.position.z;

	// Getting view point rotation
	view_point_rotation_.w() = viewpoint.pose.orientation.w;
	view_point_rotation_.x() = viewpoint.pose.orientation.x;
	view_point_rotation_.y() = viewpoint.pose.orientation.y;
	view_point_rotation_.z() = viewpoint.pose.orientation.z;

	gripper_pose_ = gripper_pose;
	surface_ = surface;

	return true;

}

void feature_class::computePushFeature(Eigen::MatrixXf &out_mat ){

	// TODO: try multiple features like USC and shape context to see what works
	pcl::ShapeContext3DEstimation <PointType,PointNT,pcl::SHOT> shape_context;
	pcl::PointCloud<pcl::SHOT>::Ptr descriptors (new pcl::PointCloud<pcl::SHOT>);


	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());

	// Normal Estimation
	ROS_INFO("Size of input cloud %d ",local_cloud_->size());
	pcl::NormalEstimation<PointType, PointNT> normalEstimation;
	normalEstimation.setInputCloud (local_cloud_);
	normalEstimation.setSearchMethod (tree);

	pcl::PointCloud<PointNT>::Ptr cloudWithNormals (new	pcl::PointCloud<PointNT>);
	normalEstimation.setRadiusSearch (0.03);
	normalEstimation.compute (*cloudWithNormals);

	shape_context.setInputCloud(local_cloud_);
	shape_context.setInputNormals(cloudWithNormals);

	// Use the same KdTree from the normal estimation
	shape_context.setSearchMethod (tree);
	shape_context.setRadiusSearch (0.2);

	// Actually compute the shape contexts
	//shape_context.initCompute();
	shape_context.compute (*descriptors);

	int histSize = descriptors->at(0).descriptor.size();
	out_mat = descriptors->getMatrixXfMap (histSize, histSize + 9, 0); // use proper values
	//for dim, stride and offset, look at documentation for reasoning

	long int rows = out_mat.rows(),cols = out_mat.cols();
	out_mat.conservativeResize(1,rows*cols);
	//shape_context.computeFeatureEigen(out_mat); // verify this later

}



void feature_class::computeRefPoint(Eigen::Vector3d& result, const geometry_msgs::PoseStamped& gripper_pose) const
{
	Eigen::Vector3d trans(gripper_pose.pose.position.x, gripper_pose.pose.position.y, gripper_pose.pose.position.z);
	Eigen::Quaterniond rot(gripper_pose.pose.orientation.w, gripper_pose.pose.orientation.x,
			gripper_pose.pose.orientation.y, gripper_pose.pose.orientation.z);

	Eigen::Vector3d delta;
	//Set Template extraction point to delta
	//TODO: Load this from a Yaml file like in Alex's code???
	delta << 0,0,0; // load this from a yaml file later
	result = rot * delta + trans;
}


//template <class Derived>
void feature_class::computeFeature(Eigen::MatrixXf &out_mat){

	Eigen::MatrixXf colorHist, textonMap, entropyMap, pushFeature, graspPatch;

	if(initialized_)
	{
		ROS_INFO("feature_learning::feature_class: Getting color histogram of input image");
		computeColorHist(colorHist);
		ROS_INFO("feature_learning::feature_class: Getting texton histogram of input image");
		computeTextonMap(textonMap);
		ROS_INFO("feature_learning::feature_class: Getting entropy map of input image");
		computeEntropyMap(entropyMap);
		ROS_INFO("feature_learning::feature_class: Getting push features of input image");
		computePushFeature(pushFeature);
		ROS_INFO("feature_learning::feature_class: Getting grasp patch in input image");
		computeGraspPatch(graspPatch);
		out_mat.resize(1,colorHist.cols()+textonMap.cols()+entropyMap.cols()+pushFeature.cols()+graspPatch.cols());
		//TODO: Do I need to normalize these features???
		ROS_INFO("feature_learning::feature_class: Resized features for input image");
		out_mat<<colorHist,textonMap,entropyMap,pushFeature,graspPatch;
	}

}

//template <class Derived>
void feature_class::computeGraspPatch(Eigen::MatrixXf &out_mat){

	HeightmapSampling t_gen(view_point_translation_,view_point_rotation_);
	t_gen.initialize(*local_cloud_,surface_);
	Eigen::Vector3d ref_point;
	computeRefPoint(ref_point, gripper_pose_);
	GraspTemplate templt;
	t_gen.generateTemplateOnHull(templt, ref_point);

	// Now convert output to Eigen Matrix
	std::vector<double> grasp_patch = templt.heightmap_.getGrid();
	std::vector<float> grasp_patch_float(grasp_patch.begin(), grasp_patch.end());
	// converting to float as all pcl types deal with float
	Eigen::Map<Eigen::MatrixXf >(grasp_patch_float.data(),1,grasp_patch_float.size()) = out_mat; // Eigen map is used to convert std_vector to Eigen_Matrix
}

//template <class Derived>
void feature_class::computeColorHist(Eigen::MatrixXf &out_mat){

	cv::MatND hist;
	cv::Mat hsv;

	ROS_DEBUG("Compute Patch Feature");

	cv::cvtColor(local_image_,hsv,CV_BGR2HSV);

	// let's quantize the hue to 30 levels
	// and the saturation to 32 levels
	int hbins = 30, sbins = 32;
	int histSize[] = {hbins, sbins};
	// hue varies from 0 to 179, see cvtColor
	float hranges[] = { 0, 180 };
	// saturation varies from 0 (black-gray-white) to
	// 255 (pure spectrum color)
	float sranges[] = { 0, 256 };
	const float* ranges[] = { hranges, sranges };
	// we compute the histogram from the 0-th and 1-st channels
	int channels[] = {0, 1};

	cv::calcHist( &hsv, 1, channels, cv::Mat(), // do not use mask
			hist, 2, histSize, ranges,
			true, // the histogram is uniform
			false );

	cv::cv2eigen(hist,out_mat);
	int rows = out_mat.rows(),cols = out_mat.cols();
	out_mat.conservativeResize(1,rows*cols);
}


//template <class Derived>
void feature_class::computeTextonMap(Eigen::MatrixXf &out_mat){

	cv::Mat img_gray_float(local_image_.rows, local_image_.cols, CV_64F);
	cv::Mat img_gray(local_image_.rows, local_image_.cols, CV_8U);
	cv::cvtColor(local_image_, img_gray, CV_BGR2GRAY);
	img_gray.convertTo(img_gray_float, CV_64F);
	img_gray_float/=255.0f;

	cv::imwrite("/tmp/input_gray.jpg",img_gray);
	cv::imwrite("/tmp/input_color.jpg",local_image_);
	ROS_DEBUG("Computing Texton Feature");

	int n_textons = 32;
	ROS_INFO("Initializing Texton Feature");

	learn_appearance::TextonHistogram thist(n_textons);

	ROS_INFO("Training texton Feature");
	//thist.Train(img_gray_float, cv::Mat());
	thist.InitializeCodebookFromImage(img_gray_float);
	cv::MatND hist = thist.getTextonHist();
	ROS_INFO("Converting Texton Feature");
	cv::cv2eigen(hist,out_mat);
	int rows = out_mat.rows(),cols = out_mat.cols();
	out_mat.conservativeResize(1,rows*cols);

}

//template <class Derived>
void feature_class::computeEntropyMap(Eigen::MatrixXf &out_mat){


	// fix to compute entropy per cluster i.e entropy variance between mean entropy
	// and individual entropy of each superpixel.
	cv::Mat hsv_composite;

	ROS_DEBUG("Compute Color Histogram Feature");

	std::vector<cv::Mat> hsv;
	// Computing in HSV as jeannette suggested
	cv::cvtColor(local_image_,hsv_composite,CV_BGR2HSV);
	cv::split(hsv_composite,hsv);

	/// Establish the number of bins
	int histSize = 256;

	/// Set the ranges ( for B,G,R) )
	float range[] = { 0, 256 } ;
	float range_h[] = {0, 180};
	const float* histRange = { range };
	const float* histhRange = { range_h };

	// Now get the histograms for each channel
	/// Compute the histograms:
	cv::Mat h_hist, s_hist, v_hist;
	cv::calcHist( &hsv[0], 1, 0, cv::Mat(), h_hist, 1, &histSize, &histRange, true, cv::accumulate );
	cv::calcHist( &hsv[1], 1, 0, cv::Mat(), s_hist, 1, &histSize, &histRange, true, cv::accumulate );
	cv::calcHist( &hsv[2], 1, 0, cv::Mat(), v_hist, 1, &histSize, &histRange, true, cv::accumulate );

	// Normalize the histogram
	/// Normalize the result to [ 0, histImage.rows ]

	cv::normalize(h_hist, h_hist, 0, local_image_.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(s_hist, s_hist, 0, local_image_.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(v_hist, v_hist, 0, local_image_.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	float entropy_h = 0.0, entropy_s = 0.0, entropy_v = 0.0;
	float frequency = getFrequencyOfBin(h_hist);
	for( int i = 1; i < histSize; i++ )
	{
		float Hc = abs(getHistogramBinValue(h_hist,i));
		entropy_h += -(Hc/frequency) * log10((Hc/frequency));
	}
	frequency = getFrequencyOfBin(s_hist);
	for( int i = 1; i < histSize; i++ )
	{
		float Hc = abs(getHistogramBinValue(s_hist,i));
		entropy_s += -(Hc/frequency) * log10((Hc/frequency));
	}
	frequency = getFrequencyOfBin(v_hist);
	for( int i = 1; i < histSize; i++ )
	{
		float Hc = abs(getHistogramBinValue(v_hist,i));
		entropy_v += -(Hc/frequency) * log10((Hc/frequency));
	}

	out_mat.resize(1,3);
	out_mat<< entropy_h, entropy_s,entropy_v;

}


float feature_class::getHistogramBinValue(cv::Mat hist, int binNum)
{
	return hist.at<float>(binNum);
}
float feature_class::getFrequencyOfBin(cv::Mat channel)
{
	int histSize = 255;
	float frequency = 0.0;
	for( int i = 1; i < histSize; i++ )
	{
		float Hc = abs(getHistogramBinValue(channel,i));
		frequency += Hc;
	}
	return frequency;
}


} //namespace


