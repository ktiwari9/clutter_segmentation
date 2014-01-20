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
			local_cloud_(new pcl17::PointCloud<feature_class::PointType>),initialized_(false){
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
		const geometry_msgs::Point& centroid){

	// creating input for initial values
	inputImage(image);
	inputCloud(cloud);

	centroid_ = centroid;
}

void feature_class::computePushFeature(Eigen::MatrixXf &out_mat ){

	pcl17::PointCloud<PointNT>::Ptr cloud_normals (new pcl17::PointCloud<PointNT>);
	pcl17::search::KdTree<PointType>::Ptr tree (new pcl17::search::KdTree<PointType>);

	float model_ss_ (0.02f); // make it 0.25 if too slow TODO: fix this heuristic!
	do{

		pcl17::PointCloud<int> keypoint_indices;
		pcl17::UniformSampling<PointType> uniform_sampling;
		uniform_sampling.setInputCloud (local_cloud_);
		uniform_sampling.setRadiusSearch (model_ss_);
		uniform_sampling.compute (keypoint_indices);
		pcl17::copyPointCloud (*local_cloud_, keypoint_indices.points, *local_cloud_);
		ROS_INFO("Writing PCD Files");
		pcl17::io::savePCDFileASCII ("/tmp/test_pcd.pcd", *local_cloud_);
		model_ss_ += 0.005;
		ROS_INFO("feature_learning::feature_class: Size of input cloud %d ",local_cloud_->size());

	}while(local_cloud_->size() > 800);

	pcl17::NormalEstimationOMP<PointType, PointNT> ne;
	ne.setInputCloud (local_cloud_);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	ne.setSearchMethod (tree);

	// Output datasets

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloud_normals);

	// Create the FPFH estimation class, and pass the input dataset+normals to it
	pcl17::FPFHEstimationOMP <PointType, PointNT, pcl17::FPFHSignature33> fpfh;
	fpfh.setInputCloud (local_cloud_);
	fpfh.setInputNormals (cloud_normals);
	// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	fpfh.setSearchMethod (tree);

	// Output datasets
	pcl17::PointCloud<pcl17::FPFHSignature33>::Ptr fpfhs (new pcl17::PointCloud<pcl17::FPFHSignature33> ());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch (0.05);

	// Compute the features
	fpfh.compute (*fpfhs);


	ROS_INFO("feature_learning::feature_class: getting descriptors size %d",sizeof(fpfhs->points[0].histogram)/sizeof(float));
	//int histSize = descriptors->at(0).descriptor.size();
	Eigen::MatrixXf feature_mat;
	feature_mat = fpfhs->getMatrixXfMap (33,33,0); // use proper values
	ROS_INFO("feature_learning::feature_class: sending descriptors %d rows: %d",feature_mat.cols(),feature_mat.rows());
	//for dim, stride and offset, look at documentation for reasoning
	out_mat.resize(1,feature_mat.rows());
	out_mat<<feature_mat.rowwise().mean().transpose();
	out_mat.normalize();
}

void feature_class::computeFeature(Eigen::MatrixXf &out_mat){

	Eigen::MatrixXf colorHist, textonMap, entropyMap, pushFeature;

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
		out_mat.resize(1,colorHist.cols()+textonMap.cols()+entropyMap.cols()+pushFeature.cols()+2);

		/*ROS_INFO("feature_learning::feature_class: Resized features for input image : Size %d",out_mat.cols());
		ROS_INFO_STREAM("feature_learning::feature_class: Individual feature lenghts"<<std::endl
				<<"ColorHist :"<<colorHist.cols()<<std::endl<<colorHist<<std::endl
				<<"TextonMap :"<<textonMap.cols()<<std::endl<<textonMap<<std::endl
				<<"EntropyMap :"<<entropyMap.cols()<<std::endl<<entropyMap<<std::endl
				<<"pushFeature :"<<pushFeature.cols()<<std::endl<<pushFeature<<std::endl
				<<"graspPatch :"<<graspPatch.cols()<<std::endl<<graspPatch<<std::endl
				<<"Centroid x: "<<centroid_.x<<" y: "<<centroid_.y);
*/		// Normalized Individual features TODO: Do I need to normalize again?

		out_mat<<colorHist,textonMap,entropyMap,pushFeature,centroid_.x,centroid_.y;

		// Conditioning feature vector
		for(unsigned int i = 0; i<out_mat.cols();i++)
		{
			if(isnan(out_mat(0,i)))
				out_mat(0,i) = 0;
			if(isinf(out_mat(0,i)))
				out_mat(0,i) = 1000;
			if(abs(out_mat(0,i)) < 10e-6)
				out_mat(0,i) = 0.0;
			if(abs(out_mat(0,i)) > 1000)
				out_mat(0,i) = (out_mat(0,i) > 0) ? 1000 : -1000;
		}

		out_mat.normalize(); //TODO: Normalize with centroid? does this make sense???
	}

}

void feature_class::computeColorHist(Eigen::MatrixXf &out_mat){

	cv::MatND hist;
	cv::Mat hsv;

	ROS_DEBUG("Compute Patch Feature");

	cv::cvtColor(local_image_,hsv,CV_BGR2HSV);

	// let's quantize the hue to 30 levels
	// and the saturation to 32 levels
	int hbins = 3, sbins = 6;
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
    cv::normalize( hist, hist, 0, 180, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::cv2eigen(hist,out_mat);
	int rows = out_mat.rows(),cols = out_mat.cols();
	out_mat.conservativeResize(1,rows*cols);
	out_mat.normalize();
}


void feature_class::computeTextonMap(Eigen::MatrixXf &out_mat){

	cv::Mat img_gray_float(local_image_.rows, local_image_.cols, CV_64F);
	cv::Mat img_gray(local_image_.rows, local_image_.cols, CV_8U);
	cv::cvtColor(local_image_, img_gray, CV_BGR2GRAY);
	img_gray.convertTo(img_gray_float, CV_64F);
	img_gray_float/=255.0f;

	cv::imwrite("/tmp/input_gray.jpg",img_gray);
	cv::imwrite("/tmp/input_color.jpg",local_image_);
	ROS_DEBUG("Computing Texton Feature");

	int n_textons = 24;
	ROS_INFO("Initializing Texton Feature");

	learn_appearance::TextonHistogram thist(n_textons);

	ROS_INFO("Training texton Feature");
	//thist.Train(img_gray_float, cv::Mat());
	thist.InitializeCodebookFromImage(img_gray_float);
    cv::Mat texton_map = thist.getTextonMap();
    // collect histogram over table mask for foreground
    int channels[] = {0};
    int hist_size[] = {n_textons};
    float t_ranges[] = {0,n_textons};
    const float *ranges[] = {t_ranges};
    cv::MatND hist;
    cv::calcHist( &texton_map, 1, channels, cv::Mat(),
  		hist, 1, hist_size, ranges,
  		true, // the histogram is uniform
  		false );
    cv::normalize( hist, hist, 0, 100, cv::NORM_MINMAX, -1, cv::Mat() );
    ROS_INFO("Converting Texton Feature of size %d %d",hist.cols,hist.rows);
	cv::cv2eigen(hist,out_mat);
	int rows = out_mat.rows(),cols = out_mat.cols();
	out_mat.conservativeResize(1,rows*cols);
	out_mat.normalize();

}

void feature_class::computeEntropyMap(Eigen::MatrixXf &out_mat){


	// fix to compute entropy per cluster i.e entropy variance between mean entropy
	// and individual entropy of each superpixel.
	cv::Mat hsv_composite;

	ROS_DEBUG("Compute Color Histogram Feature");

	std::vector<cv::Mat> hsv;
	// Computing in HSV as jeannette suggested
	//cv::cvtColor(local_image_,hsv_composite,CV_BGR2HSV);
	cv::split(local_image_,hsv);

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
	out_mat.normalize();
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


