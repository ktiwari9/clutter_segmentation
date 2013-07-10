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
#include <learn_appearance/texton_hist.h>

namespace feature_learning{

feature_class::feature_class(){
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

template <class Derived>
void feature_class::computeColorHist(Eigen::MatrixBase<Derived> &out_mat){

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
}

template <class Derived>
void feature_class::computeTextonMap(Eigen::MatrixBase<Derived> &out_mat){


	cv::Mat img_gray_float(local_image_.rows, local_image_.cols, CV_64F);
	cv::Mat img_gray(local_image_.rows, local_image_.cols, CV_8U);
	cv::cvtColor(local_image_, img_gray, CV_BGR2GRAY);
	img_gray.convertTo(img_gray_float, CV_64F);
	img_gray_float/=255.0f;

	ROS_DEBUG("Computing Texton Feature");

	int n_textons = 32;
	learn_appearance::TextonHistogram thist(n_textons);

	thist.Train(img_gray_float, cv::Mat());
	cv::MatND hist = thist.getTextonHist();
	cv::cv2eigen(hist,out_mat);

}

template <class Derived>
void feature_class::computeEntropyMap(Eigen::MatrixBase<Derived> &out_mat){


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

float getHistogramBinValue(cv::Mat hist, int binNum)
{
   return hist.at<float>(binNum);
}
float getFrequencyOfBin(cv::Mat channel)
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

}


