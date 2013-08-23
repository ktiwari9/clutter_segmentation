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

#include "graph_based_segmentation/segment.h"
#include "graph_based_segmentation/GraphSegment.h"

namespace graph_based_segmentation {

graph_segment::graph_segment(cv::Mat input):
    sigma_(0.25),k_(1000),min_size_(100){//sigma_(0.5),k_(1000),min_size_(20)

  image<rgb>* rgb_input = convertcvMattoNative(input);

  image<rgb> *seg = segment_image(rgb_input,sigma_,k_,min_size_,&num_ccs_);

  input_ = convertNativetocvMat(seg);


}
graph_segment::graph_segment(){}

graph_segment::~graph_segment(){}

cv::Mat graph_segment::convertNativetocvMat(image<rgb>* input){

	    int w = input->width();
	    int h = input->height();
	    cv::Mat new_image = cv::Mat::zeros(h,w,CV_8UC3);
	    cv::Mat_<cv::Vec3b> _I = new_image;

	    for(int i =0; i<h; i++){
	        for(int j=0; j<w; j++){

	            rgb curr =input->data[j+i*w];
	            _I(i,j)[0] = curr.r;
	            _I(i,j)[1] = curr.g;
	            _I(i,j)[2] = curr.b;
	        }
	    }
	    new_image = _I;

	    return new_image;
}


image<rgb>* graph_segment::convertcvMattoNative(cv::Mat input){

		int h = input.rows;
	    int w = input.cols;
	    image<rgb> *im = new image<rgb>(w,h);

	    for(int i =0; i<h; i++){
	        for(int j=0; j<w; j++){
	            rgb curr;

	            // Alter bgr pattern if incoming pattern is different
	            curr.r = input.at<cv::Vec3b>(i,j)[2];
	            curr.g = input.at<cv::Vec3b>(i,j)[1];
	            curr.b = input.at<cv::Vec3b>(i,j)[0];
	            im->data[j+i*w] = curr;
	        }
	    }
	    return im;
}

cv::Mat graph_segment::returnCVImage(const sensor_msgs::Image & img) {

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img);
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		exit(0);
	}

	return cv_ptr->image;
}

sensor_msgs::ImagePtr graph_segment::returnRosImage(const sensor_msgs::Image rgb_image){

	cv_bridge::CvImage out_msg;

	//Get header and encoding from your input image
	out_msg.header   = rgb_image.header;
	out_msg.encoding = rgb_image.encoding;
	out_msg.image    = input_;

	ROS_INFO("Populating response with type sensor_msgs/Image");
	return out_msg.toImageMsg();
}



cv::Mat graph_segment::getSegmentedImage(){return input_;}
}



