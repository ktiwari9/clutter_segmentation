/*
 * Copyright (c) 2013, Bharath Sankaran, University of Southern California (CLMC)
 * (bsankara@usc.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  1.Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  2.Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  3.The name of Bharath Sankaran or the USC-CLMC may not be used to
 *    endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Test executable for creating and detecting holes in image */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "feature_learning/macros_time.hpp"
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		std::cout<< "please provide and input image"<<std::endl;
		exit(0);
	}

	std::cout<<" Reading image ...."<<std::endl;
	std::string img_name(argv[1]);
	cv::Mat img = cv::imread(img_name.c_str());
	if(img.data == NULL){
		std::cout << "Error: Image " << img_name <<" could not be read." << std::endl;
		exit(-1);
	}

	cv::Mat img_gray;
	cv::cvtColor(img,img_gray,CV_BGR2GRAY);
	std::vector<std::vector<cv::Point> > contours_unordered;
	// closing all contours via dialation
	cv::Mat bw_new;
	// to fill cluster masks
	cv::Mat element = cv::getStructuringElement(2, cv::Size(5,5));

	/// Apply the dilation operation
	cv::dilate(img_gray.clone(), bw_new, element);

	//get center of image and put a bounding box around it
	int center_x = floor(img.rows/2);
	int center_y = floor(img.cols/2);

	std::cout<<"Center point x:"<<center_x<<" y:"<<center_y<<std::endl;

	//cv::Rect myROI(center_y-100, center_x-100,center_y+100,center_x+100);
	//cv::Mat croppedImage = bw_new(myROI);
	// convert to BW
	cv::Mat image_bw = bw_new;//croppedImage > 50;
	int thresh = 100;
	cv::Mat canny_output;
	cv::Canny(image_bw, canny_output, thresh, thresh*2, 3 );
	cv::imwrite("/tmp/canny.jpg",canny_output);
	cv::imwrite("/tmp/holesBW.jpg",bw_new);

	cv::Mat new_canny = cv::Mat::zeros(bw_new.size(), CV_8UC1);
	new_canny = ~(bw_new > 0);
	//bw_new.convertTo(bw_new,CV_8UC1);

	std::cerr<<"Did I get till here "<<bw_new.type()<<" "<<bw_new.channels()<<std::endl;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(new_canny.clone(), contours_unordered, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
	std::cout<<"what about till here "<<std::endl;
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


	// show colour coded texture map
	cv::imshow("Image Gray", new_canny);
	cv::waitKey();





}

