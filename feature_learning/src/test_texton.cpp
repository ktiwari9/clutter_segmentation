/*
 * Copyright (c) 2013, Jeannette Bohg and Bharath Sankaran, MPI for Intelligent Systems
 * (jbohg@tuebingen.mpg.de)
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
 *  3.The name of Jeannette Bohg or the MPI-IS may not be used to
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

/* Test executable for learning table features and detecting nuts on it. */

#include <opencv2/highgui/highgui.hpp>
#include "feature_learning/macros_time.hpp"
#include <learn_appearance/texton_hist.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  int n_textons = 32; // first try with 24, then 32
  learn_appearance::TextonHistogram thist(n_textons);

  // DEBUG: Visualise Filters
  /*
  std::vector<cv::Mat> filters = thist.getFilterBank();

  cv::namedWindow( "Filter", 1 );
  for(size_t i=0; i< filters.size(); ++i){
    filters[i]*=100.0f;
    cv::imshow( "Filter", filters[i] );
    cv::waitKey();
  }
  */

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
  cv::Mat img_gray_float(img.rows, img.cols, CV_64F);
  cv::Mat img_gray(img.rows, img.cols, CV_8U);
  cv::cvtColor(img, img_gray, CV_BGR2GRAY);
  img_gray.convertTo(img_gray_float, CV_64F);
  img_gray_float/=255.0f;

  std::cout<<" Initializing the code book from the image ...."<<std::endl;


  INIT_PROFILING
  thist.InitializeCodebookFromImage(img_gray_float);
  MEASURE("initialize_codebook")
  std::cout<<" Done getting code book from image ...."<<std::endl;


  std::cout << "Color Map " << std::endl;
  std::vector<cv::Scalar> color_map(n_textons);
  for(size_t i=0; i<color_map.size(); ++i){
    color_map[i] = cv::Scalar( rand()&255, rand()&255, rand()&255 );
    std::cout << i << ": " << color_map[i](0) << " " << color_map[i](1) << " " << color_map[i](2) << std::endl;
  }
  std::cout<<" Getting textonMap from image ...."<<std::endl;

  cv::Mat texton_map = thist.getTextonMap();
  std::cout<<" got textonMap from image ...."<<std::endl;
  cv::Mat labeled = cv::Mat(texton_map.rows, texton_map.cols, CV_8UC3);
  for(int i = 0; i < texton_map.rows; i++)
    {
      const unsigned char* texton_i = texton_map.ptr<unsigned char>(i);
      unsigned char* labeled_i = labeled.ptr<unsigned char>(i);
      for(int j = 0; j < texton_map.cols; j++){
	cv::Scalar color = color_map[texton_i[j]];
	for(int c=0; c<3; ++c){
	  labeled_i[j*3+c] = color(c);
	}
      }
    }

  // show colour coded texture map
  cv::imshow("Map", labeled);
  cv::waitKey();

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

  // normalise
  cv::normalize( hist, hist, 0, 1.0, cv::NORM_MINMAX, -1, cv::Mat() );

  // pretty picture
  int scale = 10;
  int height = 200;
  cv::Mat hist_img = cv::Mat::ones(height, n_textons*scale, CV_8UC3);
  hist_img = cv::Scalar(200,200,200);

  for( int h = 0; h < n_textons; h++ )
    {
      float binVal = hist.at<float>(h, 0);
      cv::rectangle( hist_img,
		     cv::Point( (h+1)*scale - 1, height - (binVal * height)),
		     cv::Point( h*scale, height),
		     color_map[h],
		     CV_FILLED );
    }

  cv::imshow("hist", hist_img);
  cv::waitKey();




}

