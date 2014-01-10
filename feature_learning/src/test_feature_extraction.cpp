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
//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/usc.h> // Unique shape context feature
#include <pcl/filters/crop_box.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <iostream>

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::Normal PointNT;

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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(img_name,*cloud);

	std::cout<<"feature_learning::feature_class: Size of input cloud "<<cloud->size()<<std::endl;

	// TODO: try multiple features like USC and shape context to see what works
	pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptors (new pcl::PointCloud<pcl::ShapeContext1980>);

	pcl::UniqueShapeContext<PointType,pcl::ShapeContext1980> unique_context;

	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
	// Normal Estimation
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud);

	std::cout<<"feature_learning::feature_class: Size of voxelized input cloud "<<cloud->size()<<std::endl;

	float model_ss_ (0.05f); // make it 0.25 if too slow

	pcl::PointCloud<int> keypoint_indices;

	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud (cloud);
	uniform_sampling.setRadiusSearch (model_ss_);
	uniform_sampling.compute (keypoint_indices);
	std::cout<<"No of Keypoints found "<<(int)keypoint_indices.points.size()<<std::endl;
	pcl::copyPointCloud (*cloud, keypoint_indices.points, *cloud);
	std::cout<<"feature_learning::feature_class: Size of sampled input cloud "<<cloud->size()<<std::endl;

	pcl::PointCloud<pcl::PointXYZ> copy_cloud(*cloud);
	pcl::PCDWriter writer;
	writer.writeASCII(std::string("/tmp/sampled_pcd.pcd"), copy_cloud);

	// FOR USC parameters check test file
	std::cout<<"feature_learning::feature_class: Shape Context start "<<std::endl;
	unique_context.setInputCloud(cloud);
	unique_context.setSearchMethod(tree);
	unique_context.setRadiusSearch(0.5);
	// Use the same KdTree from the normal estimation
	unique_context.compute(*descriptors);

	std::cout<<"feature_learning::feature_class: getting descriptors "<<std::endl;
	int histSize = 1980;//descriptors->at(0).descriptor.size();
	Eigen::MatrixXf out_mat;
	std::cout<<"feature_learning::feature_class: descriptors size "<<histSize<<std::endl;
	out_mat = descriptors->getMatrixXfMap (histSize, histSize + 9, 0); // use proper values
	//for dim, stride and offset, look at documentation for reasoning
	long int rows = out_mat.rows(),cols = out_mat.cols();
	std::cout<<"feature_learning::feature_class: rows and cols "<<rows<<" "<<cols<<std::endl;
	Eigen::MatrixXf final_mat(1,cols);
	final_mat<<out_mat.colwise().mean();
	std::cout<<"Colwise mean "<<final_mat<<std::endl;

	std::stringstream eigen_filename;
	eigen_filename<<"/tmp/eigen_test.txt";
	ofstream ofs(eigen_filename.str().c_str(),ios::out | ios::trunc);
	if(ofs)
	{
		// instructions
		ofs << std::endl;
		ofs << final_mat;
		ofs.close();
	}
	else  // sinon
	{
		std::cerr << "Erreur à l'ouverture !" << std::endl;
	}



}

