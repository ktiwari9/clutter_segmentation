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

#include <ros/ros.h>
#include "feature_learning/macros_time.hpp"
//PCL includes
#include <pcl17/point_cloud.h>
#include <pcl17/point_types.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/pcl_base.h>
#include <pcl17/PointIndices.h>
#include <pcl17/ModelCoefficients.h>
#include <pcl17/features/normal_3d.h>
#include <pcl17/features/normal_3d_omp.h>
#include <pcl17/features/organized_edge_detection.h>
#include <pcl17_ros/point_cloud.h>

#include <iostream>

using namespace std;
typedef pcl17::PointXYZ PointType;
typedef pcl17::Normal PointNT;


std::vector<pcl17::PointIndices> compute_edges(const pcl17::PointCloud<PointType>::Ptr &input_cloud){


	pcl17::PointCloud<PointType>::Ptr filtered_cloud (new pcl17::PointCloud<PointType>);
	pcl17::PointCloud<PointNT>::Ptr cloud_normals (new pcl17::PointCloud<PointNT>);
	pcl17::search::KdTree<PointType>::Ptr tree (new pcl17::search::KdTree<PointType>);

	pcl17::NormalEstimationOMP<PointType, PointNT> ne;
	//pcl17::NormalEstimation<PointType, PointNT> ne;
	ne.setInputCloud (input_cloud);
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


	pcl17::OrganizedEdgeFromNormals<PointType,PointNT, pcl17::Label> oed;
	oed.setInputCloud (input_cloud);
	oed.setInputNormals (cloud_normals);
	oed.setDepthDisconThreshold (0.005); // 2cm
	oed.setMaxSearchNeighbors (500);
	pcl17::PointCloud<pcl17::Label> labels;
	std::vector<pcl17::PointIndices> label_indices;

	ROS_INFO("feature_learning::extract_features: Computing organized edges");
	oed.compute (labels, label_indices);
	ROS_INFO("feature_learning::extract_features: Computed labels size:%d", labels.points.size());

	return label_indices;

}

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		std::cout<< "Please provide input type: Usage: edge_extraction <0/1>"<<std::endl;
		std::cout<< "0 for pcd input, 1 for topic stream"<<std::endl;
		exit(0);
	}

	std::string input_string(argv[1]);
	int input_type = atoi(input_string.c_str());

	pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud (new pcl17::PointCloud<pcl17::PointXYZ>);

	if(input_type == 0)
	{
		std::string file_name;
		std::cout<<"Enter name (path) of pcd file "<<std::endl;
		std::cin>>file_name;

		pcl17::io::loadPCDFile(file_name,*cloud);

		std::vector<pcl17::PointIndices> label_indices = compute_edges(cloud);

		pcl17::PointCloud<PointType>::Ptr occluding_edges (new pcl17::PointCloud<PointType>),
				occluded_edges (new pcl17::PointCloud<PointType>),
				boundary_edges (new pcl17::PointCloud<PointType>),
				high_curvature_edges (new pcl17::PointCloud<PointType>),
				rgb_edges (new pcl17::PointCloud<PointType>);

		pcl17::copyPointCloud (*cloud, label_indices[0].indices, *boundary_edges);
		pcl17::copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
		pcl17::copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
		pcl17::copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);
		pcl17::copyPointCloud (*cloud, label_indices[4].indices, *rgb_edges);

		pcl17::PCDWriter writer;
		writer.writeASCII(std::string("/tmp/boundary_edges.pcd"), *boundary_edges);
		writer.writeASCII(std::string("/tmp/occluding_edges.pcd"),*occluding_edges);
		writer.writeASCII(std::string("/tmp/occluded_edges.pcd"), *occluded_edges);
		writer.writeASCII(std::string("/tmp/high_curvature_edges.pcd"), *high_curvature_edges);
		writer.writeASCII(std::string("/tmp/rgb_edges.pcd"), *rgb_edges);

		return 1;

	}
	else
	{

		ros::init(argc, argv, "test_edge_extraction");
		ros::NodeHandle nh;
		ros::Publisher edge_pub = nh.advertise<pcl17::PointCloud<PointType> >("/edge_points", 1);

		std::string topic_name;
		ROS_INFO("Enter topic name to listen to ");
		std::cin >> topic_name;

		while(nh.ok()){

			sensor_msgs::PointCloud2ConstPtr ros_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic_name, nh, ros::Duration(5.0));
			pcl17::fromROSMsg (*ros_cloud, *cloud);

			std::vector<pcl17::PointIndices> label_indices = compute_edges(cloud);

			pcl17::PointCloud<PointType>::Ptr occluding_edges (new pcl17::PointCloud<PointType>),
					occluded_edges (new pcl17::PointCloud<PointType>),
					boundary_edges (new pcl17::PointCloud<PointType>),
					high_curvature_edges (new pcl17::PointCloud<PointType>),
					rgb_edges (new pcl17::PointCloud<PointType>);

			pcl17::copyPointCloud (*cloud, label_indices[0].indices, *boundary_edges);
			pcl17::copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
			pcl17::copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
			pcl17::copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);
			pcl17::copyPointCloud (*cloud, label_indices[4].indices, *rgb_edges);

			pcl17::PointCloud<PointType>::Ptr edge_list (new pcl17::PointCloud<PointType>);

			*edge_list += *boundary_edges; *edge_list += *occluding_edges;
			*edge_list += *occluded_edges; *edge_list += *high_curvature_edges; *edge_list += *rgb_edges;

			edge_list->header = cloud->header;
			edge_list->header.stamp = ros::Time();
			edge_pub.publish(edge_list->makeShared());

			ros::spinOnce();

		}
	}
}

