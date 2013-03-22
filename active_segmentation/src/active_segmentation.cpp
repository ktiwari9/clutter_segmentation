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
 * @b computes connectivity graph for static segmented connected components
 */

#include "graph_module/EGraph.h"
#include "active_segmentation/active_segmentation.hpp"
#include "tabletop_segmenter/TabletopSegmentation.h"
// PCL includes
#include "pcl_ros/transforms.h"
#include <pcl/filters/extract_indices.h>

#define DEBUG true

namespace active_segmentation {

active_segment::active_segment(cv::Mat input, cv::Mat segment, graph_module::EGraph graph):
		    graph_iter_(Container(graph_list_)){

  segment_ = segment;
  input_ = input;

}

active_segment::active_segment(ros::NodeHandle & nh):
			    nh_(nh),nh_priv_("~"),graph_iter_(Container(graph_list_)),
			    table_coefficients_(new pcl::ModelCoefficients ()),first_call_(true){

  nh_priv_.param<std::string>("static_service",static_service_,std::string("/static_segment_srv"));
  nh_priv_.param<std::string>("window_thread",window_thread_,std::string("Display window"));
  nh_priv_.param<bool>("tracking",tracking_,false);
  nh_priv_.param<std::string>("left_camera_topic",left_camera_topic_,std::string("/Honeybee/left/camera_info"));
  nh_priv_.param<std::string>("tabletop_service", tabletop_service_,std::string("/tabletop_segmentation"));
  nh_priv_.param<std::string>("rgb_input",rgb_topic_,std::string("/Honeybee/left/image_rect_color"));

  pose_publisher_ = nh_priv_.advertise<geometry_msgs::PoseStamped>("/push_pose",5);

  cv::namedWindow( window_thread_.c_str(), CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::startWindowThread();

  // Initializing some values
  pyramid_scale_ = 0.5;
  levels_ = 3;
  win_size_ = 11;
  of_iter_ = 5;
  poly_pixel_ = 6;
  poly_sigma_ = 1.25;
  push_hand_pose_.header.frame_id = "/BASE";


}



active_segment::~active_segment(){ cv::destroyWindow(window_thread_.c_str());}

cv::Mat active_segment::returnCVImage(const sensor_msgs::Image & img) {

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    exit(0);
  }

  return cv_ptr->image;
}

void active_segment::initGraphHelpers(){

	// Getting table top parameters to retrieve plane parameters
	tabletop_segmenter::TabletopSegmentation table_srv_;
	while (!ros::service::waitForService(tabletop_service_,
			ros::Duration().fromSec(3.0)) && nh_.ok()) {
		ROS_INFO("Waiting for service %s...", tabletop_service_.c_str());
	}

	if (!ros::service::call(tabletop_service_, table_srv_)) {
		ROS_ERROR("Call to segmentation service failed");
		got_table_ = false;
		return;
	}

	// transforming table points from table frame to "/BASE FRAME"

	tf::Transform table_tf;
	tf::poseMsgToTF(table_srv_.response.table.pose.pose,table_tf);
	Eigen::Matrix4f table_transform;
	sensor_msgs::PointCloud2 transform_table_cloud;


	pcl_ros::transformPointCloud(table_srv_.response.table.pose.header.frame_id,
			table_tf,table_srv_.response.table.table_points,
			transform_table_cloud);

	ROS_VERIFY(listener_.waitForTransform("/BASE", transform_table_cloud.header.frame_id,
			transform_table_cloud.header.stamp, ros::Duration(5.0)));
	ROS_VERIFY(pcl_ros::transformPointCloud("/BASE", transform_table_cloud,
			transform_table_cloud, listener_));

	pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(transform_table_cloud, *table_cloud_pcl);

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (table_cloud_pcl);
	seg.segment (*inliers, *table_coefficients_);

	got_table_ = true;
}

cv::Mat active_segment::constructVisGraph(cv::Mat input){

  cv::Mat draw(input);

  int count=0;

  for(std::vector<local_graph>::iterator node = graph_iter_.begin();
      node != graph_iter_.end() ; ++node){

	ROS_DEBUG("Finding Max Vertex %d",count);
    graph::Vertex_ros push_vertex = node->graph_.findMaxVertex();

    ROS_DEBUG("Max Vertex index %d",push_vertex.index_);

    for(graph::ros_graph::IGraph_it iter_= node->graph_.graph_.begin();
        iter_!=node->graph_.graph_.end(); ++iter_){

      // First point
      graph::Edge_ros edge = *iter_;
      cv::Point center_1(edge.edge_.first.x_,edge.edge_.first.y_);
      // Second point
      cv::Point center_2(edge.edge_.second.x_,edge.edge_.second.y_);

      // Display shenanigans
      cv::circle(draw,center_1, 5, cv::Scalar(128,0,128), -1);
      cv::circle(draw,center_2, 5, cv::Scalar(128,0,128), -1);
      cv::line(draw,center_1,center_2,cv::Scalar(128,0,0),1,0);

      if(push_vertex.index_ != edge.edge_.first.index_)
        cv::putText(draw, boost::lexical_cast<string>(edge.edge_.first.index_), center_1,
                    CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
      else
        if(count != 0)
          cv::putText(draw, " LMV "+boost::lexical_cast<string>(edge.edge_.first.index_), center_1,
                      CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
        else
          cv::putText(draw, "Max Vertex: "+boost::lexical_cast<string>(edge.edge_.first.index_), center_1,
                      CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);


      if(push_vertex.index_ != edge.edge_.second.index_)
        cv::putText(draw, boost::lexical_cast<string>(edge.edge_.second.index_), center_2,
                    CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
      else
        if(count != 0)
          cv::putText(draw, " LMV "+ boost::lexical_cast<string>(edge.edge_.second.index_), center_2,
                      CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
        else
          cv::putText(draw, "Max Vertex: "+ boost::lexical_cast<string>(edge.edge_.second.index_), center_2,
                      CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

    }
    count++;
  }


  if(DEBUG)
	  ROS_INFO("Number of Edges %d",count);
  if(!input_.empty())
    cv::imwrite("/tmp/full_graph.png",draw);

  return draw;
}

void active_segment::constructVisGraph(){

  cv::Mat disp_image = constructVisGraph(input_);
  if(!disp_image.empty())
    cv::imshow(window_thread_.c_str(), disp_image);
}

cv::MatND active_segment::computePatchFeature(cv::Mat input, cv::Mat mask){

  cv::MatND hist;
  cv::Mat hsv;

  ROS_DEBUG("Compute Patch Feature");

  cv::cvtColor(input,hsv,CV_BGR2HSV);

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

  cv::calcHist( &hsv, 1, channels, mask, // do not use mask
            hist, 2, histSize, ranges,
            true, // the histogram is uniform
            false );

  return hist;
}

void active_segment::updateGraphList(cv::Mat flow){

	unsigned char *input = (unsigned char*)(flow.data);
	// Updating the initial flow list
	std::vector<int> visited_index;

	for(std::vector<local_graph>::iterator node = graph_iter_.begin();
			node != graph_iter_.end() ; ++node){


		std::vector<geometry_msgs::Point> mean_dist;
		for(graph::ros_graph::IGraph_it iter_= node->graph_.graph_.begin();
				iter_!=node->graph_.graph_.end(); ++iter_){

			// First point
			graph::Edge_ros edge = *iter_;

			if(std::find(visited_index.begin(), visited_index.end(), edge.edge_.first.index_)!=visited_index.end()){
				visited_index.push_back(edge.edge_.first.index_);

				iter_->edge_.first.x_ += input[(int)(flow.cols*edge.edge_.first.y_+edge.edge_.first.x_)];
				iter_->edge_.first.y_ += input[(int)(flow.cols*edge.edge_.first.y_+edge.edge_.first.x_ + 1)];

				geometry_msgs::Point mean_pt;
				mean_pt.x = iter_->edge_.first.x_;mean_pt.y = iter_->edge_.first.y_;mean_pt.z=1;
				mean_dist.push_back(mean_pt);
			}
			if(std::find(visited_index.begin(), visited_index.end(), edge.edge_.second.index_)!=visited_index.end()){
				visited_index.push_back(edge.edge_.second.index_);


				iter_->edge_.second.x_ += input[(int)(flow.cols*edge.edge_.first.y_+edge.edge_.first.x_)];
				iter_->edge_.second.y_ += input[(int)(flow.cols*edge.edge_.first.y_+edge.edge_.first.x_ + 1)];

				geometry_msgs::Point mean_pt;
				mean_pt.x = iter_->edge_.first.x_;mean_pt.y = iter_->edge_.first.y_;mean_pt.z=1;
				mean_dist.push_back(mean_pt);
			}
		}

		// Use the optical flow update to compute the centroids update
		geometry_msgs::Point tot_sum;
		tot_sum.x = 0.0;tot_sum.y = 0.0;tot_sum.z = 0.0;
		int count = 0;
		for(std::vector<geometry_msgs::Point>::iterator pt_iter = mean_dist.begin();
				pt_iter !=mean_dist.end(); ++pt_iter)
		{
			tot_sum.x+=pt_iter->x; tot_sum.y+=pt_iter->y;
			count+=1;
		}

		tot_sum.x/=count;tot_sum.y/=count;
		node->centroid_.x = tot_sum.x; node->centroid_.y = tot_sum.y;
	}
}

void active_segment::TrackandUpdate(){

  // track a set of points using openCV
	cv::Mat flow;

	cv::Mat prev_gray,current_gray;

	cv::cvtColor(prev_input_, prev_gray, CV_BGR2GRAY);
	cv::cvtColor(input_, current_gray, CV_BGR2GRAY);

	cv::calcOpticalFlowFarneback(prev_gray,current_gray,flow,pyramid_scale_,levels_,win_size_,
			of_iter_,poly_pixel_,poly_sigma_,1);

	if(DEBUG){
		cv::Mat xy[2];
		cv::split(flow,xy);

		//calculate angle and magnitude
		cv::Mat magnitude, angle;
		cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

		//translate magnitude to range [0;1]
		double min_val, max_val;
		cv::minMaxLoc(magnitude, &min_val, &max_val);
		magnitude.convertTo(magnitude, -1, 1.0/max_val);

		//build hsv image
		cv::Mat _hsv[3], hsv;
		_hsv[0] = angle;
		_hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
		_hsv[2] = magnitude;
		cv::merge(_hsv, 3, hsv);

		//convert to BGR and show
		cv::Mat bgr;//CV_32FC3 matrix
		cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
		cv::imwrite("/tmp/opticalflow.jpg", bgr);

	}

	updateGraphList(flow);

}



void active_segment::getPushPoint(pcl::PointCloud<pcl::PointXYZ> push_ray,
		geometry_msgs::Point &push_loc){

    float t;
    t = -(table_coefficients_->values[3] + table_coefficients_->values[0]*push_ray.points[0].x +
    		table_coefficients_->values[1]*push_ray.points[0].y+ table_coefficients_->values[2]*push_ray.points[0].z);
    t /= (table_coefficients_->values[0]*push_ray.points[1].x +
    		table_coefficients_->values[1]*push_ray.points[1].y+ table_coefficients_->values[2]*push_ray.points[1].z);

    pcl::PointXYZ push_point;
    push_loc.x = t*push_ray.points[1].x;push_loc.y = t*push_ray.points[1].y;push_loc.z = t*push_ray.points[1].z;
}

void active_segment::projectVertex3DBASE(graph::Vertex_ros point,
		pcl::PointCloud<pcl::PointXYZ> &ray){

	// Project Vertex to 3D
	ROS_INFO("Entering base conversion function, pt.value x %f %f",point.x_,point.y_);
	ray.clear();
	cv::Point2d push_2d(point.x_,point.y_);
	cv::Point3d push_3d = left_cam_.projectPixelTo3dRay(push_2d); // getting push location in 3D
	ROS_INFO("finished projecting pixel to 3D");

	ray.push_back(pcl::PointXYZ(0,0,0));
	ray.push_back(pcl::PointXYZ((float)push_3d.x,(float)push_3d.y,(float)push_3d.z));
	ray.header = cam_info_.header;

	ROS_INFO("Creating Ray in base frame");
	ROS_VERIFY(listener_.waitForTransform("/BASE",cam_info_.header.frame_id,
			cam_info_.header.stamp, ros::Duration(5.0)));

	ROS_VERIFY(pcl_ros::transformPointCloud("/BASE", ray,
			ray, listener_));
	ROS_INFO("Converting cloud complete");

}


void active_segment::controlGraph(){

	// receive a new graph from
	if(convertToGraph()){

		ROS_INFO("Calling Static Segment service");
		// find the max node
		ROS_INFO_STREAM("Pushing max vertex");
		pcl::PointCloud<pcl::PointXYZ> push_3d_pcl;
		projectVertex3DBASE(graph_iter_[0].graph_.findMaxVertex(),push_3d_pcl);
		ROS_INFO_STREAM("The 3D pushing location in BASE FRAME IS is "<<push_3d_pcl.points[0]<<" "<<
				push_3d_pcl.points[1]);
		getPushPoint(push_3d_pcl,push_loc_);
		push_hand_pose_.header.stamp = ros::Time::now();
		push_hand_pose_.pose.position = push_loc_;
		// TODO: get orientation from recorded hand pose
		pose_publisher_.publish(push_hand_pose_);
		ROS_INFO("Published push hand pose location");
	}
}

void active_segment::buildGraphFromMsg(graph_queue& graph_list)
{
	ROS_INFO("Building graph from message");
    for(unsigned int i = 0; i < graph_msg_.size(); i++){
      local_graph single_graph;
      single_graph.graph_.buildGraph(graph_msg_[i].graph);
      //single_graph.graph_ = cluster_graph_; // need to define operator = or copy constructor
      single_graph.centroid_ = graph_msg_[i].centroid;
      single_graph.index_ = i; // TODO: Cheap trick to rearrange the image mask but we can worry
      //about that later once we figure out what the hell is going on
      graph_list.push(single_graph);
    }

}

int active_segment::findNearestNeighbour(geometry_msgs::Point vert,std::vector<local_graph> new_graph_iter){

	ROS_INFO("Finding Nearest neighbour");
	float min_dist = INFINITY;
	int index = 0;
	int count = 0;

	ROS_INFO("Search centroid is %f %f",vert.x,vert.y);
	for(std::vector<local_graph>::iterator node = new_graph_iter.begin();
			node != new_graph_iter.end() ; ++node){

		float dist = sqrt(((node->centroid_.x - vert.x)*(node->centroid_.x - vert.x))
				+ ((node->centroid_.y - vert.y)*(node->centroid_.y - vert.y)));

		ROS_INFO("Centroid %f %f distance is %f",node->centroid_.x,node->centroid_.y,dist);
		if(dist < min_dist){
			index = count;
			min_dist = dist;
		}

		count++;
	}
	ROS_INFO("Minimum distance is %f",min_dist);
	return index;

}

cv::MatND active_segment::getFeatureVector(cv::Mat input, cv::Mat segment, int index, cv::Mat cluster_mask){

	cv::Mat segment_mask = cluster_mask == (double)index;
	//segment.copyTo(segment_mask,cluster_mask);
	//segment_mask = cluster_mask == (double)index;
	cv::threshold(segment_mask, segment_mask, 20, 255, CV_THRESH_BINARY);

	if(DEBUG){
		std::stringstream ss;
		ss<<"/tmp/cluster_mask_"<<index<<"_"<<print_tag_<<".jpg";
		cv::imwrite(ss.str().c_str(),segment_mask);
	}

	// Computes appearance histogram for that patch
	return computePatchFeature(input,segment_mask);
}

bool active_segment::matchEdges(std::pair<int,int> old_edge,std::pair<int,int> new_edge,
		int index){

	ROS_INFO_STREAM("Matching Edges "<<old_edge.first<<","<<old_edge.second
			<<" and "<<new_edge.first<<","<<new_edge.second);

    // because we always look at the top of the priority queue
	print_tag_ = 1;
	cv::MatND oe_first = getFeatureVector(prev_input_,prev_segment_,old_edge.first,prev_masks_[0]);

	print_tag_ = 0;
	cv::MatND oe_second = getFeatureVector(prev_input_,prev_segment_,old_edge.second,prev_masks_[0]);

	print_tag_ = 1;
	ROS_INFO("Computing third histogram %d",index);
	cv::MatND ne_first = getFeatureVector(input_,segment_,new_edge.first,masks_[index]);
	print_tag_ = 0;
	cv::MatND ne_second = getFeatureVector(input_,segment_,new_edge.second,masks_[index]);

	// write huge dot product matching or normalized histogram matching difference
	//here comparing for both cases of edges

	double threshold = 0.10; // TODO:need to tune this threshold but how?? is 10% good enough

	ROS_INFO("Comparing Histograms Generic value :%f",cv::compareHist(oe_first,ne_first,CV_COMP_BHATTACHARYYA));

	if(cv::compareHist(oe_first,ne_first,CV_COMP_BHATTACHARYYA) < threshold &&
			cv::compareHist(oe_second,ne_second,CV_COMP_BHATTACHARYYA) < threshold)
		return true;
	else if(cv::compareHist(oe_second,ne_first,CV_COMP_BHATTACHARYYA) < threshold &&
			cv::compareHist(oe_first,ne_second,CV_COMP_BHATTACHARYYA) < threshold)
		return true;
	else
		return false;
}

bool active_segment::matchGraphs(local_graph base_graph,local_graph match_graph,
		int index){

	// Looping through edges
	ROS_INFO("Matching Graphs");

	if(DEBUG){
		cv::imwrite("/tmp/new_color.jpg",input_);
		cv::imwrite("/tmp/old_color.jpg",prev_input_);
		cv::imwrite("/tmp/old_segment.jpg",prev_segment_);
		cv::imwrite("/tmp/new_segment_color.jpg",segment_);
	}

	std::vector<int> visited_index;

	int match_score = 0;

	for(graph::ros_graph::IGraph_it iter_= base_graph.graph_.graph_.begin();
			iter_!=base_graph.graph_.graph_.end(); ++iter_){

		graph::Edge_ros edge = *iter_;

		// Check if edge has been visited
		if(std::find(visited_index.begin(), visited_index.end(), edge.edge_.first.index_) == visited_index.end()
				&& std::find(visited_index.begin(), visited_index.end(), edge.edge_.second.index_)== visited_index.end()){

			ROS_INFO("Entered clause");
			visited_index.push_back(edge.edge_.first.index_);
			visited_index.push_back(edge.edge_.second.index_);

			std::pair<int,int> org_edge = make_pair(edge.edge_.first.index_,edge.edge_.second.index_);

			bool matched = false;
			for(graph::ros_graph::IGraph_it iter_new_= match_graph.graph_.graph_.begin();
					iter_new_!=match_graph.graph_.graph_.end(); ++iter_new_){

				graph::Edge_ros edge_test = *iter_new_;
				std::pair<int,int> new_edge = make_pair(edge_test.edge_.first.index_,edge_test.edge_.second.index_);

				if(matchEdges(org_edge,new_edge,index))
					matched = true;
			}

			if(matched)
				match_score++;

		}
	}

	ROS_INFO("Visited vertices %d \n	match score %d vertices %d",
			visited_index.size(),match_score,(int)base_graph.graph_.number_of_vertices_/2);
	// If the number of edges matched are half the number of vertices, because edges are unique
	if(match_score >= (int)base_graph.graph_.number_of_vertices_/2)
		return true;
	else
		return false;
}

void active_segment::buildMaskList(std::vector<cv::Mat>& masks, std::vector<local_graph> graph_list,
		std::vector<sensor_msgs::Image> images){

	masks.clear();
	ROS_INFO("image_list %d",images.size(),
			masks.size());

	for(std::vector<local_graph>::iterator iter = graph_list.begin();iter != graph_list.end(); ++iter){
			cv::Mat current_image = returnCVImage(images[(int)(iter->index_)]);
			masks.push_back(current_image);
	}
	ROS_INFO("Confirmation call graph size %d image_list size %d",graph_list.size(),
			masks.size());

}


void active_segment::updateWithNewGraph(){

	graph_queue new_graph;
	std::vector<local_graph>& new_graph_iter(Container(new_graph));
	buildGraphFromMsg(new_graph);

	prev_masks_.clear();
	prev_masks_ = masks_;
	buildMaskList(masks_,new_graph_iter,staticsegment_srv_.response.cluster_masks);
	ROS_INFO("Updating the graph structure");

	ROS_INFO("Graph iterator size %d",graph_iter_.size());


	if(new_graph.size() != cluster_iter_.size()){

		ROS_INFO("Statisfied condition one");
		graph_list_ = new_graph;
		ROS_INFO("Manipulation has changed graph structure");
		cluster_iter_ = graph_iter_; // NOTE: this is not a iterator just a vector

	}
	else{

		ROS_INFO("Checking condition two ");
		geometry_msgs::Point v1 = graph_iter_[0].centroid_;
		//TODO: Making a big assumption here i.e that manipulating a subgraph only changes
		// that graph does not have an effect on others
		ROS_INFO_STREAM("Centroid "<<graph_iter_[0].centroid_);

		int index = findNearestNeighbour(v1,new_graph_iter);
		ROS_INFO("Returned match graph %d",index);
		local_graph sub_graph = new_graph_iter[index];

		std::vector<local_graph>::iterator node = graph_iter_.begin();
		if(matchGraphs(*node,sub_graph,index)){
			graph_list_.pop();
			ROS_INFO("Manipulation hasn't changed graph structure");
		}
		else{
			if(node->graph_.number_of_vertices_ == sub_graph.graph_.number_of_vertices_)
				*node = sub_graph;
			else{
				graph_list_.pop();
				ROS_INFO("Replacing graph structure with new graph");
				ROS_INFO("Iterator size %d",graph_iter_.size());
				graph_list_.push(sub_graph);
			}
		}
	}
}

bool active_segment::convertToGraph(){

  //Call static segmentation service
  bool call_succeeded = false;

  staticsegment_srv_.request.call = staticsegment_srv_.request.EMPTY;

  while(!call_succeeded){
    if(ros::service::call(static_service_,staticsegment_srv_)){
      call_succeeded = true;
      ROS_INFO("Service Call succeeded");

      //getting camera info
      sensor_msgs::CameraInfo::ConstPtr cam_info =
          ros::topic::waitForMessage<sensor_msgs::CameraInfo>(left_camera_topic_, nh_, ros::Duration(10.0));

      sensor_msgs::Image::ConstPtr recent_color =
            ros::topic::waitForMessage<sensor_msgs::Image>(rgb_topic_, nh_, ros::Duration(5.0));

      left_cam_.fromCameraInfo(cam_info); // Getting left camera info

      cam_info_= *cam_info;

      input_ = returnCVImage(*recent_color);

      segment_ =  returnCVImage(staticsegment_srv_.response.graph_image);

      graph_msg_ = staticsegment_srv_.response.graph_queue;

      cv::imwrite("/tmp/response_img.png",input_);

      if(first_call_){
    	  buildGraphFromMsg(graph_list_);
    	  cluster_iter_ = graph_iter_;
    	  masks_.clear();
    	  buildMaskList(masks_,graph_iter_,staticsegment_srv_.response.cluster_masks);
      }
      else
    	  updateWithNewGraph();
    }
  }

  if (staticsegment_srv_.response.result == staticsegment_srv_.response.FAILURE)
  {
    ROS_ERROR("Segmentation service returned error");
    return false;
  }

  ROS_INFO("Segmentation service succeeded. Returned Segmented Graph %d",staticsegment_srv_.response.result);

  // DEBUG of projection
  if(staticsegment_srv_.response.result == staticsegment_srv_.response.SUCCESS){
    ROS_INFO("Static Segment service call succeeded");
    return true;
  }

  return false;
}
}

int run_active_segmentation(int argc, char **argv){

	ros::init(argc, argv, "active_segment");
	ros::NodeHandle nh;
	active_segmentation::active_segment ss(nh);

	ss.initGraphHelpers();
	while(nh.ok()){

		ss.controlGraph();
		//visualize graph
		ROS_INFO("Displaying Graph");
		ss.constructVisGraph();
//		if (!ss.head_joint_trajectory_client_.lookAt(0.0,
//				0.0, ss.push_loc_))
//		{
//			ROS_ERROR("Problems when looking at stuff.");
//			return 0;
//		}

		// This needs to happen in a separate thread
		if(!ss.first_call_)
			ss.TrackandUpdate();
		else
	    	ss.first_call_ = false;

		ss.prev_input_ = ss.input_;
		ss.prev_segment_= ss.segment_;


		ros::spinOnce();
	}

  return 1;
}

int main(int argc, char **argv){

  return run_active_segmentation(argc,argv);

}
