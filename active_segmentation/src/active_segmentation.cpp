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
#include "active_segmentation/macros_time.hpp"
#include "tabletop_segmenter/TabletopSegmentation.h"
// PCL includes
#include "pcl17_ros/transforms.h"
#include <pcl17/filters/extract_indices.h>
#include <conversions/ros_to_tf.h>
#include <conversions/tf_to_ros.h>
#include <time.h>
#include <numeric>

#ifdef _OPENMP
#include <omp.h>
#endif

#define DEBUG false

namespace active_segmentation {

active_segment::active_segment(cv::Mat input, cv::Mat segment, graph_module::EGraph graph):
		marker_("/rviz_segment"),manipulation_object_l_(2),manipulation_object_r_(1),
		poke_object_(),world_state_()
	{//graph_iter_(Container(graph_list_)){

  segment_ = segment;
  input_ = input;

}

active_segment::active_segment(ros::NodeHandle & nh):
			    nh_(nh),nh_priv_("~"),//graph_iter_(Container(graph_list_)),
			    table_coefficients_(new pcl17::ModelCoefficients ()),first_call_(true),queue_empty_(false),
			    marker_("/rviz_segment"),manipulation_object_l_(2),manipulation_object_r_(1),
			    poke_object_(),world_state_(){

  nh_priv_.param<std::string>("static_service",static_service_,std::string("/static_segment_srv"));
  nh_priv_.param<std::string>("window_thread",window_thread_,std::string("Display window"));
  nh_priv_.param<std::string>("correspondence_thread",correspondence_thread_,std::string("Correspondence Window"));
  nh_priv_.param<bool>("tracking",tracking_,false);
  nh_priv_.param<std::string>("left_camera_topic",left_camera_topic_,std::string("/Honeybee/left/camera_info"));
  nh_priv_.param<std::string>("tabletop_service", tabletop_service_,std::string("/tabletop_segmentation"));
  nh_priv_.param<std::string>("rgb_input",rgb_topic_,std::string("/Honeybee/left/image_rect_color"));
  nh_priv_.param<std::string>("base_frame",base_frame_,std::string("/BASE"));

  pose_publisher_ = nh_priv_.advertise<geometry_msgs::PoseStamped>("/push_pose",5);

  cv::namedWindow( window_thread_.c_str(), CV_WINDOW_AUTOSIZE );// Create a window for display.
  cv::namedWindow( correspondence_thread_.c_str(), CV_WINDOW_AUTOSIZE );// Create a window for display.

  cv::startWindowThread();

  // Initializing some values
  pyramid_scale_ = 0.5;
  levels_ = 3;
  win_size_ = 11;
  of_iter_ = 5;
  poly_pixel_ = 6;
  poly_sigma_ = 1.25;
  global_counter_ = 0;
  push_hand_pose_.header.frame_id = base_frame_;

}



active_segment::~active_segment(){

	cv::destroyWindow(window_thread_.c_str());
	cv::destroyWindow(correspondence_thread_.c_str());
}

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


	pcl17_ros::transformPointCloud(table_srv_.response.table.pose.header.frame_id,
			table_tf,table_srv_.response.table.table_points,
			transform_table_cloud);

	ROS_VERIFY(listener_.waitForTransform(base_frame_, transform_table_cloud.header.frame_id,
			transform_table_cloud.header.stamp, ros::Duration(5.0)));
	ROS_VERIFY(pcl17_ros::transformPointCloud(base_frame_, transform_table_cloud,
			transform_table_cloud, listener_));

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

	// Create the filtering object
	pcl17::ExtractIndices<pcl17::PointXYZ> extract;

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



	ROS_DEBUG("Number of Edges %d",count);
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

void active_segment::trackAndUpdate(){

  // track a set of points using openCV
	cv::Mat flow;

	cv::Mat prev_gray,current_gray;

	cv::cvtColor(prev_input_, prev_gray, CV_BGR2GRAY);
	cv::cvtColor(input_, current_gray, CV_BGR2GRAY);

	cv::calcOpticalFlowFarneback(prev_gray,current_gray,flow,pyramid_scale_,levels_,win_size_,
			of_iter_,poly_pixel_,poly_sigma_,1);

	if(!DEBUG){
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



void active_segment::getPushPoint(pcl17::PointCloud<pcl17::PointXYZ> push_ray,
		geometry_msgs::Point &push_loc){

    float t;
    t = (table_coefficients_->values[3] + table_coefficients_->values[0]*push_ray.points[0].x +
    		table_coefficients_->values[1]*push_ray.points[0].y+ table_coefficients_->values[2]*push_ray.points[0].z);
    t /= (table_coefficients_->values[0]*push_ray.points[1].x +
    		table_coefficients_->values[1]*push_ray.points[1].y+ table_coefficients_->values[2]*push_ray.points[1].z);

    pcl17::PointXYZ push_point;
    push_loc.x = t*push_ray.points[1].x;push_loc.y = t*push_ray.points[1].y; push_loc.z = t*push_ray.points[1].z;

}

void active_segment::projectVertex3DBASE(graph::Vertex_ros point,
		pcl17::PointCloud<pcl17::PointXYZ> &ray){

	// Project Vertex to 3D
	ROS_DEBUG("Entering base conversion function, pt.value x %f %f",point.x_,point.y_);
	ray.clear();
	cv::Point2d push_2d(point.x_,point.y_);
	cv::Point3d push_3d = left_cam_.projectPixelTo3dRay(push_2d); // getting push location in 3D

	ray.push_back(pcl17::PointXYZ(0,0,0));
	ray.push_back(pcl17::PointXYZ((float)push_3d.x,(float)push_3d.y,(float)push_3d.z));
	ray.header = cam_info_.header;

	ROS_DEBUG("Creating Ray in base frame");
	ROS_VERIFY(listener_.waitForTransform(base_frame_,cam_info_.header.frame_id,
			ros::Time::now(), ros::Duration(5.0)));

	ROS_VERIFY(pcl17_ros::transformPointCloud(base_frame_, ray,
			ray, listener_));
	ROS_DEBUG("Converting cloud complete");

}

double active_segment::getPushDirection(const geometry_msgs::Pose &start_direction, geometry_msgs::Pose &push_dir){

	// TODO:think of a smart way to get the push direction
	std::vector<local_graph>::iterator node = graph_iter_.begin();

	geometry_msgs::Point start_point = start_direction.position;
	geometry_msgs::Point mean_point;
	mean_point.x = 0;mean_point.y = 0;mean_point.z = 0;
	int count = 0;

	for(graph::ros_graph::IGraph_it iter_= node->graph_.graph_.begin();
			iter_!=node->graph_.graph_.end(); ++iter_){

		pcl17::PointCloud<pcl17::PointXYZ> ray_1;
		projectVertex3DBASE(iter_->edge_.first,ray_1);
		geometry_msgs::Point push_loc_1;
		getPushPoint(ray_1,push_loc_1);
		mean_point.x += push_loc_1.x;
		mean_point.y += push_loc_1.y;
		mean_point.z += push_loc_1.z;


		pcl17::PointCloud<pcl17::PointXYZ> ray_2;
		projectVertex3DBASE(iter_->edge_.second,ray_2);
		geometry_msgs::Point push_loc_2;
		getPushPoint(ray_2,push_loc_2);
		mean_point.x += push_loc_2.x;
		mean_point.y += push_loc_2.y;
		mean_point.z += push_loc_2.z;

		count++;
	}

	mean_point.x /= (2*count); mean_point.y /= (2*count); mean_point.z /= (2*count);
	// converting mean point into mean vector
	mean_point.x -= start_direction.position.x; mean_point.y -= start_direction.position.y; mean_point.z -= start_direction.position.z;

	// Computing the yaw
	double theta = acos(mean_point.y/sqrt((mean_point.x*mean_point.x)
			+ (mean_point.y*mean_point.y)+ (mean_point.z*mean_point.z)));

	tf::Vector3 z_axis(0,0,1);
	tf::Pose start_direction_tf;
	conversions::convert(start_direction,start_direction_tf);

	tf::Vector3 local_z_axis = start_direction_tf.getBasis().inverse()*z_axis;
	tf::Matrix3x3 m(tf::Quaternion(local_z_axis,-theta));
	tf::Transform pose_transform = tf::Transform::getIdentity();
	pose_transform.setBasis(m);

	tf::Pose new_pose = start_direction_tf*pose_transform;
	conversions::convert(new_pose,push_dir);

	return theta;


}

bool active_segment::graspNode(geometry_msgs::PoseStamped push_pose, ArmInterface& manipulation_object){

	ROS_INFO("Converting pose to TF");
	tf::Pose push_tf;
	tf::poseMsgToTF(push_pose.pose,push_tf);
	ROS_INFO("Converting pose complete");

	// Move to a position that is 10cm above the grasp point
	push_tf.getOrigin().setZ(push_tf.getOrigin().getZ() + 0.10);

	// Moving to desired initial pose
	poke_object_.initialize(manipulation_object.getEndeffectorId());
	world_state_.createTable();
	ROS_INFO("Creating table complete");

	// opening hand
	if(manipulation_object.openFingersAndVerify())
	{
		geometry_msgs::Pose poke_pose;
		poke_pose.position = push_pose.pose.position;
		poke_pose.position.z = push_pose.pose.position.z - 0.10;
		poke_pose.orientation = push_pose.pose.orientation; // This would have to be 1,0,0,0
		ROS_INFO("Fingers opened");


		if(!DEBUG){
			if(manipulation_object.planAndMoveTo(push_tf)){

				// Move to poke position
				geometry_msgs::Pose poke_position;
				bool poked, is_pose_relative;
				ROS_INFO("Planning and moved to positions");
				manipulation_object.ft_sensor_.calibrate();
				manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
				ROS_INFO("Switching controller stack complete");
				if(poke_object_.run(poke_pose,5.0,3.0,poke_position,poked,is_pose_relative))
				{
					geometry_msgs::Pose pick_position;
					pick_position = poke_position;
					pick_position.position.z += 0.05;//Move  5cm above pick point
					ROS_INFO("poke Successful");
					bool success;
					manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
					success = manipulation_object.cartesian_.moveTo(pick_position,3.0);
					ROS_INFO("Moving Cartesian");
					if(success){

						// manipulation object closing Hand
						double mypos[] = {0,2.4,2.4,2.4};
						std::vector<double> hand_joint_positions (mypos, mypos + sizeof(mypos) / sizeof(double) );
						double myforces[] = {-0.5,-0.5,-0.5};
						std::vector<double> hand_joint_forces (myforces, myforces + sizeof(myforces) / sizeof(double) );
						success = manipulation_object.hand_joints_.sendPositionsForces(hand_joint_positions,hand_joint_forces,5.0);
						ROS_INFO("Closing Hands");
						if(success)
						{
							// Now checking the joint angles to check if the Hand is completely closed
							manipulation_object.getHandJointPositions(hand_joint_positions);
							double joint_sum = 0;
							for(int i = 1; i< hand_joint_positions.size() ;i++)
								joint_sum+= hand_joint_positions[i];
							if(joint_sum < 6.0)
							{
								bool place_success = false;
								geometry_msgs::Pose place_position;
								place_position.position = push_pose.pose.position;
								place_position.orientation = push_pose.pose.orientation;
								// Dropping from 5 cn above the ground
								place_position.position.z += 0.05;
								// Now move to a position that is 20 cm away
								double move_x_increment;
								if(manipulation_object.getEndeffectorId() == 2)
									move_x_increment = -0.35;
								else
									move_x_increment = 0.35;
								// Move to a position away from the object
								place_position.position.x += move_x_increment;
								ROS_INFO("Planning to place position");
								do{
									ROS_INFO("Trying to place again");
									tf::Pose place_pose_tf;
									tf::poseMsgToTF(place_position,place_pose_tf);
									if(manipulation_object.planAndMoveTo(place_pose_tf))
									{
										manipulation_object.openFingers();
										ROS_INFO("Dropping object");
										place_success = true;
									}
									else
										place_position.position.y += 0.05;

								}while(!place_success);

								return manipulation_object.goHome();
							}
							else
							{
								// Open fingers and go Home
								ROS_WARN("Couldn't Grasp Object!");
								manipulation_object.openFingers();
								manipulation_object.goHome();
								return false;
							}
						}
						else{
							ROS_WARN("Unable to close hands!");
							manipulation_object.openFingers();
							manipulation_object.goHome();
							return false;
						}
					}
					else
						return false;
				}
				else
					return false;
			}
			else
				return false;
		}
		else{

			if(manipulation_object.planAndMoveTo(push_tf)){

				geometry_msgs::Pose pick_position;
				//TODO: Do something smarter
				pick_position = push_pose.pose;
				pick_position.position.z += 0.05; // Go 5cm away above the body
				bool success;
				manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
				success = manipulation_object.cartesian_.moveTo(pick_position,3.0);

				// Now do a blind Grasp

				if(success){

					// manipulation object closing Hand
					double mypos[] = {0,2.4,2.4,2.4}; // Dont alter the spread just the other joint angles
					std::vector<double> hand_joint_positions (mypos, mypos + sizeof(mypos) / sizeof(double) );
					double myforces[] = {-0.5,-0.5,-0.5};
					std::vector<double> hand_joint_forces (myforces, myforces + sizeof(myforces) / sizeof(double) );
					success = manipulation_object.hand_joints_.sendPositionsForces(hand_joint_positions,hand_joint_forces,5.0);

					if(success)
					{
						// Now checking the joint angles to check if the Hand is completely closed
						manipulation_object.getHandJointPositions(hand_joint_positions);
						double joint_sum = 0;
						for(int i = 1; i< hand_joint_positions.size() ;i++)
							joint_sum+= hand_joint_positions[i];
						if(joint_sum < 6.0)
						{
							bool place_success = false;
							geometry_msgs::Pose place_position;
							place_position.position = push_pose.pose.position;
							place_position.orientation = push_pose.pose.orientation;
							// Now move to a position that is 20 cm away
							double move_x_increment;
							if(manipulation_object.getEndeffectorId() == 2)
								move_x_increment = -0.25;
							else
								move_x_increment = 0.25;
							// Move to a position away from the object
							place_position.position.x += move_x_increment;
							do{
								tf::Pose place_pose_tf;
								tf::poseMsgToTF(place_position,place_pose_tf);
								if(manipulation_object.planAndMoveTo(place_pose_tf))
								{
									manipulation_object.openFingers();
									place_success = true;
								}
								else
									place_position.position.y += 0.05;

							}while(!place_success);

							return manipulation_object.goHome();
						}
						else
						{
							// Open fingers and go Home
							ROS_WARN("Couldn't Grasp Object!");
							manipulation_object.openFingers();
							return manipulation_object.goHome();
						}
					}
					else{
						ROS_WARN("Unable to close hands!");
						manipulation_object.openFingers();
						manipulation_object.goHome();
						return false;
					}
				}
				else
					return false;
			}
			else
				return false;
		}
	}
	else{
		ROS_WARN("Unable to open hands!");
		return false;
	}
}

bool active_segment::pushNode(geometry_msgs::PoseStamped push_pose, double y_dir, ArmInterface& manipulation_object){

	tf::Pose push_tf;
	tf::poseMsgToTF(push_pose.pose,push_tf);
	manipulation_object.goHome();
	// Move to a position that is 10cm behind and 10cm above the desired location
	push_tf.getOrigin().setY(push_tf.getOrigin().getY() - 0.10);
	push_tf.getOrigin().setZ(push_tf.getOrigin().getZ() + 0.10);

	manipulation_object.goHome(); // Just to go home between runs

	poke_object_.initialize(manipulation_object.getEndeffectorId());
	// Moving to desired initial pose
    world_state_.createTable();


	geometry_msgs::Pose poke_pose;
	poke_pose.position = push_pose.pose.position;
	poke_pose.position.z = push_pose.pose.position.z - 0.10;
	poke_pose.orientation = push_pose.pose.orientation;

	if(!DEBUG){
		if(manipulation_object.planAndMoveTo(push_tf)){

			// Remove comment when running on robot
			geometry_msgs::Pose poke_position;
			bool poked, is_pose_relative;
			manipulation_object.ft_sensor_.calibrate();
			manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
			if(poke_object_.run(poke_pose,5.0,3.0,poke_position,poked,is_pose_relative))
			{
				geometry_msgs::Pose push_position;
				push_position = poke_position;
				//y dir vector is [sin(t),cos(t),0]
				push_position.position.y += 0.20*(cos(y_dir)); // Push it 20cm away from the body

				if(manipulation_object.getEndeffectorId() == 2)
					push_position.position.x -= 0.20*(sin(y_dir)); // Push it 20cm away from the body
				else
					push_position.position.x += 0.20*(sin(y_dir)); // Push it 20cm away from the body

				bool success;
				manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
				success = manipulation_object.cartesian_.moveTo(push_position,3.0);

				if(success)
					return manipulation_object.goHome();
				else
					return false;
			}
			else
				return false;
		}
		else
			return false;
	}
	else{

		if(manipulation_object.planAndMoveTo(push_tf)){

			geometry_msgs::Pose push_position;
			//TODO: Do something smarter
			push_position = push_pose.pose;
			push_position.position.y += 0.05; // Push it 5cm away from the body
			bool success;
			manipulation_object.switchControl(ArmInterface::CARTESIAN_CONTROL_);
			success = manipulation_object.cartesian_.moveTo(push_position,3.0);

			if(success)
				return manipulation_object.goHome();
			else
				return false;
		}
		else
			return false;
	}
}

bool active_segment::manipulateNode(const geometry_msgs::Point &push_loc){

	tf::Vector3 x_axis(1,0,0);
	tf::Vector3 z_axis(0,0,1); // For grasp reorientation

	// Computing the push pose, i.e rotating Hand around x axis by 90 degrees
	geometry_msgs::PoseStamped push_pose;
	push_pose.header.stamp = ros::Time::now();
	push_pose.pose.position = push_loc;

	// Recorded Hand pose
//	push_hand_pose_.pose.orientation.w = -0.416;
//	push_hand_pose_.pose.orientation.x = 0.560;
//	push_hand_pose_.pose.orientation.y = 0.556;
//	push_hand_pose_.pose.orientation.z = 0.452;

	push_pose.pose.orientation.w = 1;
	push_pose.pose.orientation.x = 0;
	push_pose.pose.orientation.y = 0;
	push_pose.pose.orientation.z = 0;

	// Getting the correct push direction pose from the graph structure
	// And pushing y axis
	double y_dir = getPushDirection(push_pose.pose,push_pose.pose);

	tf::Pose push_pose_tf;
	tf::poseMsgToTF(push_pose.pose,push_pose_tf);

	double push_theta = M_PI/2;
	tf::Vector3 local_push_x_axis = push_pose_tf.getBasis().inverse()*x_axis;
	tf::Matrix3x3 m_push(tf::Quaternion(local_push_x_axis,push_theta));
	tf::Transform push_pose_transform = tf::Transform::getIdentity();
	push_pose_transform.setBasis(m_push);
	tf::Pose new_push_pose = push_pose_tf*push_pose_transform;
	conversions::convert(new_push_pose,push_pose.pose);

	marker_.publishPose(push_pose,"active_segmentation",0.2);
	pose_publisher_.publish(push_pose);
	ROS_INFO("Published push hand pose location");

	// Now computing the GRASP Pose
	geometry_msgs::PoseStamped grasp_pose;
	grasp_pose = push_pose;
	grasp_pose.pose.orientation.w = 1.0;
	grasp_pose.pose.orientation.z = 0.0;
	grasp_pose.pose.orientation.y = 0.0;
	grasp_pose.pose.orientation.x = 0.0;

	manipulation_object_l_.goHome(); // Just to go home between runs
	manipulation_object_r_.goHome(); // Just to go home between runs
	// First get the Hand positions
	tf::Pose object_pose;
	tf::poseMsgToTF(grasp_pose.pose,object_pose);

	//Now rotate the hand (object_pose) by 180 degrees
	// Computing the yaw
	double theta = M_PI;
	tf::Vector3 local_x_axis = object_pose.getBasis().inverse()*x_axis;
	tf::Matrix3x3 m_x(tf::Quaternion(local_x_axis,theta));
	tf::Transform grasp_x_pose_transform = tf::Transform::getIdentity();
	grasp_x_pose_transform.setBasis(m_x);
	tf::Pose new_object_pose = object_pose*grasp_x_pose_transform;
	conversions::convert(new_object_pose,grasp_pose.pose);

	// Now rotate the GRASP Pose in the local axis by z-dir
	// TODO: Use to the local point cloud template based on the max
	// radius of the hand and compute eigen values and align with the direction
	// that does not correspond to the dominant eigen value i.e thinner cloud
	double psi = -y_dir;
	tf::Vector3 local_z_axis = object_pose.getBasis().inverse()*z_axis;
	tf::Matrix3x3 m_z(tf::Quaternion(local_z_axis,psi));
	tf::Transform grasp_z_pose_transform = tf::Transform::getIdentity();
	grasp_z_pose_transform.setBasis(m_z);
	new_object_pose = object_pose*grasp_z_pose_transform;
	conversions::convert(new_object_pose,grasp_pose.pose);


	// Checking which hand is closer to the manipulation point

	tf::Pose l_tfhand_pose,r_tfhand_pose;
	manipulation_object_l_.getHandPose(l_tfhand_pose);
	manipulation_object_r_.getHandPose(r_tfhand_pose);

	double distance_r = l_tfhand_pose.getOrigin().distance(new_object_pose.getOrigin());
	double distance_l = r_tfhand_pose.getOrigin().distance(new_object_pose.getOrigin());
	ROS_INFO_STREAM("Distance from Right Hand "<<distance_r<<" Distance from Left Hand "<<distance_l);
	std::cin.ignore(std::numeric_limits<streamsize>::max(),'\n');

	if(distance_r < distance_l)
	{
		ROS_INFO("Manipulating with Right hand");
		// Manipulate with right hand/arm
		if(graspNode(grasp_pose,manipulation_object_r_))
		{
			ROS_INFO("Grasping Succeeded ");
			return true;


		}
		else{

			ROS_INFO("Grasping with right hand failed, Attempting push");
			if(pushNode(push_pose,y_dir,manipulation_object_r_)){
				ROS_INFO("Pushing Succeeded ");
				return true;
			}
			else{
				ROS_INFO("Pushing Failed too");
				return false;
			}

		}

	}
	else{
		// Use the left hand for manipulation
		ROS_INFO("Manipulating with Left hand");
		if(graspNode(grasp_pose,manipulation_object_l_))
		{
			ROS_INFO("Grasping Succeeded ");
			return true;


		}
		else{

			ROS_INFO("Grasping with left hand failed, Attempting push");
			if(pushNode(push_pose,y_dir,manipulation_object_l_)){
				ROS_INFO("Pushing Succeeded ");
				return true;
			}
			else{
				ROS_INFO("Pushing Failed too");
				return false;
			}

		}
	}

}


void active_segment::controlGraph(){

	// receive a new graph from
	if(convertToGraph() && !queue_empty_){

		// Checking if priority queue is empty
		if(graph_list_.empty()){
			queue_empty_ = true;
			return;
		}
    	// find the max node
		ROS_DEBUG("Pushing max vertex");
		pcl17::PointCloud<pcl17::PointXYZ> push_3d_pcl;
		projectVertex3DBASE(graph_iter_[0].graph_.findMaxVertex(),push_3d_pcl);
//		geometry_msgs::PoseStamped view_pose;
		ROS_INFO_STREAM("The 3D pushing location in BASE FRAME IS is "<<push_3d_pcl.points[0]<<" "<<
				push_3d_pcl.points[1]);
		getPushPoint(push_3d_pcl,push_loc_);

//		view_pose.header.frame_id = base_frame_;
//		view_pose.header.stamp = ros::Time::now();
//		view_pose.pose.position.x = push_loc_.x;
//		view_pose.pose.position.y = push_loc_.y;
//		view_pose.pose.position.z = push_loc_.z;
//		view_pose.pose.orientation.w = 1;
//		view_pose.pose.orientation.z = 0;
//		view_pose.pose.orientation.y = 0;
//		view_pose.pose.orientation.x = 0;

//		ROS_INFO_STREAM("The 3D pushing location in BASE FRAME IS is "<<push_loc_);
//
//		marker_.publishPose(view_pose,"active_segmentation",0.2);


		if(manipulateNode(push_loc_))
			ROS_INFO("Node push success");
		else
			ROS_WARN("Node push Failed");

	}
}

void active_segment::buildGraphFromMsg(graph_queue& graph_list)
{
	for(unsigned int i = 0; i < graph_msg_.size(); i++){

		local_graph single_graph;
		single_graph.graph_.buildGraph(graph_msg_[i].graph);
		single_graph.centroid_ = graph_msg_[i].centroid;
		single_graph.index_ = i;
		graph_list.push(single_graph);

	}

	ROS_DEBUG("Built graph from message of size %d",graph_list.size());

}

int active_segment::findNearestNeighbour(geometry_msgs::Point vert, std::vector<local_graph> new_graph_iter){

	ROS_INFO("Finding Nearest neighbour");
	float min_dist = INFINITY;
	int index = 0;
	int count = 0;

	ROS_DEBUG("Search centroid is %f %f",vert.x,vert.y);
	for(std::vector<local_graph>::iterator node = new_graph_iter.begin();
			node != new_graph_iter.end() ; ++node){

		float dist = sqrt(((node->centroid_.x - vert.x)*(node->centroid_.x - vert.x))
				+ ((node->centroid_.y - vert.y)*(node->centroid_.y - vert.y)));

		ROS_DEBUG("Centroid %f %f distance is %f",node->centroid_.x,node->centroid_.y,dist);
		if(dist < min_dist){
			index = count;
			min_dist = dist;
		}

		count++;
	}
	ROS_DEBUG("Minimum distance is %f",min_dist);
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
		int index,bool &inverted){

	if(DEBUG)
		ROS_INFO_STREAM("Matching Edges "<<old_edge.first<<","<<old_edge.second
				<<" and "<<new_edge.first<<","<<new_edge.second);

    // because we always look at the top of the priority queue
	print_tag_ = 1;
	cv::MatND oe_first = getFeatureVector(prev_input_,prev_segment_,old_edge.first,masks_[0]);
	cv::normalize(oe_first,oe_first,255);

	print_tag_ = 0;
	cv::MatND oe_second = getFeatureVector(prev_input_,prev_segment_,old_edge.second,masks_[0]);
	cv::normalize(oe_second,oe_second,255);

	print_tag_ = 1;
	ROS_DEBUG("Computing third histogram %d",index);
	cv::MatND ne_first = getFeatureVector(input_,segment_,new_edge.first,new_masks_[index]);
	cv::normalize(ne_first,ne_first,255);
	print_tag_ = 0;
	cv::MatND ne_second = getFeatureVector(input_,segment_,new_edge.second,new_masks_[index]);
	cv::normalize(ne_second,ne_second,255);

	// write huge dot product matching or normalized histogram matching difference
	//here comparing for both cases of edges

	double threshold = 0.55; // TODO : need to tune this threshold but how?? is 10% good enough

	ROS_DEBUG("Comparing Histograms 1 - 1 value :%f",cv::compareHist(oe_first,ne_first,CV_COMP_BHATTACHARYYA));
	ROS_DEBUG("Comparing Histograms 2 - 2 value :%f",cv::compareHist(oe_second,ne_second,CV_COMP_BHATTACHARYYA));
	ROS_DEBUG("Comparing Histograms 2 - 1 value :%f",cv::compareHist(oe_second,ne_first,CV_COMP_BHATTACHARYYA));
	ROS_DEBUG("Comparing Histograms 1 - 2 value :%f",cv::compareHist(oe_first,ne_second,CV_COMP_BHATTACHARYYA));

	if(cv::compareHist(oe_first,ne_first,CV_COMP_BHATTACHARYYA) < threshold &&
			cv::compareHist(oe_second,ne_second,CV_COMP_BHATTACHARYYA) < threshold){

		inverted = false;
		return true;
	}
	else if(cv::compareHist(oe_second,ne_first,CV_COMP_BHATTACHARYYA) < threshold &&
			cv::compareHist(oe_first,ne_second,CV_COMP_BHATTACHARYYA) < threshold){

		inverted = true;
		return true;
	}
	else
		return false;
}

void active_segment::drawCorrespondence(cv::Mat input_l,cv::Mat input_r,cv::Mat mask_l,	cv::Mat mask_r,
		std::vector< std::pair< std::pair<float,float> , std::pair<float,float> > > correspondences){

	cv::Mat img1,img2;
	input_l.copyTo(img1,mask_l);
	input_r.copyTo(img2,mask_r);

	cv::Mat imgResult(img1.rows,2*img1.cols,img1.type()); // Your final image

	cv::Mat roiImgResult_Left = imgResult(cv::Rect(0,0,img1.cols,img1.rows)); //Img1 will be on the left part
	cv::Mat roiImgResult_Right = imgResult(cv::Rect(img1.cols,0,img2.cols,img2.rows)); //Img2 will be on the right part, we shift the roi of img1.cols on the right

	cv::Mat roiImg1 = img1(cv::Rect(0,0,img1.cols,img1.rows));
	cv::Mat roiImg2 = img2(cv::Rect(0,0,img2.cols,img2.rows));

	roiImg1.copyTo(roiImgResult_Left); //Img1 will be on the left of imgResult
	roiImg2.copyTo(roiImgResult_Right); //Img2 will be on the right of imgResult

	for(unsigned int i=0 ; i < correspondences.size() ; i++){

		cv::Point center_1(correspondences[i].first.first,correspondences[i].first.second);
		// Second point
		cv::Point center_2(correspondences[i].second.first+img1.cols,correspondences[i].second.second);
		// Display shenanigans
		cv::circle(imgResult,center_1, 5, cv::Scalar(128,0,128), -1);
		cv::circle(imgResult,center_2, 5, cv::Scalar(128,0,128), -1);
		cv::line(imgResult,center_1,center_2,cv::Scalar(0,128,0),1,0);
	}

	if(!DEBUG)
		cv::imwrite("/tmp/correspondence.jpg",imgResult);
	cv::imshow(correspondence_thread_.c_str(), imgResult);
}


bool active_segment::matchGraphs(local_graph base_graph,local_graph match_graph,
		int index){

	// Looping through edges
	ROS_INFO("Matching Graphs");

	if(!DEBUG){
		cv::imwrite("/tmp/new_color.jpg",input_);
		cv::imwrite("/tmp/old_color.jpg",prev_input_);
		cv::imwrite("/tmp/old_segment.jpg",prev_segment_);
		cv::imwrite("/tmp/new_segment_color.jpg",segment_);

		std::stringstream ss_1,ss_2;
		ss_1<<"/tmp/input_mask_"<<global_counter_<<".jpg";
		ss_2<<"/tmp/prev_mask_"<<global_counter_<<".jpg";
		cv::imwrite(ss_1.str().c_str(),new_masks_[index]);
		cv::imwrite(ss_2.str().c_str(),masks_[0]);

		global_counter_++;
	}

	//std::vector<int> visited_index;

	std::vector< std::pair< std::pair<float,float> , std::pair<float,float> > > correspondences;

	std::vector<int> match_score;
	//unsigned int graph_size = base_graph.graph_.graph_.size();
	//int iteration = 0;
	match_score.resize(base_graph.graph_.graph_.size());
	// Parallelizing the edge matching
	// timer to measure graph matching time
	INIT_PROFILING

	std::vector<graph::ros_graph::IGraph_it > base_graph_iter;

	for(graph::ros_graph::IGraph_it iter_= base_graph.graph_.graph_.begin();iter_!=base_graph.graph_.graph_.end();
			++iter_)
		base_graph_iter.push_back(iter_);


#pragma omp parallel for
	for(int iteration= 0; iteration< base_graph_iter.size(); iteration++){

		graph::Edge_ros edge = *base_graph_iter[iteration]; //*iter_;

		// Check if edge has been visited
		//		if(std::find(visited_index.begin(), visited_index.end(), edge.edge_.first.index_) == visited_index.end()
		//				&& std::find(visited_index.begin(), visited_index.end(), edge.edge_.second.index_)== visited_index.end()){

		ROS_DEBUG("Entered clause");
		//visited_index.push_back(edge.edge_.first.index_);
		//visited_index.push_back(edge.edge_.second.index_);

		std::pair<int,int> org_edge = make_pair(edge.edge_.first.index_,edge.edge_.second.index_);

		bool matched = false;

		for(graph::ros_graph::IGraph_it iter_new_= match_graph.graph_.graph_.begin();
				iter_new_!=match_graph.graph_.graph_.end(); ++iter_new_){

			graph::Edge_ros edge_test = *iter_new_;
			std::pair<int,int> new_edge = make_pair(edge_test.edge_.first.index_,edge_test.edge_.second.index_);

			bool inverted;
			if(matchEdges(org_edge,new_edge,index,inverted)){
				matched = true;
				if(inverted)
				{
					correspondences.push_back(make_pair(make_pair(edge.edge_.first.x_,edge.edge_.first.y_)
							,make_pair(edge_test.edge_.second.x_,edge_test.edge_.second.y_)));
					correspondences.push_back(make_pair(make_pair(edge.edge_.second.x_,edge.edge_.second.y_)
							,make_pair(edge_test.edge_.first.x_,edge_test.edge_.first.y_)));
				}
				else
				{
					correspondences.push_back(make_pair(make_pair(edge.edge_.first.x_,edge.edge_.first.y_)
							,make_pair(edge_test.edge_.first.x_,edge_test.edge_.first.y_)));
					correspondences.push_back(make_pair(make_pair(edge.edge_.second.x_,edge.edge_.second.y_)
							,make_pair(edge_test.edge_.second.x_,edge_test.edge_.second.y_)));
				}
				break; // If an edge is matched break out of the loop
			}
		}

		if(matched)
			match_score[iteration] = 1;
	}


	MEASURE("graph matching")


	// TODO: This ia very disgusting hack but let's see if it works
	ROS_INFO_STREAM("Match Score is "<<std::accumulate(match_score.begin(), match_score.end(), 0));
	//exit(0);
	//	ROS_INFO("Visited vertices %d \n match score %d vertices %f",
	//			visited_index.size(),match_score,std::floor((double)base_graph.graph_.number_of_vertices_/2));
	// If the number of edges matched are half the number of vertices, because edges are unique
	if((std::accumulate(match_score.begin(), match_score.end(), 0) >= std::floor((double)base_graph.graph_.number_of_vertices_/2) - 1) &&
			std::accumulate(match_score.begin(), match_score.end(), 0) > 0)
	{
		drawCorrespondence(prev_input_,input_,masks_[0],new_masks_[index],correspondences);
		return true;
	}

	else{
		ROS_INFO(" No Correspondences ");
		return false;
	}
}

void active_segment::buildMaskList(std::vector<cv::Mat>& masks, std::vector<local_graph> graph_list,
		std::vector<sensor_msgs::Image> images){

	masks.clear();
	ROS_DEBUG("image_list %d mask_list %d",images.size(),masks.size());
	masks.resize(graph_list.size());
    int i=0;
	for(std::vector<local_graph>::iterator iter = graph_list.begin();iter != graph_list.end(); ++iter){
		//cv::Mat current_image = returnCVImage(images[(int)(iter->index_)]);
		//masks.push_back(current_image);
		masks[i] = returnCVImage(images[(int)(iter->index_)]); i++;
	}
	ROS_DEBUG("Confirmation call graph size %d image_list size %d",graph_list.size(), images.size());

}

void active_segment::updateMaskList(std::vector<cv::Mat>& masks,std::vector<local_graph> graph_list){

	std::vector<cv::Mat> mask_list;
	mask_list.clear();
	mask_list.resize(graph_list.size());

	int i = 0;
	for(std::vector<local_graph>::iterator iter = graph_list.begin();iter != graph_list.end(); ++iter){
		mask_list[i] = masks[iter->index_];
		i++;
	}
	masks.clear();masks.resize(mask_list.size());
	copy(mask_list.begin(),mask_list.end(),masks.begin());
}

void active_segment::convertGraphToIter(graph_queue graph_list, std::vector<local_graph>& graph_iter){

	std::vector<local_graph> temp_graph = graph_list.getVector();
	graph_iter.clear();
	graph_iter.resize(temp_graph.size());
	copy(temp_graph.begin(),temp_graph.end(),graph_iter.begin());
}


void active_segment::updateWithNewGraph(){

	graph_queue new_graph;
	std::vector<local_graph> new_graph_iter;
	buildGraphFromMsg(new_graph);
	new_graph_iter = new_graph.getVector();

	buildMaskList(new_masks_,new_graph_iter,staticsegment_srv_.response.cluster_masks);
	ROS_INFO("Updating the graph structure");
	convertGraphToIter(graph_list_,graph_iter_);
	//updateMaskList(new_masks_,graph_iter_);

//	ss.prev_masks_.clear();ss.prev_masks_.resize(ss.masks_.size());
//	copy(ss.masks_.begin(),ss.masks_.end(),ss.prev_masks_.begin());

	if(new_graph.size() != cluster_iter_.size()){

		ROS_INFO("Statisfied condition one");
		graph_list_ = new_graph;
		masks_ = new_masks_; // Updating masks_list_
		convertGraphToIter(graph_list_,graph_iter_);
		ROS_INFO("Manipulation has changed graph structure");
		cluster_iter_.clear();cluster_iter_.resize(graph_iter_.size());
		copy(graph_iter_.begin(),graph_iter_.end(),cluster_iter_.begin());

	}
	else{

		ROS_INFO("Checking condition two ");
		geometry_msgs::Point v1 = graph_iter_[0].centroid_;
		//TODO: Making a big assumption here i.e that manipulating a subgraph only changes
		// that graph does not have an effect on others
		int index = findNearestNeighbour(v1,new_graph_iter);

		ROS_DEBUG("Returned match graph %d",index);
		local_graph sub_graph = new_graph_iter[index];

		ROS_INFO_STREAM("Original centroid :"<<v1<<std::endl<<" Returned Centroid :"<<sub_graph.centroid_);

		std::vector<local_graph>::iterator node = graph_iter_.begin();
		if(matchGraphs(*node,sub_graph,index)){

			graph_list_.pop();
			masks_.erase(masks_.begin()); // removing the front element
			ROS_INFO("Manipulation hasn't changed graph structure");
			convertGraphToIter(graph_list_,graph_iter_);
		}
		else{
			graph_list_.pop();
			ROS_INFO("Replacing graph structure with new graph");
			graph_list_.push(sub_graph);
			masks_[0] = new_masks_[index];
			convertGraphToIter(graph_list_,graph_iter_);
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
				convertGraphToIter(graph_list_,graph_iter_);
				cluster_iter_.clear(); cluster_iter_.resize(graph_iter_.size());
				copy(graph_iter_.begin(),graph_iter_.end(),cluster_iter_.begin());
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
	arm_controller_interface::init();
	ros::NodeHandle nh;
	ros::AsyncSpinner mts(0);
	active_segmentation::active_segment ss(nh);

	mts.start();
	ss.initGraphHelpers();
	while(nh.ok()){

		ss.controlGraph();
		//visualize graph
		ROS_INFO("Displaying Graph");
		ss.constructVisGraph();

		if(ss.queue_empty_){
			ROS_INFO("Segmentation complete");
			cv::waitKey();
			return 1;
		}


		if (!ss.head_joint_trajectory_client_.lookAt(0.0,
				0.0, ss.push_loc_))
		{
			ROS_ERROR("Problems when looking at stuff.");
			return 0;
		}

		// This needs to happen in a separate thread
		if(!ss.first_call_)
			ss.trackAndUpdate();
		else
			ss.first_call_ = false;

		// Updating the images and masks
		ss.prev_input_ = ss.input_;
		ss.prev_segment_= ss.segment_;
	}

	return 0;
}

int main(int argc, char **argv){

  return run_active_segmentation(argc,argv);

}
